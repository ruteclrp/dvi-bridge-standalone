import json
import os
import threading
import time
import tempfile

from flask import Flask, jsonify, request

from entity_map import ENTITY_MAP

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WWW_DIR = os.path.join(BASE_DIR, "www")

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

STATE_PATH = "./../state.json"
COMMANDS_PATH = "./../commands.json"
STATE_POLL_INTERVAL = 1.0  # seconds

# -----------------------------------------------------------------------------
# Global state (HA-style)
# -----------------------------------------------------------------------------

states = {}
state_lock = threading.Lock()
commands_lock = threading.Lock()

# Initialize empty HA-style state objects
for entity_id, cfg in ENTITY_MAP.items():
    attrs = cfg.get("attributes", {}).copy()  # Copy to include friendly_name
    # Add source name as 'name' attribute for compatibility with MQTT discovery naming
    source_group, source_key = cfg["source"]
    attrs["name"] = source_key
    states[entity_id] = {
        "state": None,
        "attributes": attrs
    }

def load_state_loop():
    last_mtime = 0

    while True:
        try:
            stat = os.stat(STATE_PATH)
            if stat.st_mtime != last_mtime:
                with open(STATE_PATH) as f:
                    payload = json.load(f)

                with state_lock:
                    for entity_id, cfg in ENTITY_MAP.items():
                        source_group, source_key = cfg["source"]

                        group = payload.get(source_group)
                        if not isinstance(group, dict):
                            continue

                        if source_key not in group:
                            continue

                        value = group[source_key]

                        if cfg["domain"] == "binary_sensor":
                            value = "on" if int(value) == 1 else "off"
                        elif cfg["domain"] == "select":
                            mapping = cfg.get("mapping", {})
                            value = mapping.get(int(value), str(value))

                        states[entity_id]["state"] = value

                last_mtime = stat.st_mtime

        except FileNotFoundError:
            pass  # bridge not ready yet
        except Exception as e:
            print(f"⚠️ Failed to load state file: {e}")

        time.sleep(STATE_POLL_INTERVAL)

threading.Thread(target=load_state_loop, daemon=True).start()
print(f"✅ Reading state from {STATE_PATH}")

# -----------------------------------------------------------------------------
# Flask app
# -----------------------------------------------------------------------------

app = Flask(__name__, static_folder=WWW_DIR, static_url_path="")

# Disable caching for development
#app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

#@app.after_request
#def add_no_cache_headers(response):
#    """Add no-cache headers to prevent browser caching during development"""
#    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
#    response.headers['Pragma'] = 'no-cache'
#    response.headers['Expires'] = '0'
#    return response

@app.route("/")
def index():
    return app.send_static_file("index.html")

@app.route("/api/states")
def api_states():
    with state_lock:
        return jsonify(states)

@app.route("/api/history/<sensor_name>")
def api_history(sensor_name):
    """Serve 24-hour history for a specific sensor"""
    try:
        # Map entity keys to friendly names used in sensor_history.json
        entity_to_friendly = {
            "storage_tank_vv": "Storage tank VV",
            "storage_tank_cv": "Storage tank CV",
            "cv_forward_temp": "CV Forward",
            "cv_return_temp": "CV Return",
            "outdoor_temp": "Outdoor",
            "evaporator_temp": "Evaporator",
            "compressor_hp_temp": "Compressor HP",
            "compressor_lp_temp": "Compressor LP",
            "defrost": "Defrost"
        }
        
        # Convert entity key to friendly name for lookup
        lookup_name = entity_to_friendly.get(sensor_name, sensor_name)
        
        history_path = "./../sensor_history.json"
        if not os.path.exists(history_path):
            return jsonify({"error": "No history data"}), 404
        
        with open(history_path, "r") as f:
            history = json.load(f)
        
        if lookup_name not in history:
            return jsonify({"error": f"Sensor {lookup_name} not found", "available": list(history.keys())}), 404
        
        return jsonify({
            "sensor": sensor_name,
            "data": history[lookup_name]
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/api/services/<domain>/<service>", methods=["POST"])
def api_service(domain, service):
    """Handle service calls from the web interface for all selects and numbers"""
    try:
        data = request.get_json() or {}
        entity_id = data.get("entity_id")
        
        print(f"🌐 API call received: {domain}.{service} for {entity_id}")
        print(f"   Data: {data}")
        
        if not entity_id:
            return jsonify({"error": "entity_id required"}), 400
        
        # Get entity config
        entity_cfg = ENTITY_MAP.get(entity_id)
        if not entity_cfg:
            return jsonify({"error": f"Unknown entity: {entity_id}"}), 404
        
        # Verify domain matches
        if entity_cfg.get("domain") != domain:
            return jsonify({"error": f"Domain mismatch for {entity_id}"}), 400
        
        # Build command based on domain and service
        command = None
        
        if domain == "select" and service == "select_option":
            option = data.get("option")
            if not option:
                return jsonify({"error": "option required"}), 400
            
            # Reverse lookup: option string -> numeric value
            mapping = entity_cfg.get("mapping", {})
            reverse_map = {v: k for k, v in mapping.items()}
            value = reverse_map.get(option)
            
            if value is None:
                return jsonify({"error": f"Invalid option: {option}"}), 400
            
            command = {
                "entity_id": entity_id,
                "domain": domain,
                "service": service,
                "value": value,
                "command_topic": entity_cfg.get("command_topic")
            }
        
        elif domain == "number" and service == "set_value":
            value = data.get("value")
            if value is None:
                return jsonify({"error": "value required"}), 400
            
            # Validate range
            attrs = entity_cfg.get("attributes", {})
            min_val = attrs.get("min", 0)
            max_val = attrs.get("max", 100)
            step = attrs.get("step", 1)
            
            try:
                value = float(value)
                if value < min_val or value > max_val:
                    return jsonify({"error": f"Value must be between {min_val} and {max_val}"}), 400
            except ValueError:
                return jsonify({"error": "Invalid numeric value"}), 400
            
            command = {
                "entity_id": entity_id,
                "domain": domain,
                "service": service,
                "value": int(value),  # Convert to int for modbus
                "command_topic": entity_cfg.get("command_topic")
            }
        
        else:
            return jsonify({"error": f"Unsupported service: {domain}.{service}"}), 400
        
        # Write command to queue file
        if command:
            write_command(command)
            print(f"✅ Command queued: {entity_id} = {command['value']}")
            return jsonify({"success": True})
        
        return jsonify({"error": "Failed to create command"}), 500
        
    except Exception as e:
        print(f"❌ Service call error: {e}")
        return jsonify({"error": str(e)}), 500

def write_command(command):
    """Append command to commands.json for bridge.py to process (atomic write)"""
    print(f"📝 Writing command to {COMMANDS_PATH}: {command}")
    with commands_lock:
        commands = []
        
        # Read existing commands if file exists
        if os.path.exists(COMMANDS_PATH):
            try:
                with open(COMMANDS_PATH, "r") as f:
                    commands = json.load(f)
                    if not isinstance(commands, list):
                        commands = []
            except:
                commands = []
        
        print(f"   Existing commands in queue: {len(commands)}")
        
        # Add timestamp
        command["timestamp"] = time.time()
        
        # Append new command
        commands.append(command)
        
        print(f"   Total commands after append: {len(commands)}")
        
        # Atomic write: write to temp file, then rename
        commands_dir = os.path.dirname(os.path.abspath(COMMANDS_PATH))
        with tempfile.NamedTemporaryFile(mode="w", dir=commands_dir, delete=False) as tmp:
            json.dump(commands, tmp, indent=2)
            tmp_path = tmp.name
        
        os.replace(tmp_path, COMMANDS_PATH)
        print(f"✅ Command written to {COMMANDS_PATH}")

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"✅ Web sidecar listening on http://{HTTP_HOST}:{HTTP_PORT}")
    app.run(host=HTTP_HOST, port=HTTP_PORT)

