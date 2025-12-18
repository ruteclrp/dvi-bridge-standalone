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
    states[entity_id] = {
        "state": None,
        "attributes": cfg.get("attributes", {})
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

@app.route("/")
def index():
    return app.send_static_file("index.html")

@app.route("/api/states")
def api_states():
    with state_lock:
        return jsonify(states)

@app.route("/api/services/<domain>/<service>", methods=["POST"])
def api_service(domain, service):
    """Handle service calls from the web interface - TESTING: aux_heating only"""
    try:
        data = request.get_json() or {}
        entity_id = data.get("entity_id")
        
        if not entity_id:
            return jsonify({"error": "entity_id required"}), 400
        
        # TESTING: Only allow aux_heating for now
        if entity_id != "select.aux_heating":
            return jsonify({"error": f"Only aux_heating supported during testing"}), 400
        
        # Get entity config
        entity_cfg = ENTITY_MAP.get(entity_id)
        if not entity_cfg:
            return jsonify({"error": f"Unknown entity: {entity_id}"}), 404
        
        # Build command - only select.select_option for aux_heating
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
        
        # Add timestamp
        command["timestamp"] = time.time()
        
        # Append new command
        commands.append(command)
        
        # Atomic write: write to temp file, then rename
        commands_dir = os.path.dirname(os.path.abspath(COMMANDS_PATH))
        with tempfile.NamedTemporaryFile(mode="w", dir=commands_dir, delete=False) as tmp:
            json.dump(commands, tmp, indent=2)
            tmp_path = tmp.name
        
        os.replace(tmp_path, COMMANDS_PATH)

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"✅ Web sidecar listening on http://{HTTP_HOST}:{HTTP_PORT}")
    app.run(host=HTTP_HOST, port=HTTP_PORT)

