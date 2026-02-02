import json
import logging
import os
import threading
import time
import tempfile
import subprocess
from pathlib import Path
from dotenv import load_dotenv

from flask import Flask, jsonify, request

from entity_map import ENTITY_MAP

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
logging.basicConfig(level=logging.DEBUG, format='[%(asctime)s] %(levelname)s %(message)s')
WWW_DIR = os.path.join(BASE_DIR, "www")

# Load environment from .env (actual RPi structure: /home/dviha/dvi-bridge/.env)
env_locations = [
    Path("/home/dviha/dvi-bridge/.env"),  # Standard installation location
]
for env_path in env_locations:
    if env_path.exists():
        load_dotenv(env_path)
        break

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

STATE_PATH = "./../state.json"
COMMANDS_PATH = "./../commands.json"
STATE_POLL_INTERVAL = 1.0  # seconds
TUNNEL_CONFIG_FILE = Path("/home/dviha/dvi-bridge/tunnel_config.json")

# -----------------------------------------------------------------------------
# Device identification - just use os.getenv like bridge.py
# -----------------------------------------------------------------------------

pump_id = f"pump-{os.getenv('FABNR')}" if os.getenv('FABNR') else "pump-unknown"

# -----------------------------------------------------------------------------
# Global state (HA-style)
# -----------------------------------------------------------------------------

states = {}
pump_type = "AW"  # Default to AW, will be updated from state.json
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
    global pump_type
    last_mtime = 0

    while True:
        try:
            stat = os.stat(STATE_PATH)
            if stat.st_mtime != last_mtime:
                with open(STATE_PATH) as f:
                    payload = json.load(f)

                # Extract pump type from payload
                if "pump_type" in payload:
                    pump_type = payload["pump_type"]

                with state_lock:
                    for entity_id, cfg in ENTITY_MAP.items():
                        source_group, source_key = cfg["source"]

                        # Special handling for pump_type sensor
                        if source_group == "pump_type":
                            states[entity_id]["state"] = pump_type
                            continue

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
print(f"✅ Device ID: {pump_id}")

# Load tunnel info if available
tunnel_info = {}
if TUNNEL_CONFIG_FILE.exists():
    try:
        with open(TUNNEL_CONFIG_FILE) as f:
            tunnel_info = json.load(f)
        print(f"✅ Device registered: {pump_id}")
        print(f"   Owner URL: https://{tunnel_info.get('owner_hostname', 'N/A')}")
        print(f"   Maker URL: https://{tunnel_info.get('maker_hostname', 'N/A')}")
    except Exception as e:
        print(f"⚠️  Failed to load tunnel config: {e}")
else:
    print(f"⚠️  Device not registered yet (pump_id: {pump_id})")
    print("   Run: sudo python3 registration.py")

# -----------------------------------------------------------------------------
# Flask app
# -----------------------------------------------------------------------------


app = Flask(__name__, static_folder=WWW_DIR, static_url_path="")

# -----------------------------------------------------------------------------
# Authentication Middleware
# -----------------------------------------------------------------------------
# This middleware protects the web interface when accessed via Cloudflare Zero Trust tunnel.
#
# - If the request hostname starts with "Owner-", authentication is required.
#   The Owner's app must send an Authorization header: 'Bearer <token>'
#   The token must be pre-shared and its SHA256 hash must be present in load_valid_token_hashes().
# - If accessed via local network (hostname does not start with "Owner-"), no authentication is required.
# - Endpoints listed in PUBLIC_PATHS are always accessible without authentication.
#
# To add a valid token:
#   1. Generate a secure random token (e.g., with `openssl rand -hex 32`).
#   2. Compute its SHA256 hash (e.g., `echo -n <token> | sha256sum`).
#   3. Add the hash to the set returned by load_valid_token_hashes().
#   4. Distribute the token securely to the Owner's app.
#
# To add a new public endpoint, add its path to the PUBLIC_PATHS set.

import hashlib
from flask import abort, g

import secrets

TOKEN_HASHES_FILE = os.path.join(BASE_DIR, "valid_token_hashes.json")

def load_valid_token_hashes():
    """
    Load valid token hashes from a file for persistence.
    """
    if not os.path.exists(TOKEN_HASHES_FILE):
        logging.debug(f"Token hashes file not found: {TOKEN_HASHES_FILE}")
        return set()
    try:
        with open(TOKEN_HASHES_FILE, "r") as f:
            hashes = json.load(f)
        logging.debug(f"Loaded token hashes: {hashes}")
        return set(hashes)
    except Exception as e:
        logging.error(f"Error loading token hashes: {e}")
        return set()

def save_valid_token_hash(token_hash):
    """
    Add a new token hash to the persistent file.
    """
    hashes = load_valid_token_hashes()
    hashes.add(token_hash)
    try:
        with open(TOKEN_HASHES_FILE, "w") as f:
            json.dump(list(hashes), f, indent=2)
        logging.debug(f"Saved new token hash: {token_hash}")
    except Exception as e:
        logging.error(f"Error saving token hash: {e}")

PUBLIC_PATHS = {
    "/pair",
    "/health",
}

def verify_token(token: str) -> bool:
    """
    Check token against stored hashes.
    Replace this with your real storage.
    """
    token_hash = hashlib.sha256(token.encode()).hexdigest()
    valid = token_hash in load_valid_token_hashes()
    logging.debug(f"Verifying token: {token[:6]}... hash: {token_hash}, valid: {valid}")
    return valid

@app.before_request
def auth_middleware():
    # Allow public endpoints
    if request.path in PUBLIC_PATHS:
        return

    # Only require auth for Owner app via Cloudflare tunnel (hostname check)
    host = request.host.split(":")[0]  # Remove port if present
    if host.startswith("Owner-"):
        auth = request.headers.get("Authorization", "")
        if not auth.startswith("Bearer "):
            abort(401)
        token = auth.removeprefix("Bearer ").strip()
        if not token or not verify_token(token):
            abort(401)
        # Optional: attach identity info
        g.owner_authenticated = True
    # If not Owner app, allow (local network, etc.)

# Disable caching for development
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

@app.after_request
def add_no_cache_headers(response):
    """Add no-cache headers to prevent browser caching during development"""
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

@app.route("/")
def index():
    return app.send_static_file("index.html")


# -----------------------------------------------------------------------------
# Pairing Endpoint (local network only)
# -----------------------------------------------------------------------------
@app.route("/pair", methods=["POST"])
def pair():
    """
    Allow pairing only from local network (not via Cloudflare tunnel/Owner-* hostnames).
    Generates a new token, stores its hash, and returns the token to the app.
    """
    host = request.host.split(":")[0]
    # Block if accessed via tunnel (Owner-* hostnames)
    if host.startswith("Owner-"):
        return jsonify({"error": "Pairing not allowed from tunnel"}), 403

    # Optionally, restrict by IP (e.g., only allow RFC1918 private IPs)
    # client_ip = request.remote_addr
    # if not client_ip.startswith(("192.168.", "10.", "172.")):
    #     return jsonify({"error": "Pairing only allowed from local network"}), 403

    # Generate secure random token
    token = secrets.token_hex(32)
    token_hash = hashlib.sha256(token.encode()).hexdigest()
    logging.debug(f"Generated new token: {token[:6]}... hash: {token_hash}")
    save_valid_token_hash(token_hash)
    return jsonify({"token": token})

@app.route("/api/states")
def api_states():
    with state_lock:
        states_copy = states.copy()
        
        # TESTING OVERRIDE: Force geothermal pump on for BW testing
        # Uncomment the following lines to test BW mode with brine circulation active
        # if "binary_sensor.circ_pump_geothermal" in states_copy:
        #     states_copy["binary_sensor.circ_pump_geothermal"]["state"] = "on"
        
        # TESTING OVERRIDE: Add fake BW temperature sensors for testing
        # Uncomment the following lines to test BW cold side temperatures
        # states_copy["sensor.cold_side_warm_temp"] = {
        #     "state": "8.5",
        #     "attributes": {"unit_of_measurement": "°C", "friendly_name": "Cold side warm"}
        # }
        # states_copy["sensor.cold_side_cold_temp"] = {
        #     "state": "2.3",
        #     "attributes": {"unit_of_measurement": "°C", "friendly_name": "Cold side cold"}
        # }
        
        return jsonify(states_copy)

@app.route("/api/pump_type")
def api_pump_type():
    """Return the detected pump type (AW or BW)"""
    # TESTING OVERRIDE: Uncomment the next line to force BW mode in frontend
    # return jsonify({"pump_type": "BW"})
    return jsonify({"pump_type": pump_type})


@app.route("/api/device_info")
def api_device_info():
    """Return device identification and tunnel information"""
    info = {
        "pump_id": pump_id,
        "pump_type": pump_type,
        "registered": TUNNEL_CONFIG_FILE.exists()
    }
    
    # Add tunnel info if available
    if TUNNEL_CONFIG_FILE.exists():
        try:
            with open(TUNNEL_CONFIG_FILE) as f:
                tunnel_data = json.load(f)
            info["tunnel"] = {
                "owner_hostname": tunnel_data.get("owner_hostname"),
                "maker_hostname": tunnel_data.get("maker_hostname"),
                "owner_url": f"https://{tunnel_data.get('owner_hostname')}" if tunnel_data.get('owner_hostname') else None,
                "maker_url": f"https://{tunnel_data.get('maker_hostname')}" if tunnel_data.get('maker_hostname') else None
            }
        except Exception as e:
            logging.error(f"Failed to load tunnel config: {e}")
    
    return jsonify(info)


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
            "cold_side_warm_temp": "Cold side warm",
            "cold_side_cold_temp": "Cold side cold",
            "defrost": "Defrost",
            "aux_heating": "Aux heating"
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

