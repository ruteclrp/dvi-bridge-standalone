import json
import os
import threading

from flask import Flask, jsonify

from entity_map import ENTITY_MAP

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WWW_DIR = os.path.join(BASE_DIR, "www")

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

# -----------------------------------------------------------------------------
# Global state (HA-style)
# -----------------------------------------------------------------------------

states = {}
state_lock = threading.Lock()

# Initialize empty HA-style state objects
for entity_id, cfg in ENTITY_MAP.items():
    states[entity_id] = {
        "state": None,
        "attributes": cfg.get("attributes", {})
    }

STATE_PATH = "./../state.json"
STATE_POLL_INTERVAL = 1.0  # seconds

import time

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

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"✅ Web sidecar listening on http://{HTTP_HOST}:{HTTP_PORT}")
    app.run(host=HTTP_HOST, port=HTTP_PORT)

