import json
import logging
import os
import threading
import time
import tempfile
import subprocess
from datetime import date, datetime, timedelta
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
USAGE_DAILY_PATH = "./../meter_usage_daily.json"
STATE_POLL_INTERVAL = 1.0  # seconds
TUNNEL_CONFIG_FILE = Path("/home/dviha/dvi-bridge/tunnel_config.json")
DEVICE_REGISTRATION_FILE = Path("/home/dviha/dvi-bridge/device_registration.json")
OPEN_REQUEST_FILE = Path("/home/dviha/dvi-bridge/open_request.json")
OPEN_CONFIRM_FILE = Path("/home/dviha/dvi-bridge/open_confirm.json")
OPEN_CLOSE_FILE = Path("/home/dviha/dvi-bridge/open_close.json")
OPEN_REQUEST_TTL = int(os.getenv("OPEN_REQUEST_TTL", "900"))
OPEN_CONFIRM_TTL = 15 * 60

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
sumalarm_latched = False

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
    global pump_type, sumalarm_latched
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
                        if entity_id == "binary_sensor.sum_alarm_failure" and value == "on":
                            sumalarm_latched = True

                last_mtime = stat.st_mtime

        except FileNotFoundError:
            pass  # bridge not ready yet
        except Exception as e:
            print(f"⚠️ Failed to load state file: {e}")

        time.sleep(STATE_POLL_INTERVAL)

threading.Thread(target=load_state_loop, daemon=True).start()
print(f"✅ Reading state from {STATE_PATH}")
print(f"✅ Device ID: {pump_id}")


def _read_open_request() -> dict:
    if not OPEN_REQUEST_FILE.exists():
        return {"pending": False}
    try:
        payload = json.loads(OPEN_REQUEST_FILE.read_text())
        payload["pending"] = bool(payload.get("pending"))
        requested_at = payload.get("requested_at")
        if payload["pending"] and requested_at:
            expires_at = int(requested_at) + OPEN_REQUEST_TTL
            if int(time.time()) >= expires_at:
                OPEN_REQUEST_FILE.unlink(missing_ok=True)
                return {"pending": False, "expired": True}
        return payload
    except Exception:
        return {"pending": False}


def _clear_open_request() -> None:
    try:
        if OPEN_REQUEST_FILE.exists():
            OPEN_REQUEST_FILE.unlink()
    except Exception:
        return


def _read_open_confirm() -> dict:
    if not OPEN_CONFIRM_FILE.exists():
        return {"confirmed": False}
    try:
        payload = json.loads(OPEN_CONFIRM_FILE.read_text())
        confirmed_at = payload.get("confirmed_at")
        if not confirmed_at:
            return {"confirmed": False}
        expires_at = int(confirmed_at) + OPEN_CONFIRM_TTL
        if int(time.time()) >= expires_at:
            OPEN_CONFIRM_FILE.unlink(missing_ok=True)
            return {"confirmed": False, "expired": True}
        return {
            "confirmed": True,
            "confirmed_at": int(confirmed_at),
            "expires_at": expires_at,
        }
    except Exception:
        return {"confirmed": False}

# Load registration info if available
registration_info = {}
tunnel_info = {}
registration_mode = "unregistered"

if DEVICE_REGISTRATION_FILE.exists():
    try:
        with open(DEVICE_REGISTRATION_FILE) as f:
            registration_info = json.load(f)
        registration_mode = registration_info.get("mode", "backend_access")
        print(f"✅ Device registered: {pump_id}")
        if registration_info.get("backend_url"):
            print(f"   Backend URL: {registration_info.get('backend_url')}")
    except Exception as e:
        print(f"⚠️  Failed to load registration config: {e}")

if TUNNEL_CONFIG_FILE.exists():
    try:
        with open(TUNNEL_CONFIG_FILE) as f:
            tunnel_info = json.load(f)
        if registration_mode == "unregistered":
            registration_mode = "legacy_tunnel"
        print(f"✅ Owner tunnel configured: {pump_id}")
        print(f"   Owner URL: https://{tunnel_info.get('owner_hostname', 'N/A')}")
    except Exception as e:
        print(f"⚠️  Failed to load tunnel config: {e}")

if not registration_info and not tunnel_info:
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
AUTH_COOKIE_NAME = "dvi_owner_session"
AUTH_COOKIE_MAX_AGE = 60 * 60 * 24 * 30  # 30 days

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

def hash_token(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()

def verify_token_hash(token_hash: str) -> bool:
    """Check token hash against stored hashes."""
    valid = token_hash in load_valid_token_hashes()
    logging.debug(f"Verifying token hash: {token_hash[:8]}..., valid: {valid}")
    return valid

def verify_token(token: str) -> bool:
    """Check raw token against stored hashes."""
    token_hash = hash_token(token)
    logging.debug(f"Verifying token: {token[:6]}... hash: {token_hash}")
    return verify_token_hash(token_hash)

@app.before_request
def auth_middleware():
    # Allow public endpoints
    if request.path in PUBLIC_PATHS:
        return

    # Only require auth for Owner app via Cloudflare tunnel (hostname check)
    host = request.host.split(":")[0]  # Remove port if present
    if "-owner." in host.lower():
        logging.debug(f"Tunnel access detected for host: {host}, path: {request.path}")

        auth = request.headers.get("Authorization", "")
        token = ""
        if auth.startswith("Bearer "):
            token = auth.removeprefix("Bearer ").strip()

        cookie_hash = request.cookies.get(AUTH_COOKIE_NAME, "")

        if token:
            token_hash = hash_token(token)
            if not verify_token_hash(token_hash):
                logging.warning(f"Invalid token from {host}")
                abort(401)
            g.owner_authenticated = True
            g.set_auth_cookie = True
            g.auth_cookie_hash = token_hash
            logging.debug(f"Successfully authenticated via header for {request.path}")
            return

        if cookie_hash and verify_token_hash(cookie_hash):
            g.owner_authenticated = True
            logging.debug(f"Successfully authenticated via cookie for {request.path}")
            return

        if not auth:
            logging.warning(f"Missing Authorization header from {host}")
        else:
            logging.warning(f"Invalid Authorization header from {host}")
        abort(401)
    # If not Owner app, allow (local network, etc.)

# Disable caching for development
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

@app.after_request
def add_no_cache_headers(response):
    """Add no-cache headers to prevent browser caching during development"""
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'

    if getattr(g, "set_auth_cookie", False):
        secure_cookie = request.is_secure
        response.set_cookie(
            AUTH_COOKIE_NAME,
            g.auth_cookie_hash,
            max_age=AUTH_COOKIE_MAX_AGE,
            httponly=True,
            secure=secure_cookie,
            samesite="Lax",
        )
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
    # Block if accessed via tunnel (owner hostnames contain -owner.)
    if "-owner." in host.lower():
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


# -----------------------------------------------------------------------------
# Tunnel Info Endpoint (local network only)
# -----------------------------------------------------------------------------
@app.route("/api/tunnel", methods=["GET"])
def api_tunnel():
    """
    Return tunnel URL for remote access.
    This endpoint allows the Owner's app to discover the tunnel URL
    when connected via local network.
    
    The app should first obtain a credential token via /pair endpoint,
    then use this tunnel URL with that token for authenticated remote access.
    """
    host = request.host.split(":")[0]
    
    # Block access via tunnel (same security as /pair endpoint)
    if "-owner." in host.lower():
        return jsonify({"error": "Tunnel info not available via tunnel"}), 403
    
    # Check if device is registered
    if not DEVICE_REGISTRATION_FILE.exists() and not TUNNEL_CONFIG_FILE.exists():
        return jsonify({
            "error": "Device not registered",
            "registered": False
        }), 404

    response = {
        "registered": True,
        "mode": registration_mode,
    }

    if DEVICE_REGISTRATION_FILE.exists():
        try:
            with open(DEVICE_REGISTRATION_FILE) as f:
                device_data = json.load(f)
            response["backend_url"] = device_data.get("backend_url")
            response["device_id"] = device_data.get("device_id")
        except Exception as e:
            logging.error(f"Failed to load registration config: {e}")
            return jsonify({"error": "Failed to load registration configuration"}), 500

    if TUNNEL_CONFIG_FILE.exists():
        try:
            with open(TUNNEL_CONFIG_FILE) as f:
                tunnel_data = json.load(f)
        except Exception as e:
            logging.error(f"Failed to load tunnel config: {e}")
            return jsonify({"error": "Failed to load tunnel configuration"}), 500

        owner_hostname = tunnel_data.get("owner_hostname")
        if not owner_hostname:
            return jsonify({"error": "Owner hostname not configured"}), 500
        response["tunnel_url"] = f"https://{owner_hostname}"

    return jsonify(response)


@app.route("/api/states")
def api_states():
    global sumalarm_latched
    with state_lock:
        states_copy = states.copy()
        
        open_request = _read_open_request()
        open_confirm = _read_open_confirm()
        open_status = "off"
        if open_request.get("pending"):
            open_status = "pending"
        elif open_confirm.get("confirmed"):
            open_status = "confirmed"
        states_copy["binary_sensor.open_request"] = {
            "state": "on" if open_status in {"pending", "confirmed"} else "off",
            "attributes": {
                "friendly_name": "Open request",
                "status": open_status,
                "confirmed_until": open_confirm.get("expires_at"),
            },
        }

        if sumalarm_latched:
            current_alarm = states_copy.get("binary_sensor.sum_alarm_failure", {})
            attributes = dict(current_alarm.get("attributes", {}))
            attributes["latched"] = True
            states_copy["binary_sensor.sum_alarm_failure"] = {
                "state": "on",
                "attributes": attributes,
            }

        return jsonify(states_copy)


@app.route("/api/alarm/sumalarm/ack", methods=["POST"])
def api_ack_sumalarm():
    global sumalarm_latched
    with state_lock:
        sumalarm_latched = False
    return jsonify({"status": "ok"})

@app.route("/api/pump_type")
def api_pump_type():
    """Return the detected pump type (AW or BW)"""
    return jsonify({"pump_type": pump_type})


@app.route("/api/device_info")
def api_device_info():
    """Return device identification and tunnel information"""
    info = {
        "pump_id": pump_id,
        "pump_type": pump_type,
        "registered": DEVICE_REGISTRATION_FILE.exists() or TUNNEL_CONFIG_FILE.exists(),
        "mode": registration_mode,
    }

    # Add registration info if available
    if DEVICE_REGISTRATION_FILE.exists():
        try:
            with open(DEVICE_REGISTRATION_FILE) as f:
                device_data = json.load(f)
            info["backend"] = {
                "backend_url": device_data.get("backend_url"),
                "device_id": device_data.get("device_id"),
            }
        except Exception as e:
            logging.error(f"Failed to load registration config: {e}")

    if TUNNEL_CONFIG_FILE.exists():
        try:
            with open(TUNNEL_CONFIG_FILE) as f:
                tunnel_data = json.load(f)
            info["tunnel"] = {
                "owner_hostname": tunnel_data.get("owner_hostname"),
                "owner_url": f"https://{tunnel_data.get('owner_hostname')}" if tunnel_data.get('owner_hostname') else None,
            }
        except Exception as e:
            logging.error(f"Failed to load tunnel config: {e}")
    
    return jsonify(info)


@app.route("/api/history/<sensor_name>")
def api_history(sensor_name):
    """Serve history for a specific sensor filtered to a selected local date."""
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

        requested_date_raw = (request.args.get("date") or "").strip()
        today = datetime.now().date()
        if requested_date_raw:
            requested_date = _parse_iso_date(requested_date_raw)
            if requested_date is None:
                return jsonify({"error": "Invalid date format. Expected YYYY-MM-DD"}), 400
        else:
            requested_date = today

        earliest_date = today - timedelta(days=29)
        if requested_date < earliest_date or requested_date > today:
            return jsonify({
                "error": "Date must be within the last 30 days",
                "min_date": earliest_date.isoformat(),
                "max_date": today.isoformat(),
            }), 400
        
        history_path = "./../sensor_history.json"
        if not os.path.exists(history_path):
            return jsonify({"error": "No history data"}), 404
        
        with open(history_path, "r") as f:
            history = json.load(f)
        
        if lookup_name not in history:
            return jsonify({"error": f"Sensor {lookup_name} not found", "available": list(history.keys())}), 404

        selected_data = [
            point for point in history[lookup_name]
            if datetime.fromtimestamp(point.get("timestamp", 0)).date() == requested_date
        ]

        start_dt = datetime.combine(requested_date, datetime.min.time())
        if requested_date == today:
            end_ts = time.time()
        else:
            end_ts = (start_dt + timedelta(days=1)).timestamp()
        
        return jsonify({
            "sensor": sensor_name,
            "date": requested_date.isoformat(),
            "period": {
                "start_timestamp": start_dt.timestamp(),
                "end_timestamp": end_ts,
            },
            "data": selected_data,
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value):
    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return None


def _to_energy(value):
    try:
        return round(float(value), 1)
    except (TypeError, ValueError):
        return None


def _load_daily_usage_snapshots():
    if not os.path.exists(USAGE_DAILY_PATH):
        return {}
    try:
        with open(USAGE_DAILY_PATH, "r") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _sorted_snapshot_dates(snapshots):
    valid_dates = []
    for key in snapshots.keys():
        try:
            valid_dates.append(datetime.strptime(key, "%Y-%m-%d").date())
        except ValueError:
            continue
    return sorted(valid_dates)


def _snapshot_for_date(snapshots, snapshot_date):
    rec = snapshots.get(snapshot_date.strftime("%Y-%m-%d"))
    if not isinstance(rec, dict):
        return None

    # New format stores nested first/latest snapshots
    if "latest" in rec and isinstance(rec.get("latest"), dict):
        rec = rec.get("latest")

    return {
        "comp_hours": _to_int(rec.get("comp_hours")),
        "vv_hours": _to_int(rec.get("vv_hours")),
        "heating_hours": _to_int(rec.get("heating_hours")),
        "em23_energy": _to_energy(rec.get("em23_energy")),
    }


def _first_snapshot_for_date(snapshots, snapshot_date):
    rec = snapshots.get(snapshot_date.strftime("%Y-%m-%d"))
    if not isinstance(rec, dict):
        return None

    # New format: explicit first snapshot
    if "first" in rec and isinstance(rec.get("first"), dict):
        first = rec.get("first")
        return {
            "comp_hours": _to_int(first.get("comp_hours")),
            "vv_hours": _to_int(first.get("vv_hours")),
            "heating_hours": _to_int(first.get("heating_hours")),
            "em23_energy": _to_energy(first.get("em23_energy")),
        }

    # Legacy format fallback
    return {
        "comp_hours": _to_int(rec.get("comp_hours")),
        "vv_hours": _to_int(rec.get("vv_hours")),
        "heating_hours": _to_int(rec.get("heating_hours")),
        "em23_energy": _to_energy(rec.get("em23_energy")),
    }


def _latest_snapshot_on_or_before(snapshots, sorted_dates, target_date):
    for d in reversed(sorted_dates):
        if d <= target_date:
            rec = _snapshot_for_date(snapshots, d)
            if rec is not None:
                return d, rec
    return None, None


def _latest_snapshot_before(snapshots, sorted_dates, target_date):
    for d in reversed(sorted_dates):
        if d < target_date:
            rec = _snapshot_for_date(snapshots, d)
            if rec is not None:
                return d, rec
    return None, None


def _earliest_snapshot_in_range(snapshots, sorted_dates, start_date, end_date):
    for d in sorted_dates:
        if d < start_date:
            continue
        if d > end_date:
            break
        rec = _first_snapshot_for_date(snapshots, d)
        if rec is not None:
            return d, rec
    return None, None


def _range_bounds(range_key, today):
    if range_key == "today":
        return today, today
    if range_key == "yesterday":
        y = today - timedelta(days=1)
        return y, y
    if range_key == "this_week":
        start = today - timedelta(days=today.weekday())
        return start, today
    if range_key == "last_week":
        this_week_start = today - timedelta(days=today.weekday())
        start = this_week_start - timedelta(days=7)
        end = this_week_start - timedelta(days=1)
        return start, end
    if range_key == "this_month":
        start = today.replace(day=1)
        return start, today
    if range_key == "last_month":
        this_month_start = today.replace(day=1)
        end = this_month_start - timedelta(days=1)
        start = end.replace(day=1)
        return start, end
    if range_key == "this_year":
        start = date(today.year, 1, 1)
        return start, today
    if range_key == "last_year":
        start = date(today.year - 1, 1, 1)
        end = date(today.year - 1, 12, 31)
        return start, end
    return None, None


def _parse_iso_date(value):
    if not value:
        return None
    try:
        return date.fromisoformat(value)
    except ValueError:
        return None


def _resolve_usage_period(args):
    start_value = (args.get("start") or "").strip()
    end_value = (args.get("end") or "").strip()
    if start_value or end_value:
        if not start_value or not end_value:
            return None, None, "Both 'start' and 'end' must be provided together"

        start_date = _parse_iso_date(start_value)
        end_date = _parse_iso_date(end_value)
        if start_date is None or end_date is None:
            return None, None, "Invalid date format. Expected YYYY-MM-DD"
        if start_date > end_date:
            return None, None, "'start' must be on or before 'end'"
        return start_date, end_date, None

    preset = (args.get("preset") or args.get("range") or "today").strip().lower().replace(" ", "_")
    allowed_presets = {
        "today",
        "yesterday",
        "this_week",
        "last_week",
        "this_month",
        "last_month",
        "this_year",
        "last_year",
    }
    if preset not in allowed_presets:
        return None, None, {
            "error": f"Unsupported preset '{preset}'",
            "allowed": sorted(allowed_presets),
        }

    today = datetime.now().date()
    start_date, end_date = _range_bounds(preset, today)
    return start_date, end_date, None


def _current_meter_totals_from_states():
    with state_lock:
        return {
            "comp_hours": _to_int(states.get("sensor.comp_hours", {}).get("state")),
            "vv_hours": _to_int(states.get("sensor.vv_hours", {}).get("state")),
            "heating_hours": _to_int(states.get("sensor.heating_hours", {}).get("state")),
            "em23_energy": _to_energy(states.get("sensor.em23_energy", {}).get("state")),
        }


def _compute_usage_delta(start_totals, end_totals):
    keys = ("comp_hours", "vv_hours", "heating_hours", "em23_energy")
    delta = {}
    reset_detected = False
    for key in keys:
        start_val = start_totals.get(key) if start_totals else None
        end_val = end_totals.get(key) if end_totals else None
        if start_val is None or end_val is None:
            delta[key] = None
            continue
        diff = end_val - start_val
        if diff < 0:
            reset_detected = True
            diff = max(end_val, 0.0)
        if key == "em23_energy":
            delta[key] = round(diff, 1)
        else:
            delta[key] = int(round(diff))
    delta["reset_detected"] = reset_detected
    return delta


@app.route("/api/usage_summary")
def api_usage_summary():
    start_date, end_date, error = _resolve_usage_period(request.args)
    if isinstance(error, dict):
        return jsonify(error), 400
    if error:
        return jsonify({"error": error}), 400

    snapshots = _load_daily_usage_snapshots()
    sorted_dates = _sorted_snapshot_dates(snapshots)
    today = datetime.now().date()

    if end_date == today:
        end_source_date, end_totals = today, _current_meter_totals_from_states()
    else:
        end_source_date, end_totals = _latest_snapshot_on_or_before(snapshots, sorted_dates, end_date)

    start_source_date, start_totals = _latest_snapshot_before(snapshots, sorted_dates, start_date)

    # Fallback: if no snapshot before period start, use first snapshot of start day
    # so in-progress ranges (today/this week/month/year) remain meaningful.
    if start_totals is None:
        first_start_totals = _first_snapshot_for_date(snapshots, start_date)
        if first_start_totals is not None:
            start_source_date, start_totals = start_date, first_start_totals

    # Fallback for partial history: use earliest available snapshot inside period.
    # This avoids returning '-' for ranges like this_week/this_month/this_year when
    # only recent days exist in the snapshot file.
    if start_totals is None:
        first_in_range_date, first_in_range_totals = _earliest_snapshot_in_range(
            snapshots,
            sorted_dates,
            start_date,
            end_date,
        )
        if first_in_range_totals is not None:
            start_source_date, start_totals = first_in_range_date, first_in_range_totals

    delta = _compute_usage_delta(start_totals, end_totals)

    return jsonify({
        "range": request.args.get("range"),
        "preset": (request.args.get("preset") or request.args.get("range") or None),
        "period": {
            "start_date": start_date.isoformat(),
            "end_date": end_date.isoformat(),
            "start_source_date": start_source_date.isoformat() if start_source_date else None,
            "end_source_date": end_source_date.isoformat() if end_source_date else None,
        },
        "summary": delta,
    })


@app.route("/api/open_request", methods=["GET"])
def api_open_request():
    return jsonify(_read_open_request())


@app.route("/api/open_request/confirm", methods=["POST"])
def api_open_request_confirm():
    payload = {
        "pump_id": pump_id,
        "confirmed_at": int(time.time()),
    }
    try:
        OPEN_CONFIRM_FILE.parent.mkdir(parents=True, exist_ok=True)
        OPEN_CONFIRM_FILE.write_text(json.dumps(payload))
        _clear_open_request()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/open_request/close", methods=["POST"])
def api_open_request_close():
    try:
        payload = {
            "pump_id": pump_id,
            "closed_at": int(time.time()),
        }
        OPEN_CLOSE_FILE.parent.mkdir(parents=True, exist_ok=True)
        OPEN_CLOSE_FILE.write_text(json.dumps(payload))
        OPEN_CONFIRM_FILE.unlink(missing_ok=True)
        _clear_open_request()
        return jsonify({"status": "ok"})
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

