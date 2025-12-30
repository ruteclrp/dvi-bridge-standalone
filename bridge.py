from dotenv import load_dotenv
import sys
import os
import minimalmodbus
import paho.mqtt.client as mqtt
import struct
import json
import time
import threading
import warnings
import glob
import subprocess
import socket  # <-- nødvendig til _get_default_gateway_linux
import tempfile
from typing import Optional

# Find STM32 Virtual COM Port automatically
devices = glob.glob("/dev/serial/by-id/*STM32*")

if not devices:
    print("❌ STM32 Virtual COM Port not found! Check USB cable and that the heatpump interface is connected.")
    raise RuntimeError("STM32 Virtual COM Port not found!")
else:
    serial_port = devices[0]
    print(f"✅ Connected to STM32 Virtual COM Port: {serial_port}")

load_dotenv()  # this will read .env in the current directory
warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_VALUES_SCRIPT = os.path.join(SCRIPT_DIR, "read_static_values_modbustk.py")

# History tracking
HISTORY_FILE = os.path.join(SCRIPT_DIR, "sensor_history.json")
HISTORY_MAX_AGE = 86400  # 24 hours in seconds
HISTORY_SAMPLE_INTERVAL = 15  # Sample every 15 seconds

def _refresh_static_values() -> None:
    if not os.path.isfile(STATIC_VALUES_SCRIPT):
        print("⚠️ read_static_values_modbustk.py not found; skipping static refresh.")
        return
    print("ℹ️ Refreshing static values via read_static_values_modbustk.py ...")
    try:
        result = subprocess.run(
            [sys.executable, STATIC_VALUES_SCRIPT],
            cwd=os.path.dirname(STATIC_VALUES_SCRIPT),
            timeout=60,
            capture_output=True,
            text=True,
        )
        print(result.stdout, end="")
        if result.returncode != 0:
            print(f"⚠️ read_static_values_modbustk.py exited with code {result.returncode}")
            if result.stderr:
                print(result.stderr, end="")
            return
        load_dotenv(override=True)
    except Exception as e:
        print(f"⚠️ Failed to run read_static_values_modbustk.py: {e}")

STATE_PATH = "./state.json"

def write_state_atomic(state: dict) -> None:
    """
    Atomically write JSON state to STATE_PATH.
    Readers will always see either the old or the new file, never a partial write.
    """
    dir_name = os.path.dirname(STATE_PATH)

    # Ensure directory exists
    os.makedirs(dir_name, exist_ok=True)

    with tempfile.NamedTemporaryFile(
        mode="w",
        dir=dir_name,
        delete=False
    ) as tmp:
        json.dump(state, tmp)
        tmp.flush()
        os.fsync(tmp.fileno())
        temp_name = tmp.name

    # Atomic replace on POSIX
    os.replace(temp_name, STATE_PATH)

def _ensure_pump_id() -> Optional[str]:
    _refresh_static_values()
    return os.getenv("FABNR") or None

PUMP_ID: Optional[str] = _ensure_pump_id()
SWBOT: Optional[str] = os.getenv("SWBOT") or None
SWTOP: Optional[str] = os.getenv("SWTOP") or None
INSTALL_DD: Optional[str] = os.getenv("INSTALL_DD") or None
INSTALL_MM: Optional[str] = os.getenv("INSTALL_MM") or None
INSTALL_YY: Optional[str] = os.getenv("INSTALL_YY") or None
SERVICE_DD: Optional[str] = os.getenv("SERVICE_DD") or None
SERVICE_MM: Optional[str] = os.getenv("SERVICE_MM") or None
SERVICE_YY: Optional[str] = os.getenv("SERVICE_YY") or None

if PUMP_ID:
    print(f"🆔 Fabrication ID set to: {PUMP_ID}")
else:
    print("⚠️ No fabrication ID (FABNR) found – will not be able to upload data to DVI backend.")

if SWBOT or SWTOP:
    print(f"ℹ️ SW versions from .env: SWBOT={SWBOT or 'unknown'}, SWTOP={SWTOP or 'unknown'}")
else:
    print("ℹ️ No SWBOT/SWTOP set in .env")

if INSTALL_DD and INSTALL_MM and INSTALL_YY:
    print(f"ℹ️ Install date from .env: {INSTALL_DD}-{INSTALL_MM}-{INSTALL_YY}")
else:
    print("ℹ️ No install date (INDA) set in .env")

if SERVICE_DD and SERVICE_MM and SERVICE_YY:
    print(f"ℹ️ Service date from .env: {SERVICE_DD}-{SERVICE_MM}-{SERVICE_YY}")
else:
    print("ℹ️ No service date (SEDA) set in .env")

# Modbus setup
instrument = minimalmodbus.Instrument(serial_port, 0x10)
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 2
instrument.mode = minimalmodbus.MODE_RTU

modbus_lock = threading.Lock()

# Read credentials and broker info from environment variables
MQTT_USER: Optional[str] = os.getenv("MQTT_USER") or None
MQTT_PASS: Optional[str] = os.getenv("MQTT_PASS") or None
MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

# Detect pump type from register 0x15 (0=AW, 1=BW)
PUMP_TYPE = None  # Will be determined from register 0x15

def detect_pump_type() -> Optional[str]:
    """Read register 0x15 to determine pump type: 0=AW (Air-to-Water), 1=BW (Brine-to-Water/Geothermal)"""
    try:
        with modbus_lock:
            payload = struct.pack('>HH', 0x15, 0x0000)
            response = instrument._perform_command(6, payload)
            _, value = struct.unpack('>HH', response)
            if value == 0:
                print("✅ Detected pump type: AW (Air-to-Water) (register 0x15 = 0)")
                return "AW"
            elif value == 1:
                print("✅ Detected pump type: BW (Brine-to-Water/Geothermal) (register 0x15 = 1)")
                return "BW"
            else:
                print(f"⚠️ Unknown pump type value in register 0x15: {value}")
                return None
    except Exception as e:
        print(f"❌ Failed to read pump type from register 0x15: {e}")
        return None


# MQTT setup
mqtt_client = mqtt.Client()
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)

# Only set username/password if both are provided (support brokers with no auth)
if MQTT_USER and MQTT_PASS:
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)

def _build_device_info() -> dict:
    pump_model = PUMP_TYPE or "Unknown"
    device = {
        "name": f"DVI {pump_model}",
        "identifiers": [f"dvi_{pump_model.lower()}"],
        "manufacturer": "DVI",
        "model": f"{pump_model} Heatpump"
    }
    if PUMP_ID:
        device["identifiers"].append(f"pump_{PUMP_ID}")
        device["serial_number"] = PUMP_ID
    # Brug SWTOP som primær sw_version, ellers SWBOT
    sw = SWTOP or SWBOT
    if sw:
        device["sw_version"] = sw
    return device

def publish_discovery_sensor(name, unique_id, value_template,
                             unit=None, device_class=None, entity_category=None, state_class=None):
    config_topic = f"homeassistant/sensor/{unique_id}/config"
    payload = {
        "name": name,
        "state_topic": "dvi/measurement",
        "value_template": value_template,
        "unique_id": unique_id,
        "device": _build_device_info()
    }
    if unit: payload["unit_of_measurement"] = unit
    if device_class: payload["device_class"] = device_class
    if state_class: payload["state_class"] = state_class
    if entity_category: payload["entity_category"] = entity_category
    msg = json.dumps(payload)
    mqtt_client.publish(config_topic, msg, retain=True)

def publish_discovery_binary(name, unique_id, coil_key, device_class=None):
    config_topic = f"homeassistant/binary_sensor/{unique_id}/config"
    value_template = (
        f"{{{{ 'ON' if value_json.coils['{coil_key}'] == 1 else 'OFF' }}}}"
    )
    payload = {
        "name": name,
        "state_topic": "dvi/measurement",
        "value_template": value_template,
        "unique_id": unique_id,
        "device": _build_device_info(),
        "entity_category": "diagnostic"
    }
    if device_class:
        payload["device_class"] = device_class
    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)

def publish_discovery_number(name, unique_id, command_topic, state_template,
                             min_val=0, max_val=100, step=1, unit=None, entity_category=None):
    config_topic = f"homeassistant/number/{unique_id}/config"
    payload = {
        "name": name,
        "command_topic": command_topic,
        "state_topic": "dvi/measurement",
        "value_template": state_template,
        "unique_id": unique_id,
        "min": min_val,
        "max": max_val,
        "step": step,
        "mode": "box",
        "device": _build_device_info()
    }
    if unit: payload["unit_of_measurement"] = unit
    if entity_category: payload["entity_category"] = entity_category
    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)

def publish_discovery_select(name, unique_id, command_topic, state_template, options, entity_category=None):
    config_topic = f"homeassistant/select/{unique_id}/config"
    payload = {
        "name": name,
        "command_topic": command_topic,
        "state_topic": "dvi/measurement",
        "value_template": state_template,
        "unique_id": unique_id,
        "options": options,
        "device": _build_device_info()
    }
    if entity_category:
        payload["entity_category"] = entity_category
    msg = json.dumps(payload)
    mqtt_client.publish(config_topic, msg, retain=True)

# Coil mapping (coil 13 omitted)
# Base coil names for AW (Air-to-Water) type - all coils valid
coil_names_aw = {
    0: "Soft starter Compressor",
    1: "3-Way shunt VV open/close",
    2: "Start/stop expansion valve",
    3: "Heating element",
    4: "Circ. pump warm side",
    5: "El-tracing CV/drain",
    8: "4-way valve defrost",
    9: "Liquid injection solenoid valve",
    10: "3-way shunt CV open",
    11: "3-way shunt CV close",
    12: "Circ. pump CV",
    14: "Sum alarm failure"
}

# Coil names for BW (Brine-to-Water/Geothermal) type
coil_names_bw = {
    0: "Soft starter Compressor",
    # 1: excluded (AW-only: 3-Way shunt VV open/close)
    2: "Start/stop expansion valve",
    3: "Heating element",
    4: "Circ. pump warm side",
    # 5: not used for BW
    8: "Circ. pump geothermal",  # Different name for BW
    # 9: not used for BW
    10: "3-way shunt CV open",
    11: "3-way shunt CV close",
    12: "Circ. pump CV",
    # 14: not used for BW
}

def filter_coils_by_type(pump_type: Optional[str]) -> dict:
    """Return only the coils applicable to the detected pump type"""
    if pump_type is None:
        return coil_names_aw  # If type unknown, return AW (all coils)
    elif pump_type == "AW":
        return coil_names_aw
    elif pump_type == "BW":
        return coil_names_bw
    else:
        return coil_names_aw

# FC04 sensor mapping and filtering
omit_fc04 = {"sensor_4", "sensor_8", "sensor_9", "sensor_10"}

# FC04 sensors for AW (Air-to-Water) type
fc04_labels_aw = {
    "sensor_1": "CV Forward",
    "sensor_2": "CV Return",
    "sensor_3": "Storage tank VV",
    "sensor_5": "Storage tank CV",
    "sensor_6": "Evaporator",
    "sensor_7": "Outdoor",
    "sensor_11": "Compressor HP",
    "sensor_12": "Compressor LP"
}

# FC04 sensors for BW (Brine-to-Water/Geothermal) type
fc04_labels_bw = {
    "sensor_1": "CV Forward",
    "sensor_2": "CV Return",
    "sensor_3": "Storage tank VV",
    "sensor_5": "Storage tank CV",
    # sensor_6 excluded (AW-only: Evaporator)
    "sensor_7": "Outdoor",
    "sensor_11": "Compressor HP",
    "sensor_12": "Compressor LP",
    "sensor_13": "Cold side warm",   # BW-specific
    "sensor_14": "Cold side cold"    # BW-specific
}

def filter_fc04_by_type(pump_type: Optional[str]) -> dict:
    """Return only the FC04 sensors applicable to the detected pump type"""
    if pump_type is None:
        return fc04_labels_aw  # If type unknown, return AW
    elif pump_type == "AW":
        return fc04_labels_aw
    elif pump_type == "BW":
        return fc04_labels_bw
    else:
        return fc04_labels_aw

# Modbus-safe wrappers
def read_coils():
    try:
        with modbus_lock:
            payload = struct.pack('>HH', 0x0001, 0x000E)
            response = instrument._perform_command(1, payload)

        if len(response) < 3 or response[0] != 2:
            raise ValueError("FC01 response malformed")

        bitmask = (response[2] << 8) | response[1]
        bits = [(bitmask >> i) & 1 for i in range(16)]

        # Filter coils based on pump type
        active_coils = filter_coils_by_type(PUMP_TYPE)
        result = dict(sorted({active_coils[i]: bits[i] for i in active_coils}.items()))
        
        # TESTING OVERRIDE: Force geothermal pump on for BW testing
        # Uncomment the next line to test BW mode with brine circulation active
        # if "Circ. pump geothermal" in result:
        #     result["Circ. pump geothermal"] = 1
        
        return result
    except Exception as e:
        print(f"FC01 read failed: {e}")
        return {}

def read_input(register, signed=False):
    try:
        with modbus_lock:
            return instrument.read_register(register, number_of_decimals=0, functioncode=4, signed=signed)
    except Exception as e:
        print(f"FC04 read failed for 0x{register:02X}: {e}")
        return None

def read_via_fc06(register, signed=False):
    try:
        with modbus_lock:
            payload = struct.pack('>HH', register, 0x0000)
            response = instrument._perform_command(6, payload)
            _, value = struct.unpack('>HH', response)
            if signed:
                value = struct.unpack('>h', struct.pack('>H', value))[0]
            return value
    except Exception as e:
        print(f"FC06 echo failed for 0x{register:02X}: {e}")
        return None

def write_fc06(register, value):
    payload = struct.pack('>HH', register, value)
    try:
        with modbus_lock:
            instrument._perform_command(6, payload)  # Don't store or parse response
        print(f"✅ FC06 write sent: reg={register}, value={value}")
    except Exception as e:
        print(f"❌ FC06 write failed: {e}")

    # Store  raw values
def resolve_curve_register(which: str) -> Optional[dict]:
    """
    which: "-12" or "12"
    Returns {'read': int, 'write': int} for the current central heating config (0x1A).
    """
    raw_val = read_via_fc06(0x1A)
    if raw_val is None:
        print("⚠️ Could not read 0x1A to resolve curve register")
        return None

    last_writes["central_heating_config_raw"] = raw_val

    if raw_val == 0:
        # write: 0x12F / 0x130 → read: 0x02F / 0x030
        write_map = {"12": 0x12F, "-12": 0x130}
        read_map  = {"12": 0x2F,  "-12": 0x30}
    elif raw_val == 1:
        # write: 0x131 / 0x132 → read: 0x031 / 0x032
        write_map = {"12": 0x131, "-12": 0x132}
        read_map  = {"12": 0x31,  "-12": 0x32}
    elif raw_val == 2:
        # write: 0x133 / 0x134 → read: 0x033 / 0x034
        write_map = {"12": 0x133, "-12": 0x134}
        read_map  = {"12": 0x33,  "-12": 0x34}
    else:
        print(f"⚠️ Unknown 0x01A value {raw_val}, cannot resolve curve register")
        return None

    if which not in write_map:
        print(f"⚠️ Unknown curve selector '{which}'")
        return None

    return {"read": read_map[which], "write": write_map[which]}


# --- MQTT command handling for Modbus writes ---
command_map = {
    "dvi/command/cvstate": {"register": 0x101, "scale": 1},
    "dvi/command/cvcurve": {"register": 0x102, "scale": 1},
    "dvi/command/cvnight": {"register": 0x104, "scale": 1},
    "dvi/command/vvstate": {"register": 0x10A, "scale": 1},
    "dvi/command/vvsetpoint": {"register": 0x10B, "scale": 1},
    "dvi/command/vvschedule": {"register": 0x10C, "scale": 1},
    "dvi/command/tvstate": {"register": 0x10F, "scale": 1},
    "dvi/command/centralheatingconfig": {"register": 0x11A, "scale": 1},
    "dvi/command/cvmax": {"register": 0x11B, "scale": 1},
    "dvi/command/cvmin": {"register": 0x11C, "scale": 1},
#    "dvi/command/outdoorcal": {"register": 0x18D, "scale": 1}, 
    "dvi/command/curveset-12": {"dynamic_curve": "-12", "scale": 1},
    "dvi/command/curveset12": {"dynamic_curve": "12", "scale": 1},
}

# Map string payloads from HA selects to numeric register values
select_map = {
    "dvi/command/cvstate": {"Off": 0, "On": 1},
    "dvi/command/vvstate": {"Off": 0, "On": 1, "Timer": 2},
    "dvi/command/cvnight": {"Timer": 0, "Constant day": 1, "Constant night": 2},
    "dvi/command/vvschedule": {"Timer": 0, "Constant on": 1, "Constant off": 2},
    "dvi/command/tvstate": {"Off": 0, "Automatic": 1, "Backup operation": 2},
    "dvi/command/centralheatingconfig": {"Under floor heating w/o shunt": 0,
                                           "Under floor heating w. shunt": 1,
                                           "Radiator and mixed systems": 2}
}

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload_str = msg.payload.decode().strip()
        cfg = command_map.get(topic)
        if not cfg:
            return

        # Handle HA Select payloads (e.g. "Off", "On", "Automatic")
        if topic in select_map:
            if payload_str not in select_map[topic]:
                print(f"⚠️ Unknown select option '{payload_str}' for topic {topic}")
                return
            value_raw = select_map[topic][payload_str]
        else:
            value_raw = int(payload_str)

        # Dynamic curve register resolution
        if "dynamic_curve" in cfg:
            reg_info = resolve_curve_register(cfg["dynamic_curve"])
            if reg_info is None:
                print(f"❌ Could not resolve register for {topic}")
                return
            print(f"Writing dynamic curve register 0x{reg_info['write']:02X} with value {value_raw}")
            write_fc06(reg_info["write"], value_raw)
            print(f"✅ FC06 write: topic={topic} value={value_raw} reg=0x{reg_info['write']:02X}")
            return

        # Static writes
        scaled = value_raw * cfg.get("scale", 1)
        print(f"Writing to register {cfg['register']} with value {scaled}")
        write_fc06(cfg["register"], scaled)
        print(f"✅ FC06 write: topic={topic} value={value_raw} reg=0x{cfg['register']:02X}")

    except Exception as e:
        print(f"❌ Command handling failed for {msg.topic}: {e}")

# --- File-based command processing (standalone mode) ---

COMMANDS_FILE = os.path.join(SCRIPT_DIR, "commands.json")

def load_history():
    """Load existing history or return empty dict"""
    if os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, "r") as f:
                return json.load(f)
        except:
            pass
    return {}

def prune_old_data(history, cutoff_time):
    """Remove data points older than 24 hours"""
    for sensor in history:
        history[sensor] = [
            point for point in history[sensor]
            if point["timestamp"] > cutoff_time
        ]
    return history

def save_history_sample(sensors_dict):
    """Append current sensor readings to history file (all temperature sensors)"""
    history = load_history()
    current_time = time.time()
    cutoff_time = current_time - HISTORY_MAX_AGE
    
    # Add new samples (only VV temp for now)
    for sensor_name, value in sensors_dict.items():
        if sensor_name not in history:
            history[sensor_name] = []
        history[sensor_name].append({
            "timestamp": current_time,
            "value": value
        })
    
    # Prune old data
    history = prune_old_data(history, cutoff_time)
    
    # Atomic write
    history_dir = os.path.dirname(os.path.abspath(HISTORY_FILE))
    with tempfile.NamedTemporaryFile(mode="w", dir=history_dir, delete=False) as tmp:
        json.dump(history, tmp)
        tmp_path = tmp.name
    os.replace(tmp_path, HISTORY_FILE)

def process_file_commands():
    """Read commands.json, execute all select and number commands, remove processed"""
    if not os.path.exists(COMMANDS_FILE):
        return
    
    try:
        with open(COMMANDS_FILE, "r") as f:
            commands = json.load(f)
        
        if not isinstance(commands, list):
            return
        
        remaining_commands = []
        
        for cmd in commands:
            entity_id = cmd.get("entity_id")
            domain = cmd.get("domain")
            service = cmd.get("service")
            value = cmd.get("value")
            command_topic = cmd.get("command_topic")
            
            if not entity_id or not domain or not service or value is None:
                print(f"⏭️ Skipping incomplete command: {cmd}")
                continue  # Remove from queue
            
            # Map command_topic to register configuration
            cmd_cfg = command_map.get(command_topic)
            if not cmd_cfg:
                print(f"⚠️ Unknown command_topic: {command_topic}")
                continue  # Remove from queue
            
            try:
                # Handle dynamic curve registers
                if "dynamic_curve" in cmd_cfg:
                    reg_info = resolve_curve_register(cmd_cfg["dynamic_curve"])
                    if reg_info is None:
                        print(f"❌ Could not resolve register for {command_topic}")
                        remaining_commands.append(cmd)
                        continue
                    
                    register = reg_info["write"]
                    print(f"📝 File command: {entity_id} = {value} (dynamic curve reg=0x{register:02X})")
                    write_fc06(register, value)
                    print(f"✅ Command executed: {entity_id} reg=0x{register:02X} value={value}")
                
                # Handle standard registers
                else:
                    register = cmd_cfg["register"]
                    scaled = value * cmd_cfg.get("scale", 1)
                    
                    print(f"📝 File command: {entity_id} = {value} (reg=0x{register:02X})")
                    write_fc06(register, scaled)
                    print(f"✅ Command executed: {entity_id} reg=0x{register:02X} value={scaled}")
                
                # Successfully executed - don't keep in queue
                
            except Exception as e:
                print(f"❌ Failed to write {entity_id}: {e}")
                # Keep failed command in queue for retry
                remaining_commands.append(cmd)
        
        # Atomically update commands.json with only remaining (failed) commands
        commands_dir = os.path.dirname(os.path.abspath(COMMANDS_FILE))
        with tempfile.NamedTemporaryFile(mode="w", dir=commands_dir, delete=False) as tmp:
            json.dump(remaining_commands, tmp, indent=2)
            tmp_path = tmp.name
        
        os.replace(tmp_path, COMMANDS_FILE)
            
    except Exception as e:
        print(f"❌ Error processing file commands: {e}")

# --- FC06 registers discovery ---

fc06_registers = {
    0x01: "cv_mode",
    0x02: "cv_curve",
    0x03: "cv_setpoint",
    0x04: "cv_night",
    0x0A: "vv_mode",
    0x0B: "vv_setpoint",
    0x0C: "vv_schedule",
    0x0F: "aux_heating",
    0xA1: "comp_hours",
    0xA2: "vv_hours",
    0xA3: "heating_hours",
    0xD0: "curve_temp",
    0x1A: "central_heating_config",
    0x1B: "cv_max",
    0x1C: "cv_min",
    0x8D: "outdoor_cal"
}

# Special FC06 sensor definitions
special_fc06 = {
    "comp_hours": {
        "unit": "h",
        "device_class": "duration",
        "state_class": "total_increasing"
    },
    "vv_hours": {
        "unit": "h",
        "device_class": "duration",
        "state_class": "total_increasing"
    },
    "heating_hours": {
        "unit": "h",
        "device_class": "duration",
        "state_class": "total_increasing"
    }
}

# Define the valid options for each mode register
mode_options = {
    "cv_mode": ["Off", "On"],
    "cv_night": ["Timer", "Constant day", "Constant night"],
    "vv_mode": ["Off", "On", "Timer"],
    "vv_schedule": ["Timer", "Constant on", "Constant off"],
    "aux_heating": ["Off", "Automatic", "Backup operation"],
    "central_heating_config": ["Under floor heating w/o shunt",
                               "Under floor heating w. shunt",
                               "Radiator and mixed systems"]
}

# --- Samlet discovery-funktion (placeret EFTER fc06_registers/special_fc06/mode_options) ---

def publish_all_discovery() -> None:
    """Publish alle Home Assistant discovery configs (kaldes ved hver MQTT connect)."""

    # Filter coils and sensors based on detected pump type (AW vs BW)
    active_coils = filter_coils_by_type(PUMP_TYPE)
    active_fc04 = filter_fc04_by_type(PUMP_TYPE)

    # Coils -> binary_sensors
    for idx, label in active_coils.items():
        publish_discovery_binary(
            name=label,
            unique_id=f"dvi_coil_{idx}",
            coil_key=label
        )

    # FC04 sensors -> temperature sensors
    for key, label in active_fc04.items():
        publish_discovery_sensor(
            name=label,
            unique_id=f"dvi_fc04_{key}",
            value_template=f"{{{{ value_json.input_registers['{label}'] }}}}",
            unit="°C",
            device_class="temperature",
            state_class="measurement"
        )

    # Special FC04 cases
    publish_discovery_sensor(
        name="em23_power",
        unique_id="dvi_fc04_power",
        value_template="{{ value_json.input_registers['em23_power'] | float | round(3) }}",
        unit="kW",
        device_class="power",
        state_class="measurement"
    )
    publish_discovery_sensor(
        name="em23_energy",
        unique_id="dvi_fc04_energy",
        value_template="{{ value_json.input_registers['em23_energy'] }}",
        unit="kWh",
        device_class="energy",
        state_class="total_increasing"
    )

    # Install / service date as diagnostic sensors
    publish_discovery_sensor(
        name="Installation Date",
        unique_id="dvi_static_install_date",
        value_template="{{ '%s-%s-%s' | format(value_json.install_date.dd, value_json.install_date.mm, value_json.install_date.yy) }}",
        entity_category="diagnostic"
    )
    publish_discovery_sensor(
        name="Service Date",
        unique_id="dvi_static_service_date",
        value_template="{{ '%s-%s-%s' | format(value_json.service_date.dd, value_json.service_date.mm, value_json.service_date.yy) }}",
        entity_category="diagnostic"
    )

    # Pump type, ID, and firmware versions
    publish_discovery_sensor(
        name="Pump Type",
        unique_id="dvi_static_pump_type",
        value_template="{{ value_json.pump_type }}"
    )
    publish_discovery_sensor(
        name="Pump ID",
        unique_id="dvi_static_pumpid",
        value_template="{{ value_json.pumpid }}",
        entity_category="diagnostic"
    )
    publish_discovery_sensor(
        name="Firmware Version Bottom",
        unique_id="dvi_static_sw_bot",
        value_template="{{ value_json.sw_bot }}",
        entity_category="diagnostic"
    )
    publish_discovery_sensor(
        name="Firmware Version Top",
        unique_id="dvi_static_sw_top",
        value_template="{{ value_json.sw_top }}",
        entity_category="diagnostic"
    )

    # FC06 discovery
    for reg, label in fc06_registers.items():
        try:
            if label in mode_options:
                cmd_topic = {
                    "cv_mode": "dvi/command/cvstate",
                    "cv_night": "dvi/command/cvnight",
                    "vv_mode": "dvi/command/vvstate",
                    "vv_schedule": "dvi/command/vvschedule",
                    "aux_heating": "dvi/command/tvstate",
                    "central_heating_config": "dvi/command/centralheatingconfig"
                }[label]

                mapping = {
                    "cv_mode": {0: "Off", 1: "On"},
                    "cv_night": {0: "Timer", 1: "Constant day", 2: "Constant night"},
                    "vv_mode": {0: "Off", 1: "On", 2: "Timer"},
                    "vv_schedule": {0: "Timer", 1: "Constant on", 2: "Constant off"},
                    "aux_heating": {0: "Off", 1: "Automatic", 2: "Backup operation"},
                    "central_heating_config": {
                        0: "Under floor heating w/o shunt",
                        1: "Under floor heating w. shunt",
                        2: "Radiator and mixed systems"
                    }
                }[label]

                if label == "central_heating_config":
                    publish_discovery_select(
                        name=label,
                        unique_id=f"dvi_fc06_{label}",
                        command_topic=cmd_topic,
                        state_template=f"""
                          {{% set map = {mapping} %}}
                          {{{{ map[value_json.write_registers['{label}']] }}}}
                        """,
                        options=mode_options[label],
                        entity_category="config"
                    )
                else:
                    publish_discovery_select(
                        name=label,
                        unique_id=f"dvi_fc06_{label}",
                        command_topic=cmd_topic,
                        state_template=f"""
                          {{% set map = {mapping} %}}
                          {{{{ map[value_json.write_registers['{label}']] }}}}
                        """,
                        options=mode_options[label]
                   )
                print(f"🟢 Published select discovery: {label} -> {cmd_topic}")

            elif label == "cv_curve":
                publish_discovery_number(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    command_topic="dvi/command/cvcurve",
                    state_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    min_val=1,
                    max_val=20,
                    step=1
                )
                print(f"🟢 Published number discovery: {label} -> dvi/command/cvcurve")

            elif label == "vv_setpoint":
                publish_discovery_number(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    command_topic="dvi/command/vvsetpoint",
                    state_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    min_val=10,
                    max_val=60,
                    step=1,
                    unit="°C"
                )
                print(f"🟢 Published number discovery: {label} -> dvi/command/vvsetpoint")

            elif label == "cv_max":
                publish_discovery_number(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    command_topic="dvi/command/cvmax",
                    state_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    min_val=20,
                    max_val=55,
                    step=1,
                    unit="°C",
                    entity_category="config"
                )
                print(f"🟢 Published number discovery: {label} -> dvi/command/cvmax")

            elif label == "cv_min":
                publish_discovery_number(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    command_topic="dvi/command/cvmin",
                    state_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    min_val=10,
                    max_val=45,
                    step=1,
                    unit="°C",
                    entity_category="config"
                )
                print(f"🟢 Published number discovery: {label} -> dvi/command/cvmin")

            elif label == "curve_temp":
                publish_discovery_sensor(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    unit="°C",
                    device_class="temperature",
                    state_class="measurement"
                )
                print(f"🟢 Published sensor discovery: {label}")

            elif label == "cv_setpoint":
                publish_discovery_sensor(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    unit="°C",
                    device_class="temperature",
                    state_class="measurement"
                )
                print(f"🟢 Published sensor discovery: {label}")

            elif label in special_fc06:
                cfg = special_fc06[label]
                publish_discovery_sensor(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    unit=cfg.get("unit"),
                    device_class=cfg.get("device_class"),
                    state_class=cfg.get("state_class")
                )
                print(f"🟢 Published sensor discovery: {label}")

            else:
                publish_discovery_sensor(
                    name=label,
                    unique_id=f"dvi_fc06_{label}",
                    value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
                    state_class="measurement"
                )
                print(f"🟢 Published sensor discovery: {label}")

        except Exception as e:
            print(f"⚠️ Discovery generation failed for {label}: {e}")

    publish_discovery_number(
        name="curve_set_-12",
        unique_id="dvi_fc06_curve_set_-12",
        command_topic="dvi/command/curveset-12",
        state_template="{{ value_json.write_registers['curve_set_-12_read'] }}",
        min_val=10,
        max_val=80,
        step=1,
        unit="°C",
        entity_category="config"
    )
    print("🟢 Published number discovery: curve_set_-12 -> dvi/command/curveset-12")

    publish_discovery_number(
        name="curve_set_12",
        unique_id="dvi_fc06_curve_set_12",
        command_topic="dvi/command/curveset12",
        state_template="{{ value_json.write_registers['curve_set_12_read'] }}",
        min_val=10,
        max_val=80,
        step=1,
        unit="°C",
        entity_category="config"
    )
    print("🟢 Published number discovery: curve_set_12 -> dvi/command/curveset12")


# --- MQTT callbacks (EFTER publish_all_discovery er defineret) --------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to MQTT broker")
        for t in command_map:
            client.subscribe(t)
        publish_all_discovery()
    else:
        print(f"❌ MQTT connection failed with code {rc}")

mqtt_client.on_connect = on_connect
for t in command_map:
    mqtt_client.subscribe(t)
mqtt_client.on_message = on_message

# --- Netværksinfo helpers (replikeret fra functions.py) ---------------------

def _get_default_gateway_linux() -> list[str]:
    """Returnerer gateway som liste [a,b,c,d] eller ['0','0','0','0'] ved fejl."""
    try:
        with open("/proc/net/route") as fh:
            for line in fh:
                fields = line.strip().split()
                # iface, destination, flags etc.
                if fields[1] != '00000000' or not int(fields[3], 16) & 2:
                    continue
                g = socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
                parts = g.split(".")
                if len(parts) == 4:
                    print(f"ℹ️ Detected default gateway from /proc/net/route: {g}")
                    return parts
    except Exception as e:
        print(f"⚠️ Failed to read default gateway: {e}")
    return ["0", "0", "0", "0"]


def _get_default_dns_linux() -> list[str]:
    """Returnerer DNS som liste [a,b,c,d] eller ['0','0','0','0'] ved fejl."""
    try:
        data = subprocess.check_output(
            "cat /etc/resolv.conf | grep -im 1 '^nameserver' | cut -d ' ' -f2",
            shell=True,
        ).decode("utf-8").strip()
        if not data:
            data = "0.0.0.0"
    except Exception as e:
        print(f"⚠️ Failed to read DNS from /etc/resolv.conf: {e}")
        data = "0.0.0.0"
    parts = data.split(".")
    if len(parts) != 4:
        return ["0", "0", "0", "0"]
    return parts


def _get_ip_address_first_if() -> list[str]:
    """Returnerer første IP som liste [a,b,c,d] eller ['0','0','0','0']."""
    try:
        g = subprocess.check_output("hostname -I", shell=True).decode("utf-8").strip()
        if not g:
            g = "0.0.0.0"
    except Exception as e:
        print(f"⚠️ Failed to read IP address via hostname -I: {e}")
        g = "0.0.0.0"
    parts = g.split()
    if not parts:
        return ["0", "0", "0", "0"]
    ip = parts[0].split(".")
    if len(ip) != 4:
        return ["0", "0", "0", "0"]
    return ip


def _push_network_config_to_modbus() -> None:
    """
    Skriv IP, gateway og DNS til Modbus registre 211–222 samt netstatus til 466,
    som i functions.py:setIP/setNetOn.
    """
    ip = _get_ip_address_first_if()
    gw = _get_default_gateway_linux()
    dns = _get_default_dns_linux()

    print(f"ℹ️ Network info: IP={'.'.join(ip)}, GW={'.'.join(gw)}, DNS={'.'.join(dns)}")

    try:
        # IP: 211–214
        write_fc06(211, int(ip[0]))
        write_fc06(212, int(ip[1]))
        write_fc06(213, int(ip[2]))
        write_fc06(214, int(ip[3]))

        # Gateway: 215–218
        write_fc06(215, int(gw[0]))
        write_fc06(216, int(gw[1]))
        write_fc06(217, int(gw[2]))
        write_fc06(218, int(gw[3]))

        # DNS: 219–222
        write_fc06(219, int(dns[0]))
        write_fc06(220, int(dns[1]))
        write_fc06(221, int(dns[2]))
        write_fc06(222, int(dns[3]))

        print("✅ Wrote IP/gateway/DNS to Modbus (regs 211–222)")
    except Exception as e:
        print(f"❌ Failed to push network config to Modbus: {e}")

    try:
        # Netværksstatus: 466, værdi 1 (on), svarer til setNetOn i functions.py
        write_fc06(466, 1)
        print("✅ Set network status ON in Modbus (reg 466)")
    except Exception as e:
        print(f"❌ Failed to set network status in Modbus: {e}")


# Timers and persistent cache
last_coil_update = 0
last_fc04_update = 0
last_misc_update = 0
last_history_sample = 0

last_coils = {}
last_inputs = {}
last_writes = {}
last_published = None

# Curve configuration maps (for command processing)
CURVE_MAPS = {
    0: {
        "write": {12: 0x12F, -12: 0x130},
        "read": {12: 0x2F, -12: 0x30},
    },
    1: {
        "write": {12: 0x131, -12: 0x132},
        "read": {12: 0x31, -12: 0x32},
    },
    2: {
        "write": {12: 0x133, -12: 0x134},
        "read": {12: 0x33, -12: 0x34},
    },
}

# Detect pump type before starting main loop
PUMP_TYPE = detect_pump_type()
# TESTING OVERRIDE: Uncomment the next line to force BW mode for Home Assistant testing
# PUMP_TYPE = "BW"
if PUMP_TYPE:
    pump_desc = "Air-to-Water" if PUMP_TYPE == "AW" else "Brine-to-Water/Geothermal"
    print(f"✅ Operating in {PUMP_TYPE} ({pump_desc}) mode")
else:
    print("⚠️ Could not detect pump type, defaulting to AW (Air-to-Water) configuration")

# Start MQTT and push net config once at startup
mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
mqtt_client.loop_start()

# Skriv IP/gateway/DNS og netstatus til DVI via STM32 bridge ved opstart
_push_network_config_to_modbus()

while True:
    now = time.time()

    # Process file-based commands (standalone mode)
    process_file_commands()

    # Coils every 13s
    if now - last_coil_update >= 13:
        coils = read_coils()
        last_coils = dict(sorted(coils.items()))
        last_coil_update = now

    # FC04 sensors every 17s
    if now - last_fc04_update >= 17:
        fc04_raw = {}
        for reg in range(0x01, 0x0F):
            val = read_input(reg, signed=True)
            if val is not None:
                fc04_raw[f"sensor_{reg}"] = val

        # Filter FC04 sensors based on pump type
        active_fc04 = filter_fc04_by_type(PUMP_TYPE)
        for key, raw in fc04_raw.items():
            if key in omit_fc04:
                continue
            if key not in active_fc04:
                continue  # Skip sensors not applicable to this pump type
            label = active_fc04[key]
            last_inputs[label] = round(raw * 0.1, 1)

        # EM23 power (FC04)
        power = read_input(0x24)
        if power is not None:
            last_inputs["em23_power"] = round(power * 0.0001, 4)

        last_fc04_update = now

    # EM23 energy + curve temp + FC06 reads every 60s
    if now - last_misc_update >= 60:
        msw = read_input(0x25)
        lsw = read_input(0x26)
        if msw is not None and lsw is not None:
            raw_energy = (msw << 16) + lsw
            last_inputs["em23_energy"] = round(raw_energy * 0.1, 1)

        # Base FC06 set (no static curve_set entries)
        fc06_regs_60s = {
            0x01: "cv_mode",
            0x02: "cv_curve",
            0x03: "cv_setpoint",
            0x04: "cv_night",
            0x0A: "vv_mode",
            0x0B: "vv_setpoint",
            0x0C: "vv_schedule",
            0x0F: "aux_heating",
            0xA1: "comp_hours",
            0xA2: "vv_hours",
            0xA3: "heating_hours",
            0xD0: "curve_temp",
            0x1A: "central_heating_config",
            0x1B: "cv_max",
            0x1C: "cv_min",
            0x8D: "outdoor_cal"
        }

        # Define adjustments: reg -> (multiplier, decimals)
        fc06_adjustments = {
            0xD0: (0.1, 1),   # curve_temp
            0x8D: (0.1, 1)    # outdoor_cal
        }

        # Define which FC06 registers are signed
        signed_fc06 = {0x8D}  # outdoor_cal
        
        # Read base FC06 set
        for reg, label in fc06_regs_60s.items():
            val = read_via_fc06(reg, signed=(reg in signed_fc06))
            if val is not None:
                if reg in fc06_adjustments:
                    mult, decimals = fc06_adjustments[reg]
                    last_writes[label] = round(val * mult, decimals)
                else:
                    last_writes[label] = val

        # Use numeric config value to decide curve_set registers
        config_val = last_writes.get("central_heating_config")
        if isinstance(config_val, int):
            curve_maps = {
                0: {
                    "write": {"12": 0x12F, "-12": 0x130},
                    "read": {"12": 0x2F, "-12": 0x30},
                },
                1: {
                    "write": {"12": 0x131, "-12": 0x132},
                    "read": {"12": 0x31, "-12": 0x32},
                },
                2: {
                    "write": {"12": 0x133, "-12": 0x134},
                    "read": {"12": 0x33, "-12": 0x34},
                },
            }
            curve_cfg = curve_maps.get(config_val)
            if curve_cfg:
                for key, reg in curve_cfg["write"].items():
                    val = read_via_fc06(reg)
                    if val is not None:
                        last_writes[f"curve_set_{key}_write"] = val
                for key, reg in curve_cfg["read"].items():
                    val = read_via_fc06(reg)
                    if val is not None:
                        read_key = f"curve_set_{key}_read"
                        last_writes[read_key] = val
                        last_writes[f"curve_set_{key}"] = val  # backwards compatibility

        last_misc_update = now

    # Sample sensor history every 60 seconds (all temperature sensors)
    if now - last_history_sample >= HISTORY_SAMPLE_INTERVAL:
        # Build history sensors dict based on available sensors
        history_sensors = {}
        active_fc04 = filter_fc04_by_type(PUMP_TYPE)
        
        # Only include sensors that are applicable to this pump type
        for sensor_label in active_fc04.values():
            val = last_inputs.get(sensor_label)
            if val is not None:
                history_sensors[sensor_label] = val
        
        # Add defrost status if available
        defrost_coil = "4-way valve defrost" if PUMP_TYPE == "AW" else "Circ. pump geothermal"
        if defrost_coil in last_coils:
            history_sensors["Defrost"] = 1 if last_coils.get(defrost_coil) else 0
        
        if history_sensors:
            save_history_sample(history_sensors)
            sensor_list = ", ".join([f"{k}={v}°C" if k != "Defrost" else f"{k}={'ON' if v else 'OFF'}" for k, v in history_sensors.items()])
            print(f"📊 History sample: {sensor_list}")
        last_history_sample = now

    # Final payload from cached values
    full_payload = {
        "coils": last_coils,
        "input_registers": dict(sorted(last_inputs.items())),
        "write_registers": dict(sorted(last_writes.items()))
    }
    # Eksponér pumpid, pump_type, SW-versioner + install/service date i measurement payload
    if PUMP_TYPE:
        full_payload["pump_type"] = PUMP_TYPE
    if PUMP_ID:
        full_payload["pumpid"] = PUMP_ID
    if SWBOT:
        full_payload["sw_bot"] = SWBOT
    if SWTOP:
        full_payload["sw_top"] = SWTOP
    if INSTALL_DD and INSTALL_MM and INSTALL_YY:
        full_payload["install_date"] = {
            "dd": INSTALL_DD,
            "mm": INSTALL_MM,
            "yy": INSTALL_YY,
        }
    if SERVICE_DD and SERVICE_MM and SERVICE_YY:
        full_payload["service_date"] = {
            "dd": SERVICE_DD,
            "mm": SERVICE_MM,
            "yy": SERVICE_YY,
        }

    # Only publish if payload changed
    if full_payload != last_published:
        mqtt_client.publish("dvi/measurement", json.dumps(full_payload))
        write_state_atomic(full_payload)
        last_published = full_payload

    time.sleep(1)