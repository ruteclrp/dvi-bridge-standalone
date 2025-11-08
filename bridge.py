from dotenv import load_dotenv

import os
import minimalmodbus
import paho.mqtt.client as mqtt
import struct
import json
import time
import threading
import warnings
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

load_dotenv()  # this will read .env in the current directory
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Modbus setup
instrument = minimalmodbus.Instrument(
    '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_48D874673036-if00', 0x10)
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 2
instrument.mode = minimalmodbus.MODE_RTU

modbus_lock = threading.Lock()

# Read credentials and broker info from environment variables
MQTT_USER = os.getenv("MQTT_USER", "default_user")
MQTT_PASS = os.getenv("MQTT_PASS", "default_pass")
MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

# MQTT setup
mqtt_client = mqtt.Client()
mqtt_client.enable_logger(logger)
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to MQTT broker")
        for t in command_map:
            client.subscribe(t)
    else:
        print(f"❌ MQTT connection failed with code {rc}")

mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)

def publish_discovery_sensor(name, unique_id, value_template,
                             unit=None, device_class=None, state_class=None):
    config_topic = f"homeassistant/sensor/{unique_id}/config"
    payload = {
        "name": name,
        "state_topic": "dvi/measurement",
        "value_template": value_template,
        "unique_id": unique_id,
        "device": {
            "name": "DVI LV12",
            "identifiers": ["dvi_lv12"],
            "manufacturer": "DVI",
            "model": "LV12 Heatpump"
        }
    }
    if unit: payload["unit_of_measurement"] = unit
    if device_class: payload["device_class"] = device_class
    if state_class: payload["state_class"] = state_class
#    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)
    msg = json.dumps(payload)
    mqtt_client.publish(config_topic, msg, retain=True)
#    print(f"[DISCOVERY] Published to {config_topic}: {msg}")



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
        "device": {
            "name": "DVI LV12",
            "identifiers": ["dvi_lv12"],
            "manufacturer": "DVI",
            "model": "LV12 Heatpump"
        }
    }
    if device_class: payload["device_class"] = device_class
    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)

def publish_discovery_number(name, unique_id, command_topic, state_template,
                             min_val=0, max_val=100, step=1, unit=None):
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
        "device": {
            "name": "DVI LV12",
            "identifiers": ["dvi_lv12"],
            "manufacturer": "DVI",
            "model": "LV12 Heatpump"
        }
    }
    if unit: payload["unit_of_measurement"] = unit
    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)

def publish_discovery_select(name, unique_id, command_topic, state_template, options):
    config_topic = f"homeassistant/select/{unique_id}/config"
    payload = {
        "name": name,
        "command_topic": command_topic,
        "state_topic": "dvi/measurement",
        "value_template": state_template,
        "unique_id": unique_id,
        "options": options,
        "device": {
            "name": "DVI LV12",
            "identifiers": ["dvi_lv12"],
            "manufacturer": "DVI",
            "model": "LV12 Heatpump"
        }
    }
    msg = json.dumps(payload)
    mqtt_client.publish(config_topic, msg, retain=True)
    print(f"[DISCOVERY] Select {name} -> {config_topic}: {msg}")

# Coil mapping (coil 13 omitted)
coil_names = {
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

# FC04 sensor mapping and filtering
omit_fc04 = {"sensor_4", "sensor_8", "sensor_9", "sensor_10", "sensor_13", "sensor_14"}
fc04_labels = {
    "sensor_1": "CV Forward",
    "sensor_2": "CV Return",
    "sensor_3": "Storage tank VV",
    "sensor_5": "Storage tank CV",
    "sensor_6": "Evaporator",
    "sensor_7": "Outdoor",
    "sensor_11": "Compressor HP",
    "sensor_12": "Compressor LP"
}

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

        return dict(sorted({coil_names[i]: bits[i] for i in coil_names}.items()))
    except Exception as e:
        print(f"FC01 read failed: {e}")
        return {}

def read_input(register):
    try:
        with modbus_lock:
            return instrument.read_register(register, number_of_decimals=0, functioncode=4)
    except Exception as e:
        print(f"FC04 read failed for 0x{register:02X}: {e}")
        return None

def read_via_fc06(register):
    try:
        with modbus_lock:
            payload = struct.pack('>HH', register, 0x0000)
            response = instrument._perform_command(6, payload)
            _, value = struct.unpack('>HH', response)
            return value
    except Exception as e:
        print(f"FC06 echo failed for 0x{register:02X}: {e}")
        return None

# --- MQTT command handling for Modbus writes ---
command_map = {
    "dvi/command/vvstate": {"register": 0x10A, "scale": 1},
    "dvi/command/cvstate": {"register": 0x101, "scale": 1},
    "dvi/command/cvcurve": {"register": 0x102, "scale": 1},
    "dvi/command/vvsetpoint": {"register": 0x10B, "scale": 1},
    "dvi/command/tvstate": {"register": 0x10F, "scale": 1},
}

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload_str = msg.payload.decode().strip()
        cfg = command_map.get(topic)
        if not cfg:
            return
        value_raw = int(payload_str)
        scaled = value_raw * cfg.get("scale", 1)
        with modbus_lock:
            instrument.write_register(cfg["register"], scaled, 0, functioncode=6)
 #       print(f"✅ FC06 write: topic={topic} value={value_raw} reg=0x{cfg['register']:02X}")
    except Exception as e:
        print(f"❌ Command handling failed for {msg.topic}: {e}")

mqtt_client.on_message = on_message

# Timers and persistent cache
last_coil_update = 0
last_fc04_update = 0
last_misc_update = 0

last_coils = {}
last_inputs = {}
last_writes = {}
last_published = None

mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
mqtt_client.loop_start()

# --- Auto-generate discovery configs once at startup ---

# Coils -> binary_sensors
for idx, label in coil_names.items():
    publish_discovery_binary(
        name=label,
        unique_id=f"dvi_coil_{idx}",
        coil_key=label
    )

# FC04 sensors -> temperature sensors
for key, label in fc04_labels.items():
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
    value_template="{{ value_json.input_registers['em23_power'] }}",
    unit="W",
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

# --- FC06 registers discovery ---

mode_options = {
    "cv_mode": ["Off", "Heating", "Cooling"],
    "vv_mode": ["Off", "Domestic Hot Water", "Heating"],
    "aux_heating": ["Disabled", "Enabled"]
}

fc06_registers = {
    0x01: "cv_mode",
    0x02: "cv_curve",
    0x03: "cv_setpoint",
    0x04: "cv_night_setback",
    0x0A: "vv_mode",
    0x0B: "vv_setpoint",
    0x0C: "vv_schedule",
    0x0F: "aux_heating",
    0xA1: "comp_hours",
    0xA2: "vv_hours",
    0xA3: "heating_hours",
    0xD0: "curve_temp"
}

for reg, label in fc06_registers.items():
    # Mode registers -> Selects
    if label in mode_options:
        # Map to correct command topic from command_map
        cmd_topic = {
            "cv_mode": "dvi/command/cvstate",
            "vv_mode": "dvi/command/vvstate",
            "aux_heating": "dvi/command/tvstate"
        }[label]

        publish_discovery_select(
            name=label,
            unique_id=f"dvi_fc06_{label}",
            command_topic=cmd_topic,
            state_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
            options=mode_options[label]
        )

    # Numeric writable registers -> Numbers
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

    elif label == "cv_setpoint":
        # Read-only -> Sensor
        publish_discovery_sensor(
            name=label,
            unique_id=f"dvi_fc06_{label}",
            value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
            unit="°C",
            device_class="temperature",
            state_class="measurement"
        )

    # Read-only registers -> Sensors
    else:
        publish_discovery_sensor(
            name=label,
            unique_id=f"dvi_fc06_{label}",
            value_template=f"{{{{ value_json.write_registers['{label}'] }}}}",
            state_class="measurement"
        )


# Main loop
while True:
    now = time.time()

    # Coils every 13s
    if now - last_coil_update >= 13:
        coils = read_coils()
        last_coils = dict(sorted(coils.items()))
        last_coil_update = now

    # FC04 sensors every 17s
    if now - last_fc04_update >= 17:
        fc04_raw = {}
        for reg in range(0x01, 0x0F):
            val = read_input(reg)
            if val is not None:
                fc04_raw[f"sensor_{reg}"] = val

        for key, raw in fc04_raw.items():
            if key in omit_fc04:
                continue
            label = fc04_labels.get(key, key)
            last_inputs[label] = round(raw * 0.1, 1)

        # EM23 power (FC04)
        power = read_input(0x24)
        if power is not None:
            last_inputs["em23_power"] = round(power * 0.0001, 4)

        last_fc04_update = now

    # EM23 energy + curve temp + FC06 dummy reads every 60s
    if now - last_misc_update >= 60:
        msw = read_input(0x25)
        lsw = read_input(0x26)
        if msw is not None and lsw is not None:
            raw_energy = (msw << 16) + lsw
            last_inputs["em23_energy"] = round(raw_energy * 0.1, 1)

        # FC06 dummy reads
        fc06_registers = {
            0x01: "cv_mode",
            0x02: "cv_curve",
            0x03: "cv_setpoint",
            0x04: "cv_night_setback",
            0x0A: "vv_mode",
            0x0B: "vv_setpoint",
            0x0C: "vv_schedule",
            0x0F: "aux_heating",
            0xA1: "comp_hours",
            0xA2: "vv_hours",
            0xA3: "heating_hours",
            0xD0: "curve_temp"
        }

        # Define adjustments: reg -> (multiplier, decimals)
        fc06_adjustments = {
            0xD0: (0.1, 1),   # curve_temp
        }

        for reg, label in fc06_registers.items():
            val = read_via_fc06(reg)
            if val is not None:
                if reg in fc06_adjustments:
                    mult, decimals = fc06_adjustments[reg]
                    last_writes[label] = round(val * mult, decimals)
                else:
                    last_writes[label] = val

        last_misc_update = now

    # Final payload from cached values
    full_payload = {
        "coils": last_coils,
        "input_registers": dict(sorted(last_inputs.items())),
        "write_registers": dict(sorted(last_writes.items()))
    }

    # Only publish if payload changed
    if full_payload != last_published:
        mqtt_client.publish("dvi/measurement", json.dumps(full_payload))
#        print("📡 Published:", json.dumps(full_payload, indent=2))
        last_published = full_payload

    time.sleep(1)
