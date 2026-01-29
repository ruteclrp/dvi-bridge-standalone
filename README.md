# DVI LV Heatpump – Bridge & Home Assistant Card

This document describes both:

1. **DVI Modbus‑MQTT bridge** for Raspberry Pi (talks Modbus RTU with the heatpump and publishes data via MQTT with Home Assistant auto‑discovery).
2. **DVI LV Heatpump Lovelace card** – a custom frontend card that visualises the heatpump diagram, modes and temperatures and lets you change key settings directly from Home Assistant.

The repository is designed so both parts can live in the same project (bridge + card) and be distributed via HACS as a frontend card.

---

## A. Raspberry Pi Modbus‑MQTT Bridge

### Prerequisites

- Raspberry Pi OS Lite (64‑bit) – when you install, **set the default username to `dviha`** to match the paths and examples in this guide.
- USB connection to DVI LV heatpump
- MQTT broker (e.g. the Mosquitto broker add‑on in Home Assistant)
- Git + Python 3.9+ on the Pi

### 1. Install required packages on the Pi

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3 python3-pip python3-venv
```

### 2. Clone the repository

```bash
cd /home/dviha
git clone https://github.com/ruteclrp/dvi-bridge-standalone.git
cd dvi-bridge-standalone
```

### 3. Create the Python virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Configure MQTT environment if you run against a Home Assistant device

Copy the example environment file and edit it with your broker settings:

```bash
cp .env.example .env
nano .env
```

Example `.env`:

  MQTT_HOST=192.168.x.xxx
  MQTT_PORT=1883

  # Optional: only set these if your MQTT broker requires authentication.
  # If left empty, the bridge will connect without username/password.
  MQTT_USER=
  MQTT_PASS=

  # Optional: replace with your specific model e.g. LV7, LV9, LV12, LV16
  # This is used to set correct topics and units (default is LV) - will be used in future updates
  HEATPUMP_MODEL=LV

Leave `MQTT_USER` / `MQTT_PASS` empty if your broker does not require authentication.

### 5. Test the bridge manually

```bash
cd dvi-bridge-standalone
source .venv/bin/activate
python bridge.py
```

You should see something like:

```text
✅ Connected to MQTT broker
```

If everything is OK, the bridge will start polling the heatpump and publish JSON payloads on the topic:

```text
dvi/measurement
```
You may receive an error due to the dviha user not being member of the dialout group giving access to the /dev/ttyACM0 (or whatever oterh ttyAxxx port name it has).

If that happens:
Step 1: Check which group owns the device
```bash
ls -l /dev/ttyACM0 - and you should see something like this: crw-rw---- 1 root dialout 166, 0 ... /dev/ttyACM0
```
Step 2: Add your user to the dialout group
```BASH
sudo usermod -aG dialout dviha
```
Step 3: Log out and back in

Test again manually

Home Assistant will also receive MQTT discovery messages so entities are created automatically.
### 6. Running the bridge as standalone with sidecar

Test the sidecar
```bash
cd dvi-bridge-standalone
source .venv/bin/activate
python webbridge.py
```


### 7. Install the systemd service (auto‑start on boot)

Copy the example service files and edit them:
If you run against HA you should only setup the systemd for running bridge.py

If you run locally with sidecar and web access you should setup a systemd for both bridge.py and webbridge.py

```bash
# Copy and customize the files
cp systemd/bridge.service.example systemd/bridge.service
cp systemd/webbridge.service.example systemd/webbridge.service

# Edit both files to replace <user> with your actual username
nano systemd/bridge.service
nano systemd/webbridge.service

# Copy to systemd
sudo cp systemd/bridge.service /etc/systemd/system/
sudo cp systemd/webbridge.service /etc/systemd/system/

# Enable and start both services
sudo systemctl daemon-reload
sudo systemctl enable bridge.service webbridge.service
sudo systemctl start bridge.service webbridge.service

# Check status
sudo systemctl status bridge.service webbridge.service

```

If you chose a different username than `dviha` when installing Raspberry Pi OS, remember to update the username in all the paths and examples in this guide.

Reload systemd and enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable bridge.service
sudo systemctl start bridge.service
```

You can check the status with:

```bash
sudo systemctl status bridge.service
```

When it is **active (running)** and your USB connection + Modbus wiring are correct, the heatpump data should be visible in Home Assistant via MQTT discovery.

When you run the sidecar standalone application webbridge.py you can access the heatpump via a browser on http://<IPof your bridge computer>:5000


---

## B. Home Assistant MQTT Entities

The bridge publishes a single JSON payload to:

```text
dvi/measurement
```

Example payload:

```json
{
  "coils": {
    "Soft starter Compressor": 1,
    "Circ. pump CV": 1
  },
  "input_registers": {
    "Outdoor": 4.6,
    "Storage tank CV": 36.9,
    "Storage tank VV": 52.9
  },
  "write_registers": {
    "cv_mode": 1,
    "vv_mode": 1,
    "vv_setpoint": 55,
    "curve_temp": 36.7
  }
}
```

On startup the bridge sends Home Assistant MQTT discovery messages for:

- Temperature sensors
- Binary sensors (coils)
- `number`/`select` entities for writable FC06 registers (modes, setpoints, etc.)

As long as MQTT discovery is enabled in Home Assistant, all entities will appear automatically and are ready to be used by the custom Lovelace card.

---

## C. 

# DVI AW/BW Heatpump Lovelace Card

The **DVI Heatpump card** provides a full visual diagram of your AW or BW heatpump, live temperatures, compressor/pump status, mode controls, animated overlays, and popup panels for detailed settings.

It is fully compatible with HACS and includes a visual configuration editor.

---

## 🔧 Install via HACS

### 1. Open HACS  
Home Assistant → **HACS** → *Frontend* → ⋮ (menu) → **Custom repositories**

### 2. Add the repository  
- **URL:**  
  `https://github.com/ruteclrp/dvi-bridge-standalone`
- **Category:** `Dashboard`

Click **Add**.

### 3. Install the card  
Go to **HACS → Frontend**, find:

**“DVI LV Heatpump Card”**

Click **Download**.  
HACS installs everything automatically into:

```
/config/www/community/dvi-bridge-standalone/
```

### 4. Reload browser  
Press **Ctrl+F5** (or full refresh on mobile) to ensure the new card files load.

---

## 🧩 Adding the card in Lovelace

1. Open any dashboard  
2. Click **Edit dashboard**  
3. Click **Add card**  
4. Select **DVI LV Heatpump Card**

### Automatic configuration (no YAML needed)
When adding the card in Lovelace, a full configuration UI appears.

1. Select your DVI LV MQTT device from the dropdown.
2. The card automatically detects all related entities on that device.
3. All fields are mapped instantly — no manual configuration required.

The auto-mapping includes:

* Heatpump state (Stand by / On)
* CV mode
* VV mode
* CV night mode
* VV schedule
* AUX electric heater mode
* Pump type (AW or BW) for correct diagram display
* All relevant temperature sensors (including BW cold side sensors)
* Pump / compressor / defrost binary sensors
* Entities for the popup panels (Info / CV / VV / AUX / Heatpump)

**Note:** The `pump_type` entity (sensor.pump_type) is automatically detected and used to display the correct diagram (AW = Air-to-Water with evaporator, BW = Brine-to-Water/Geothermal with cold side sensors). Make sure this entity is included in your configuration.

If you prefer full control, you can still override or manually edit any field in the Advanced entity mapping section.

---

## 📘 Requirements

### Optional (recommended)
**browser_mod** (installed from HACS)  
Enables beautiful popup panels when clicking the top mode chips.

Add to configuration if required:

```yaml
browser_mod:
```

---

## 🎨 Features

- Animated AW or BW heating circuit diagram  
- Real‑time temperatures drawn directly in the diagram  
- Live compressor, CV pump, evaporator and defrost icon (AW), brine circuit (BW)   
- Mode bar with:
  - CV mode + sun/moon/clock based on night mode  
  - VV mode + schedule indicator  
  - AUX heating status  
  - Info chip showing EM23 power (if installed)
- Popups for Info, CV, VV, AUX  
- Buttons for:
  - CV/VV ON/OFF  
  - CV night mode (Timer / Day / Night)  
  - AUX modes (Off / Auto / On)  
  - VV setpoint ±1°C  
- Fully HACS‑compatible asset loading (JS + images auto‑loaded)

---

## 🛠 Troubleshooting

**Card does not appear in Add Card menu?**  
- Clear cache (Ctrl+F5)  
- Check **Settings → Dashboards → Resources**

**Popups not opening?**  
- Install `browser_mod`  
- Restart Home Assistant

**Entity not found?**  
- Check MQTT entity names: *Settings → Devices & Services → MQTT*

---

## 📄 License
MIT License


## E. Energy Dashboard (optional)

Home Assistant’s **Energy dashboard** can be extended with a sensor that reports the heatpump’s electricity consumption and (optionally) generated heat energy.

With the EM23 meter you already have:

- `sensor.dvi_lv_em23_power` (instant kW)
- `sensor.dvi_lv_em23_energy` (total kWh, `state_class: total_increasing`)

To use this in the Energy dashboard:

1. Make sure `sensor.dvi_lv_em23_energy` has:
   - `device_class: energy`
   - `unit_of_measurement: kWh`
   - `state_class: total_increasing`  
   (the bridge already publishes these via discovery)
2. Go to **Settings → Dashboards → Energy → Electricity grid → Setup** and select `sensor.dvi_lv_em23_energy` as a consumption source.

If you later add a calculated **heat output** sensor (COP × electric energy), you can add that as a custom energy source as well.

---

## F. Notes & Troubleshooting

- If the card does not load, open the browser console and check for 404 errors on the JS file or images.
- If popups do not appear when clicking the chips:
  - Confirm that `browser_mod` is installed and loaded.
  - Make sure your config includes the `browser_mod:` section (if required by your version).
- If some entities show as `entity not found`, open **Settings → Devices & Services → MQTT** and verify which entity IDs were created, then update the Lovelace YAML accordingly.
- If Modbus values look wrong (e.g. very large numbers instead of negative temperatures), double‑check the Modbus register format and scaling in `bridge.py`.
