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

### 1. download the scrips package on the Pi

```bash
cd /home/dviha
wget https://raw.githubusercontent.com/ruteclrp/dvi-bridge-standalone/main/bridge_assets/dvi-installation-scripts.rpi.tar.gz
tar -xzf dvi-installation-scripts-v6.21.rpi.tar.gz
cd installation-scripts
chmod +x install.sh
```

### 2. Install the bridge RPi

```bash
./install.sh
```
When prompted, press Enter for latest release or the number of a previous release (1-10)

When prompted, select the install mode:
1 for basic bridge install w/o sidecar standalone webserver
2 for bridge + sidecar
3 for full package

When prompted, edit the .env in nano and fill in the mqtt host IP, mqtt user and mqtt password
Ctrl-o and Enter for save, Ctrl-x for exit to script completion

When the install is complete, the bridge service should be running and you will be prompted to check its status.

If sidecar is also installed, the webbridge service will also be running and you will be prompted to check its status.

You may see an error in the bridge service due to the dviha user not being member of the dialout group giving access to the /dev/ttyACM0 (or whatever other ttyAxxx port name it has).

If that happens:
Step 1: Check which group owns the device
```bash
ls -l /dev/ttyACM0 - and you should see something like this: crw-rw---- 1 root dialout 166, 0 ... /dev/ttyACM0
```
Step 2: Add your user to the dialout group
```bash
sudo usermod -aG dialout dviha
```
Step 3: Log out and back in

Start the bridge service manually
```bash
sudo systemctl start webbridge.service
sudo journalctl -u bridge.service -f
```

Home Assistant will receive MQTT discovery messages so entities are created automatically.

---------------------------------------------------------------------------------------------------------------------------------

If you chose a different username than `dviha` when installing Raspberry Pi OS, remember to update the username in all the paths and examples in this guide.

---------------------------------------------------------------------------------------------------------------------------------

When the bridge is **active (running)** and your USB connection + Modbus wiring are correct, the heatpump data should be visible in Home Assistant via MQTT discovery.

When you run the sidecar standalone application webbridge.py you can access the heatpump via a browser on http://<IP of your bridge computer>:5000
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

**“DVI Heatpump Card”**

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
4. Select **DVI Heatpump Card**

### Automatic configuration (no YAML needed)
When adding the card in Lovelace, a full configuration UI appears.

1. Select your DVI MQTT device from the dropdown.
2. The card automatically detects all related entities on that device.
3. All fields are mapped instantly — no manual configuration required.

The auto-mapping includes:

* Heatpump state (Stand-by / On)
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
  - Heatpump state (On/St-by)
- Popups for Info, CV, VV, AUX, On/St-by
- Special popup for setting the heat curve, with automatic configuration of setpoints for Floor heating w or w/o shunt, or radiator 
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
