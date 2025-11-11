# dvi-bridge-standalone
Python bridge for DVI heatpump to run on RPi connected via USB

# setup of the RPi to execute the bridge.py
1. Install a Raspberry Pi OS Lite (64-bit) with the Raspberry Pi Imager application.
2. Enable SSH if you wish to be able to remotely manage the RPi
3. update the machine and install the packages in the requirements file

## Auto-start with systemd
Copy the service file:
```bash
sudo cp systemd/bridge.service.example /etc/systemd/system/bridge.service
sudo systemctl daemon-reload
sudo systemctl enable bridge.service
sudo systemctl start bridge.service
```

# Now with auto discovery for mqtt
