# dvi-bridge-standalone
Python bridge for DVI heatpump to run on RPi connected via USB

## Auto-start with systemd
Copy the service file:
```bash
sudo cp systemd/bridge.service.example /etc/systemd/system/bridge.service
sudo systemctl daemon-reload
sudo systemctl enable bridge.service
sudo systemctl start bridge.service

#Now with auto discovery for mqtt
