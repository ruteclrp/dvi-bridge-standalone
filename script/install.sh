```bash
#!/usr/bin/env bash
set -e

REPO="ruteclrp/dvi-bridge-standalone"
VERSION="$1"

if [ -z "$VERSION" ]; then
  echo "Usage: ./install.sh vX.Y.Z"
  exit 1
fi

BASE="/home/dviha/dvi-bridge"
RELEASE_DIR="$BASE/releases/$VERSION"
TARBALL="bridge-runtime.tar.gz"
URL="https://github.com/$REPO/releases/download/$VERSION/$TARBALL"

echo "== Creating directory structure =="
mkdir -p "$BASE/releases"

echo "== Downloading runtime package $VERSION =="
cd /tmp
wget -q "$URL" -O "$TARBALL"

echo "== Extracting runtime =="
mkdir -p "$RELEASE_DIR"
tar xzf "$TARBALL" -C "$RELEASE_DIR" --strip-components=1

echo "== Creating Python virtual environment =="
if [ ! -d "$BASE/venv" ]; then
  python3 -m venv "$BASE/venv"
fi

source "$BASE/venv/bin/activate"
pip install --upgrade pip
pip install -r "$RELEASE_DIR/requirements.txt"

echo "== Creating .env file =="
if [ ! -f "$BASE/.env" ]; then
  cat > "$BASE/.env" << 'EOF'
# MQTT Configuration
MQTT_HOST=localhost
MQTT_PORT=1883
MQTT_USER=
MQTT_PASS=

# Static values (auto-populated by read_static_values_modbustk.py)
# FABNR=
# SWBOT=
# SWTOP=
# PUMP_TYPE=
# INSTALL_DD=
# INSTALL_MM=
# INSTALL_YY=
# SERVICE_DD=
# SERVICE_MM=
# SERVICE_YY=
EOF
  echo "Created $BASE/.env - Edit MQTT settings as needed"
fi

echo "== Updating current symlink =="
ln -sfn "$RELEASE_DIR" "$BASE/current"

echo "== Installing systemd services =="
for service in "$RELEASE_DIR/systemd/"*.service.example; do
  sudo cp "$service" "/etc/systemd/system/$(basename "${service%.example}")"
done

sudo systemctl daemon-reload
sudo systemctl enable bridge.service
sudo systemctl start bridge.service

echo "== Installation complete =="
echo "Bridge running from $RELEASE_DIR"
```
