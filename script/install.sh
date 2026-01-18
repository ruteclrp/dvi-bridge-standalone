```bash
#!/usr/bin/env bash
set -e

REPO="ruteclrp/dvi-bridge-standalone"
VERSION="$1"
MODE="${2:-basic}"  # basic, sidecar, or complete

if [ -z "$VERSION" ]; then
  echo "Usage: ./install.sh vX.Y.Z [mode]"
  echo "  mode: basic (default), sidecar, complete"
  echo ""
  echo "Examples:"
  echo "  ./install.sh v1.0.0 basic     - Install bridge only"
  echo "  ./install.sh v1.0.0 sidecar   - Install bridge + sidecar web interface"
  echo "  ./install.sh v1.0.0 complete  - Install all components (legacy)"
  exit 1
fi

BASE="/home/dviha/dvi-bridge"
RELEASE_DIR="$BASE/releases/$VERSION"

echo "== Creating directory structure =="
mkdir -p "$BASE/releases"
cd /tmp

case "$MODE" in
  basic)
    echo "== Installing Bridge Basic (MQTT only) =="
    TARBALL="dvi-bridge-basic-$VERSION.tar.gz"
    URL="https://github.com/$REPO/releases/download/$VERSION/$TARBALL"
    wget -q "$URL" -O "$TARBALL"
    mkdir -p "$RELEASE_DIR"
    tar xzf "$TARBALL" -C "$RELEASE_DIR" --strip-components=1
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=false
    ;;
  
  sidecar)
    echo "== Installing Bridge + Sidecar Web Interface =="
    # Download bridge basic
    BRIDGE_TAR="dvi-bridge-basic-$VERSION.tar.gz"
    wget -q "https://github.com/$REPO/releases/download/$VERSION/$BRIDGE_TAR" -O "$BRIDGE_TAR"
    mkdir -p "$RELEASE_DIR"
    tar xzf "$BRIDGE_TAR" -C "$RELEASE_DIR" --strip-components=1
    
    # Download sidecar
    SIDECAR_TAR="dvi-sidecar-$VERSION.tar.gz"
    wget -q "https://github.com/$REPO/releases/download/$VERSION/$SIDECAR_TAR" -O "$SIDECAR_TAR"
    tar xzf "$SIDECAR_TAR" -C "$RELEASE_DIR" --strip-components=1
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=true
    ;;
  
  complete)
    echo "== Installing Complete Runtime (All components) =="
    TARBALL="dvi-complete-runtime-$VERSION.tar.gz"
    URL="https://github.com/$REPO/releases/download/$VERSION/$TARBALL"
    wget -q "$URL" -O "$TARBALL"
    mkdir -p "$RELEASE_DIR"
    tar xzf "$TARBALL" -C "$RELEASE_DIR" --strip-components=1
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=true
    ;;
  
  *)
    echo "Error: Invalid mode '$MODE'. Use: basic, sidecar, or complete"
    exit 1
    ;;
esac

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
if [ "$INSTALL_BRIDGE" = true ]; then
  if [ -f "$RELEASE_DIR/systemd/bridge.service.example" ]; then
    sudo cp "$RELEASE_DIR/systemd/bridge.service.example" "/etc/systemd/system/bridge.service"
    sudo systemctl daemon-reload
    sudo systemctl enable bridge.service
    sudo systemctl start bridge.service
    echo "✓ Bridge service installed and started"
  fi
fi

if [ "$INSTALL_SIDECAR" = true ]; then
  if [ -f "$RELEASE_DIR/systemd/webbridge.service.example" ]; then
    sudo cp "$RELEASE_DIR/systemd/webbridge.service.example" "/etc/systemd/system/webbridge.service"
    sudo systemctl daemon-reload
    sudo systemctl enable webbridge.service
    sudo systemctl start webbridge.service
    echo "✓ Sidecar web interface installed and started"
  fi
fi

echo ""
echo "== Installation complete =="
echo "Version: $VERSION"
echo "Mode: $MODE"
echo "Location: $RELEASE_DIR"

if [ "$INSTALL_SIDECAR" = true ]; then
  echo ""
  echo "Web interface available at: http://$(hostname -I | awk '{print $1}'):5555"
fi
```
