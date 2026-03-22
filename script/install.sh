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
    TARBALL="dvi-bridge-basic-$VERSION.rpi.tar.gz"
    URL="https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$TARBALL"
    wget -q "$URL" -O "$TARBALL"
    mkdir -p "$RELEASE_DIR"
    tar xzf "$TARBALL" -C "$RELEASE_DIR" --strip-components=1
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=false
    ;;
  
  sidecar)
    echo "== Installing Bridge + Sidecar Web Interface =="
    # Download bridge basic
    BRIDGE_TAR="dvi-bridge-basic-$VERSION.rpi.tar.gz"
    wget -q "https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$BRIDGE_TAR" -O "$BRIDGE_TAR"
    mkdir -p "$RELEASE_DIR"
    tar xzf "$BRIDGE_TAR" -C "$RELEASE_DIR" --strip-components=1
    
    # Download sidecar
    SIDECAR_TAR="dvi-sidecar-$VERSION.rpi.tar.gz"
    wget -q "https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$SIDECAR_TAR" -O "$SIDECAR_TAR"
    tar xzf "$SIDECAR_TAR" -C "$RELEASE_DIR" --strip-components=1
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=true
    ;;
  
  complete)
    echo "== Installing Complete Runtime (All components) =="
    TARBALL="dvi-complete-runtime-$VERSION.rpi.tar.gz"
    URL="https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$TARBALL"
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

echo "== Installing cloudflared (for remote access) =="
if ! command -v cloudflared &> /dev/null; then
  ARCH=$(uname -m)
  if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64"
  elif [ "$ARCH" = "armv7l" ] || [ "$ARCH" = "armv6l" ]; then
    CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm"
  else
    CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64"
  fi
  
  echo "  Downloading cloudflared for $ARCH..."
  sudo wget -q "$CLOUDFLARED_URL" -O /usr/local/bin/cloudflared
  sudo chmod +x /usr/local/bin/cloudflared
  echo "✓ cloudflared installed"
else
  echo "✓ cloudflared already installed"
fi

echo "== Setting up device registration =="
sudo mkdir -p /etc/dvi-bridge
sudo mkdir -p /etc/cloudflared

# Copy .env if it exists in the release (with real credentials)
if [ -f "$RELEASE_DIR/sidecar/.env" ]; then
  sudo cp "$RELEASE_DIR/sidecar/.env" /etc/dvi-bridge/.env
  echo "✓ Registration configuration copied"
elif [ -f "$RELEASE_DIR/sidecar/.env.example" ]; then
  sudo cp "$RELEASE_DIR/sidecar/.env.example" /etc/dvi-bridge/.env
  echo "⚠️  Using .env.example - UPDATE /etc/dvi-bridge/.env with real credentials!"
fi

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
  
  # Install tunnel service (but don't start until registered)
  if [ -f "$RELEASE_DIR/systemd/dvi-tunnel.service" ]; then
    sudo cp "$RELEASE_DIR/systemd/dvi-tunnel.service" "/etc/systemd/system/dvi-tunnel.service"
    sudo systemctl daemon-reload
    echo "✓ Tunnel service installed (run registration to activate)"
  fi
fi

echo ""
echo "== Installation complete =="
echo "Version: $VERSION"
echo "Mode: $MODE"
echo "Location: $RELEASE_DIR"

if [ "$INSTALL_SIDECAR" = true ]; then
  echo ""
  echo "Web interface available at: http://$(hostname -I | awk '{print $1}'):5000"
  echo ""
  echo "== Device Registration (for remote access) =="
  echo "To enable remote access via Cloudflare tunnel:"
  echo "  1. Update credentials: sudo nano /etc/dvi-bridge/.env"
  echo "  2. Register device: sudo python3 $RELEASE_DIR/sidecar/registration.py"
  echo "  3. Enable tunnel: sudo systemctl enable dvi-tunnel"
  echo "  4. Start tunnel: sudo systemctl start dvi-tunnel"
  echo ""
  echo "See DEVICE_REGISTRATION.md for detailed instructions"
fi
```
