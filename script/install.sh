#!/bin/bash

# Simple and verbose installer
set -e

log() {
    echo "$@"
}

log "=========================================="
log "DVI Bridge Standalone Installer"
log "=========================================="
log ""
log "Script starting at $(date)"
log ""

REPO="ruteclrp/dvi-bridge-standalone"

# Function to get available versions from GitHub
get_available_versions() {
  log "Fetching available versions from GitHub..." >&2
  
  # Fetch directory listing from GitHub API
  local api_url="https://api.github.com/repos/$REPO/contents/bridge_assets"
  local versions=$(curl -s "$api_url" | grep '"name"' | sed 's/.*"name": "\(.*\)".*/\1/' | grep '^v' | sort -V)
  
  if [ -z "$versions" ]; then
    log "  ⚠ API failed, trying alternative method..." >&2
    # Try to fetch README.md from bridge_assets which lists versions
    versions=$(curl -s "https://raw.githubusercontent.com/$REPO/main/bridge_assets/README.md" | grep -o 'v[0-9]\+\.[0-9]\+' | sort -V | uniq)
  fi
  
  if [ -z "$versions" ]; then
    log "  ⚠ Alternative method failed, checking local directories..." >&2
    # Fallback to local directory if script is run from repo
    if [ -d "../bridge_assets" ]; then
      versions=$(ls -1 ../bridge_assets | grep '^v' | sort -V)
    fi
  fi
  
  if [ -z "$versions" ]; then
    log "  ⚠ Using hardcoded version list..." >&2
    # Last resort: hardcoded list of known versions
    versions="v6.01 v6.02 v6.03 v6.04 v6.10 v6.20 v6.21 v6.22 v6.23"
  fi
  
  log "  ✓ Found $(echo "$versions" | wc -w) version(s), showing last 10" >&2
  log "" >&2
  
  # Get last 10 versions - only this goes to stdout
  echo "$versions" | tr ' ' '\n' | tail -n 10 | tr '\n' ' '
}

# Function to select version interactively
select_version() {
  local versions=($(get_available_versions))
  
  # Check if any versions were found
  if [ ${#versions[@]} -eq 0 ]; then
    log "ERROR: No versions found. Cannot continue." >&2
    log "" >&2
    log "Please specify a version manually: ./install.sh v6.23" >&2
    exit 1
  fi
  
  local latest="${versions[-1]}"
  
  log "Available versions:" >&2
  for i in "${!versions[@]}"; do
    printf "  %d) %s" "$((i+1))" "${versions[$i]}" >&2
    if [ "${versions[$i]}" = "$latest" ]; then
      printf " (latest)" >&2
    fi
    printf "\n" >&2
  done
  log "" >&2
  
  while true; do
    read -p "Select version number (or press Enter for latest): " choice >&2
    
    if [ -z "$choice" ]; then
      echo "$latest"
      return 0
    fi
    
    if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#versions[@]}" ]; then
      echo "${versions[$((choice-1))]}"
      return 0
    else
      log "  ⚠ Invalid selection. Please enter a number between 1 and ${#versions[@]}" >&2
    fi
  done
}

# Function to select installation mode interactively
select_mode() {
  log "" >&2
  log "Installation modes:" >&2
  log "  1) basic     - Bridge only (MQTT communication)" >&2
  log "  2) sidecar   - Bridge + Web interface" >&2
  log "  3) complete  - All components (legacy)" >&2
  log "" >&2
  
  while true; do
    read -p "Select installation mode (or press Enter for basic): " choice >&2
    
    case "$choice" in
      ""|1)
        echo "basic"
        return 0
        ;;
      2)
        echo "sidecar"
        return 0
        ;;
      3)
        echo "complete"
        return 0
        ;;
      *)
        log "  ⚠ Invalid selection. Please enter 1, 2, or 3" >&2
        ;;
    esac
  done
}

# Check if running in interactive or non-interactive mode
if [ -z "$1" ]; then
  # Interactive mode
  log "Running in interactive mode..."
  log ""
  VERSION=$(select_version)
  MODE=$(select_mode)
  
  log ""
  log "You have selected:"
  log "  Version: $VERSION"
  log "  Mode: $MODE"
  log ""
  
  read -p "Proceed with installation? (y/n): " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log "Installation cancelled by user"
    exit 0
  fi
  log ""
else
  # Non-interactive mode with command-line arguments
  VERSION="$1"
  MODE="${2:-basic}"
  
  # Validate mode
  if [[ ! "$MODE" =~ ^(basic|sidecar|complete)$ ]]; then
    log "ERROR: Invalid mode '$MODE'"
    log ""
    log "Usage: ./install.sh [vX.Y.Z] [mode]"
    log "  mode: basic (default), sidecar, complete"
    log ""
    log "Examples:"
    log "  ./install.sh              - Interactive mode"
    log "  ./install.sh v6.21        - Install v6.21 in basic mode"
    log "  ./install.sh v6.21 basic     - Install bridge only"
    log "  ./install.sh v6.21 sidecar   - Install bridge + sidecar web interface"
    log "  ./install.sh v6.21 complete  - Install all components (legacy)"
    exit 1
  fi
fi

log "Installation parameters:"
log "  Version: $VERSION"
log "  Mode: $MODE"
log "  Repository: $REPO"
log ""

BASE="/home/dviha/dvi-bridge"

log "[1/7] Creating directory structure..."
mkdir -p "$BASE"
log "  ✓ Created $BASE"
cd /tmp
log "  ✓ Working directory: /tmp"
log ""

case "$MODE" in
  basic)
    log "[2/7] Downloading Bridge Basic (MQTT only)..."
    TARBALL="dvi-bridge-basic-$VERSION.rpi.tar.gz"
    URL="https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$TARBALL"
    log "  URL: $URL"
    wget -v "$URL" -O "$TARBALL"
    log "  ✓ Downloaded $TARBALL"
    log ""
    log "[3/7] Extracting files..."
    tar xzf "$TARBALL" -C "$BASE" --strip-components=1
    log "  ✓ Extracted to $BASE"
    log ""
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=false
    ;;
  
  sidecar)
    log "[2/7] Downloading Bridge + Sidecar packages..."
    
    BRIDGE_TAR="dvi-bridge-basic-$VERSION.rpi.tar.gz"
    log "  Downloading bridge..."
    wget -v "https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$BRIDGE_TAR" -O "$BRIDGE_TAR"
    log "  ✓ Downloaded bridge package"
    
    SIDECAR_TAR="dvi-sidecar-$VERSION.rpi.tar.gz"
    log "  Downloading sidecar..."
    wget -v "https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$SIDECAR_TAR" -O "$SIDECAR_TAR"
    log "  ✓ Downloaded sidecar package"
    log ""
    log "[3/7] Extracting files..."
    log "  Extracting bridge package..."
    tar xzf "$BRIDGE_TAR" -C "$BASE" --strip-components=1
    log "  ✓ Extracted bridge"
    log "  Extracting sidecar package..."
    tar xzf "$SIDECAR_TAR" -C "$BASE" --strip-components=1
    log "  ✓ Extracted sidecar"
    log "  ✓ Extracted all packages to $BASE"
    log ""
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=true
    ;;
  
  complete)
    log "[2/7] Downloading Complete Runtime package..."
    TARBALL="dvi-complete-runtime-$VERSION.rpi.tar.gz"
    URL="https://raw.githubusercontent.com/$REPO/main/bridge_assets/$VERSION/$TARBALL"
    log "  URL: $URL"
    wget -v "$URL" -O "$TARBALL"
    log "  ✓ Downloaded $TARBALL"
    log ""
    log "[3/7] Extracting files..."
    tar xzf "$TARBALL" -C "$BASE" --strip-components=1
    log "  ✓ Extracted to $BASE"
    log ""
    INSTALL_BRIDGE=true
    INSTALL_SIDECAR=true
    ;;
  
  *)
    log "ERROR: Invalid mode '$MODE'. Use: basic, sidecar, or complete"
    exit 1
    ;;
esac

log "[4/7] Setting up Python virtual environment..."
if [ ! -d "$BASE/venv" ]; then
  log "  Creating new virtual environment..."
  python3 -m venv "$BASE/venv"
  log "  ✓ Created virtual environment"
else
  log "  ✓ Virtual environment already exists"
fi

source "$BASE/venv/bin/activate"
log "  Upgrading pip..."
pip install --upgrade pip
log "  ✓ Pip upgraded"
log "  Installing Python dependencies..."
pip install -r "$BASE/requirements.txt"
log "  ✓ Dependencies installed"
log ""

log "[5/7] Creating configuration file..."
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
  log "  ✓ Created $BASE/.env"
  log ""
  log "⚠ MQTT Configuration Required!"
  log "The bridge needs MQTT broker settings to work."
  log ""
  read -p "Do you want to edit .env now? (y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    ${EDITOR:-nano} "$BASE/.env"
    log "  ✓ Configuration updated"
  else
    log "  ⚠ Remember to edit $BASE/.env before starting the service"
    log "    Edit with: nano $BASE/.env"
    log "    Then restart: sudo systemctl restart bridge.service"
  fi
else
  log "  ✓ Using existing .env file"
fi
log ""

log "[6/7] Saving version information..."
echo "$VERSION" > "$BASE/VERSION"
log "  ✓ Installed version: $VERSION"
log ""

log "[7/7] Installing and starting systemd services..."
if [ "$INSTALL_BRIDGE" = true ]; then
  if [ -f "$BASE/systemd/bridge.service.example" ]; then
    log "  Installing bridge service..."
    # Create service file with proper paths - replace ALL occurrences
    sed -e "s|/home/<user>/dvi-bridge-standalone|$BASE|g" \
        -e "s|/.venv/|/venv/|g" \
        "$BASE/systemd/bridge.service.example" | sudo tee /etc/systemd/system/bridge.service > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable bridge.service
    log "  Attempting to start bridge service..."
    if sudo systemctl start bridge.service; then
      log "  ✓ Bridge service installed and started"
    else
      log "  ⚠ Bridge service installed but failed to start"
      log "  Check status with: sudo systemctl status bridge.service"
      log "  Check logs with: sudo journalctl -u bridge.service -n 50"
    fi
  else
    log "  ⚠ Bridge service file not found"
  fi
fi

if [ "$INSTALL_SIDECAR" = true ]; then
  if [ -f "$BASE/systemd/webbridge.service.example" ]; then
    log "  Installing webbridge service..."
    # Create service file with proper paths - replace ALL occurrences
    sed -e "s|/home/<user>/dvi-bridge-standalone|$BASE|g" \
        -e "s|/.venv/|/venv/|g" \
        "$BASE/systemd/webbridge.service.example" | sudo tee /etc/systemd/system/webbridge.service > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable webbridge.service
    log "  Attempting to start webbridge service..."
    if sudo systemctl start webbridge.service; then
      log "  ✓ Webbridge service installed and started"
    else
      log "  ⚠ Webbridge service installed but failed to start"
      log "  Check status with: sudo systemctl status webbridge.service"
      log "  Check logs with: sudo journalctl -u webbridge.service -n 50"
    fi
  else
    log "  ⚠ Webbridge service file not found"
  fi
fi

log ""
log "=========================================="
log "✓ Installation Complete!"
log "=========================================="
log "Version:  $VERSION"
log "Mode:     $MODE"
log "Location: $BASE"
log ""

if [ "$INSTALL_BRIDGE" = true ]; then
  log "Bridge Status:"
  sudo systemctl status bridge.service --no-pager -l | head -n 5 || true
  log ""
fi

if [ "$INSTALL_SIDECAR" = true ]; then
  log "Web Interface:"
  log "  URL: http://$(hostname -I | awk '{print $1}'):5555"
  log ""
  log "Webbridge Status:"
  sudo systemctl status webbridge.service --no-pager -l | head -n 5 || true
  log ""
fi

log "Next steps:"
if sudo systemctl is-active --quiet bridge.service 2>/dev/null; then
  log "  ✓ Bridge is running"
else
  log "  ⚠ Bridge needs attention - check: sudo journalctl -u bridge.service -n 50"
fi
log "  1. Edit MQTT settings: nano $BASE/.env"
log "  2. Check logs: sudo journalctl -u bridge.service -f"
if [ "$INSTALL_SIDECAR" = true ]; then
  log "  3. Check web logs: sudo journalctl -u webbridge.service -f"
fi
log ""
log "Installation finished at $(date)"
