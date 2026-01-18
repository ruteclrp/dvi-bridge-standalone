```bash
#!/usr/bin/env bash
set -e

REPO="ruteclrp/dvi-bridge-standalone"
BASE="/home/dviha/dvi-bridge"
CURRENT_VERSION=$(cat "$BASE/current/VERSION" 2>/dev/null || echo "none")

# Detect what's currently installed
HAS_BRIDGE=false
HAS_SIDECAR=false

if systemctl is-enabled bridge.service &>/dev/null; then
  HAS_BRIDGE=true
fi

if systemctl is-enabled webbridge.service &>/dev/null; then
  HAS_SIDECAR=true
fi

if [ "$HAS_BRIDGE" = false ]; then
  echo "Error: No DVI bridge installation detected."
  echo "Please run install.sh first."
  exit 1
fi

echo "== Current installation =="
echo "Bridge: $HAS_BRIDGE"
echo "Sidecar: $HAS_SIDECAR"
echo "Current version: $CURRENT_VERSION"
echo ""

echo "== Checking latest release =="
LATEST=$(curl -s https://api.github.com/repos/$REPO/releases/latest | grep tag_name | cut -d '"' -f 4)

echo "Latest version: $LATEST"

if [ "$CURRENT_VERSION" = "$LATEST" ]; then
  echo "Already up to date"
  exit 0
fi

echo ""
echo "== Updating to $LATEST =="
RELEASE_DIR="$BASE/releases/$LATEST"
mkdir -p "$RELEASE_DIR"
cd /tmp

# Update bridge if installed
if [ "$HAS_BRIDGE" = true ]; then
  echo "Downloading bridge update..."
  BRIDGE_TAR="dvi-bridge-basic-$LATEST.tar.gz"
  URL="https://github.com/$REPO/releases/download/$LATEST/$BRIDGE_TAR"
  wget -q "$URL" -O "$BRIDGE_TAR"
  tar xzf "$BRIDGE_TAR" -C "$RELEASE_DIR" --strip-components=1
  echo "✓ Bridge updated"
fi

# Update sidecar if installed
if [ "$HAS_SIDECAR" = true ]; then
  echo "Downloading sidecar update..."
  SIDECAR_TAR="dvi-sidecar-$LATEST.tar.gz"
  URL="https://github.com/$REPO/releases/download/$LATEST/$SIDECAR_TAR"
  wget -q "$URL" -O "$SIDECAR_TAR"
  tar xzf "$SIDECAR_TAR" -C "$RELEASE_DIR" --strip-components=1
  echo "✓ Sidecar updated"
fi

echo ""
echo "== Updating Python dependencies =="
source "$BASE/venv/bin/activate"
pip install --upgrade pip -q

# Install dependencies from available requirements.txt
if [ -f "$RELEASE_DIR/requirements.txt" ]; then
  pip install -r "$RELEASE_DIR/requirements.txt" -q
elif [ -f "$RELEASE_DIR/sidecar/requirements.txt" ]; then
  pip install -r "$RELEASE_DIR/sidecar/requirements.txt" -q
fi

echo "== Updating current symlink =="
ln -sfn "$RELEASE_DIR" "$BASE/current"

echo ""
echo "== Restarting services =="
if [ "$HAS_BRIDGE" = true ]; then
  sudo systemctl restart bridge.service
  echo "✓ Bridge service restarted"
fi

if [ "$HAS_SIDECAR" = true ]; then
  sudo systemctl restart webbridge.service
  echo "✓ Sidecar service restarted"
fi

echo ""
echo "== Update complete =="
echo "Updated from $CURRENT_VERSION to $LATEST"
```
