```bash
#!/usr/bin/env bash
set -e

REPO="ruteclrp/dvi-bridge-standalone"
BASE="/home/dviha/dvi-bridge"
CURRENT_VERSION=$(cat "$BASE/current/VERSION" 2>/dev/null || echo "none")

echo "== Checking latest release =="
LATEST=$(curl -s https://api.github.com/repos/$REPO/releases/latest | grep tag_name | cut -d '"' -f 4)

echo "Current version: $CURRENT_VERSION"
echo "Latest version:  $LATEST"

if [ "$CURRENT_VERSION" = "$LATEST" ]; then
  echo "Already up to date"
  exit 0
fi

echo "== Updating to $LATEST =="
URL="https://github.com/$REPO/releases/download/$LATEST/bridge-runtime.tar.gz"
TARBALL="/tmp/bridge-runtime.tar.gz"
RELEASE_DIR="$BASE/releases/$LATEST"

wget -q "$URL" -O "$TARBALL"

mkdir -p "$RELEASE_DIR"
tar xzf "$TARBALL" -C "$RELEASE_DIR" --strip-components=1

echo "== Updating Python dependencies =="
source "$BASE/venv/bin/activate"
pip install -r "$RELEASE_DIR/requirements.txt"

echo "== Updating current symlink =="
ln -sfn "$RELEASE_DIR" "$BASE/current"

echo "== Restarting services =="
sudo systemctl restart bridge.service webbridge.service || true

echo "== Update complete =="
echo "Now running version $LATEST"
```
