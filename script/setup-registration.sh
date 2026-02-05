#!/bin/bash
# DVI Bridge - Device Registration Setup Script
# This script installs and configures the device registration for RPi

set -e

echo "=========================================="
echo "DVI Bridge - Registration Setup"
echo "=========================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo ""
echo "[1/6] Installing cloudflared..."
if ! command -v cloudflared &> /dev/null; then
    # Detect architecture
    ARCH=$(uname -m)
    if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
        CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64"
    elif [ "$ARCH" = "armv7l" ] || [ "$ARCH" = "armv6l" ]; then
        CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm"
    else
        CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64"
    fi
    
    echo "  Downloading cloudflared for $ARCH..."
    wget -q "$CLOUDFLARED_URL" -O /tmp/cloudflared
    chmod +x /tmp/cloudflared
    mv /tmp/cloudflared /usr/local/bin/cloudflared
    echo "  ✓ cloudflared installed"
else
    echo "  ✓ cloudflared already installed"
fi

echo ""
echo "[2/6] Installing Python dependencies..."
pip3 install -r "$PROJECT_ROOT/src/bridge/requirements.txt"
echo "  ✓ Python dependencies installed"

echo ""
echo "[3/6] Creating configuration directory..."
mkdir -p /etc/dvi-bridge
mkdir -p /etc/cloudflared
echo "  ✓ Directories created"

echo ""
echo "[4/6] Setting up environment configuration..."
if [ ! -f /etc/dvi-bridge/.env ]; then
    if [ -f "$PROJECT_ROOT/src/sidecar/.env" ]; then
        cp "$PROJECT_ROOT/src/sidecar/.env" /etc/dvi-bridge/.env
        echo "  ✓ Using existing .env file"
    elif [ -f "$PROJECT_ROOT/src/sidecar/.env.example" ]; then
        cp "$PROJECT_ROOT/src/sidecar/.env.example" /etc/dvi-bridge/.env
        echo "  ⚠️  Created .env from example - UPDATE MAKER_BACKEND_URL!"
        echo ""
        echo "  Edit /etc/dvi-bridge/.env and set:"
        echo "    MAKER_BACKEND_URL=https://your-actual-backend-url.com"
    else
        echo "  ✗ No .env file found. Please create /etc/dvi-bridge/.env"
        exit 1
    fi
else
    echo "  ✓ Configuration already exists"
fi

echo ""
echo "[5/6] Installing systemd service..."
cp "$PROJECT_ROOT/src/systemd/dvi-tunnel.service" /etc/systemd/system/
systemctl daemon-reload
echo "  ✓ Systemd service installed"

echo ""
echo "[6/6] Installing heartbeat cron job..."
CRON_FILE="/etc/cron.d/dvi-heartbeat"
HEARTBEAT_SCRIPT="$PROJECT_ROOT/src/sidecar/heartbeat.py"
cat > "$CRON_FILE" <<EOF
*/2 * * * * root /usr/bin/python3 "$HEARTBEAT_SCRIPT" >> /var/log/dvi-heartbeat.log 2>&1
EOF
chmod 644 "$CRON_FILE"
echo "  ✓ Cron job installed (${CRON_FILE})"

echo ""
echo "=========================================="
echo "✓ Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Update the backend URL in /etc/dvi-bridge/.env"
echo "   sudo nano /etc/dvi-bridge/.env"
echo ""
echo "2. Register this device with the backend:"
echo "   sudo python3 $PROJECT_ROOT/src/sidecar/registration.py"
echo ""
echo "3. Enable tunnel to start on boot:"
echo "   sudo systemctl enable dvi-tunnel"
echo "   sudo systemctl start dvi-tunnel"
echo ""
echo "4. Check tunnel status:"
echo "   sudo systemctl status dvi-tunnel"
echo ""
