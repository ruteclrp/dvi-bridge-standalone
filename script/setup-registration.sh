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
PIP_CMD=""
if [ -x "/home/dviha/dvi-bridge/venv/bin/pip" ]; then
    PIP_CMD="/home/dviha/dvi-bridge/venv/bin/pip"
elif [ -x "$PROJECT_ROOT/.venv/bin/pip" ]; then
    PIP_CMD="$PROJECT_ROOT/.venv/bin/pip"
elif command -v pip3 &> /dev/null; then
    PIP_CMD="pip3"
fi

REQ_FILE=""
if [ -f "/home/dviha/dvi-bridge/requirements.txt" ]; then
    REQ_FILE="/home/dviha/dvi-bridge/requirements.txt"
elif [ -f "$PROJECT_ROOT/src/bridge/requirements.txt" ]; then
    REQ_FILE="$PROJECT_ROOT/src/bridge/requirements.txt"
fi

if [ -z "$REQ_FILE" ]; then
    echo "  ✗ requirements.txt not found. Expected /home/dviha/dvi-bridge/requirements.txt"
    exit 1
fi

if [ -z "$PIP_CMD" ]; then
    echo "  ✗ pip not found. Install python3-pip or create a venv at $PROJECT_ROOT/.venv"
    exit 1
fi

$PIP_CMD install -r "$REQ_FILE"
echo "  ✓ Python dependencies installed"

echo ""
echo "[3/6] Creating configuration directory..."
mkdir -p /etc/dvi-bridge
mkdir -p /etc/cloudflared
echo "  ✓ Directories created"

echo ""
echo "[4/6] Setting up environment configuration..."
if [ ! -f /etc/dvi-bridge/.env ]; then
    if [ -f "/home/dviha/dvi-bridge/.env" ]; then
        cp "/home/dviha/dvi-bridge/.env" /etc/dvi-bridge/.env
        echo "  ✓ Using existing .env file from /home/dviha/dvi-bridge/.env"
    elif [ -f "$PROJECT_ROOT/src/sidecar/.env" ]; then
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
cp "$PROJECT_ROOT/dvi-bridge/systemd/dvi-tunnel.service" /etc/systemd/system/
systemctl daemon-reload
echo "  ✓ Systemd service installed"

echo ""
echo "[6/6] Installing heartbeat cron job..."
CRON_FILE="/etc/cron.d/dvi-heartbeat"
HEARTBEAT_SCRIPT="$PROJECT_ROOT/dvi-bridge/sidecar/heartbeat.py"
PYTHON_CMD="/usr/bin/python3"
if [ -x "/home/dviha/dvi-bridge/venv/bin/python" ]; then
    PYTHON_CMD="/home/dviha/dvi-bridge/venv/bin/python"
elif [ -x "$PROJECT_ROOT/.venv/bin/python" ]; then
    PYTHON_CMD="$PROJECT_ROOT/.venv/bin/python"
fi
cat > "$CRON_FILE" <<EOF
*/2 * * * * root "$PYTHON_CMD" "$HEARTBEAT_SCRIPT" >> /var/log/dvi-heartbeat.log 2>&1
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
echo "1. Update the backend URL in /home/dviha/dvi-bridge/.env"
echo "   sudo nano /home/dviha/dvi-bridge/.env"
echo ""
echo "2. Register this device with the backend:"
echo "   sudo /home/dviha/dvi-bridge/venv/bin/python /home/dviha/dvi-bridge/sidecar/registration.py"
echo ""
echo "3. Enable tunnel to start on boot:"
echo "   sudo systemctl enable dvi-tunnel"
echo "   sudo systemctl start dvi-tunnel"
echo ""
echo "4. Check tunnel status:"
echo "   sudo systemctl status dvi-tunnel"
echo ""
