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

RUNTIME_ROOT="/home/dviha/dvi-bridge"
RUNTIME_CURRENT="$RUNTIME_ROOT/current"

find_dir_with_files() {
    local dir
    for dir in "$@"; do
        [ -n "$dir" ] || continue
        [ -d "$dir" ] || continue

        case "$dir" in
            *systemd)
                [ -f "$dir/dvi-tunnel.service" ] || continue
                [ -f "$dir/device-channel.service" ] || continue
                [ -f "$dir/dvi-tunnels-ready.service" ] || continue
                [ -f "$dir/dvi-heartbeat.service" ] || continue
                [ -f "$dir/dvi-heartbeat.timer" ] || continue
                [ -f "$dir/dvi-tunnel-watchdog.service" ] || continue
                [ -f "$dir/dvi-tunnel-watchdog.timer" ] || continue
                ;;
            *sidecar)
                [ -f "$dir/.env" ] || [ -f "$dir/.env.example" ] || continue
                ;;
        esac

        echo "$dir"
        return 0
    done

    return 1
}

REPO_SIDECAR_DIR="$(find_dir_with_files \
    "$RUNTIME_CURRENT/sidecar" \
    "$RUNTIME_ROOT/sidecar" \
    "$PROJECT_ROOT/src/sidecar" \
    "$PROJECT_ROOT/sidecar")" || REPO_SIDECAR_DIR=""

REPO_SYSTEMD_DIR="$(find_dir_with_files \
    "$RUNTIME_CURRENT/systemd" \
    "$RUNTIME_ROOT/systemd" \
    "$PROJECT_ROOT/src/systemd" \
    "$PROJECT_ROOT/systemd")" || REPO_SYSTEMD_DIR=""

if [ -f "$RUNTIME_CURRENT/requirements.txt" ]; then
    REPO_REQUIREMENTS_FILE="$RUNTIME_CURRENT/requirements.txt"
elif [ -f "$RUNTIME_ROOT/requirements.txt" ]; then
    REPO_REQUIREMENTS_FILE="$RUNTIME_ROOT/requirements.txt"
elif [ -f "$PROJECT_ROOT/src/bridge/requirements.txt" ]; then
    REPO_REQUIREMENTS_FILE="$PROJECT_ROOT/src/bridge/requirements.txt"
elif [ -f "$PROJECT_ROOT/requirements.txt" ]; then
    REPO_REQUIREMENTS_FILE="$PROJECT_ROOT/requirements.txt"
else
    REPO_REQUIREMENTS_FILE=""
fi

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
if [ -x "$RUNTIME_ROOT/venv/bin/pip" ]; then
    PIP_CMD="$RUNTIME_ROOT/venv/bin/pip"
elif [ -x "$PROJECT_ROOT/.venv/bin/pip" ]; then
    PIP_CMD="$PROJECT_ROOT/.venv/bin/pip"
elif command -v pip3 &> /dev/null; then
    PIP_CMD="pip3"
fi

REQ_FILE=""
if [ -n "$REPO_REQUIREMENTS_FILE" ]; then
    REQ_FILE="$REPO_REQUIREMENTS_FILE"
fi

if [ -z "$REQ_FILE" ]; then
    echo "  ✗ requirements.txt not found. Expected $RUNTIME_CURRENT/requirements.txt, $RUNTIME_ROOT/requirements.txt, or $PROJECT_ROOT/src/bridge/requirements.txt"
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
echo "  ✓ Directories created"

echo ""
echo "[4/6] Setting up environment configuration..."
if [ ! -f /etc/dvi-bridge/.env ]; then
    if [ -f "$RUNTIME_ROOT/.env" ]; then
        cp "$RUNTIME_ROOT/.env" /etc/dvi-bridge/.env
        echo "  ✓ Using existing .env file from $RUNTIME_ROOT/.env"
    elif [ -n "$REPO_SIDECAR_DIR" ] && [ -f "$REPO_SIDECAR_DIR/.env" ]; then
        cp "$REPO_SIDECAR_DIR/.env" /etc/dvi-bridge/.env
        echo "  ✓ Using existing .env file"
    elif [ -n "$REPO_SIDECAR_DIR" ] && [ -f "$REPO_SIDECAR_DIR/.env.example" ]; then
        cp "$REPO_SIDECAR_DIR/.env.example" /etc/dvi-bridge/.env
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
echo "[5/6] Installing systemd units..."
SYSTEMD_SRC="$REPO_SYSTEMD_DIR"
if [ -z "$SYSTEMD_SRC" ]; then
    echo "  ✗ systemd unit directory not found. Expected $RUNTIME_CURRENT/systemd, $RUNTIME_ROOT/systemd, $PROJECT_ROOT/src/systemd, or $PROJECT_ROOT/systemd"
    exit 1
fi
cp "$SYSTEMD_SRC/dvi-tunnel.service" /etc/systemd/system/
cp "$SYSTEMD_SRC/device-channel.service" /etc/systemd/system/
cp "$SYSTEMD_SRC/dvi-tunnels-ready.service" /etc/systemd/system/
cp "$SYSTEMD_SRC/dvi-heartbeat.service" /etc/systemd/system/
cp "$SYSTEMD_SRC/dvi-heartbeat.timer" /etc/systemd/system/
cp "$SYSTEMD_SRC/dvi-tunnel-watchdog.service" /etc/systemd/system/
cp "$SYSTEMD_SRC/dvi-tunnel-watchdog.timer" /etc/systemd/system/
systemctl daemon-reload
echo "  ✓ Systemd units installed"

echo ""
echo "[6/6] Removing legacy heartbeat cron (if present)..."
CRON_FILE="/etc/cron.d/dvi-heartbeat"
if [ -f "$CRON_FILE" ]; then
    rm -f "$CRON_FILE"
    echo "  ✓ Removed legacy cron job (${CRON_FILE})"
else
    echo "  ✓ No legacy cron job found"
fi

echo ""
echo "=========================================="
echo "✓ Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Update backend and Access settings in /etc/dvi-bridge/.env"
echo "   sudo nano /etc/dvi-bridge/.env"
echo ""
echo "2. Register this device with the backend:"
echo "   sudo /home/dviha/dvi-bridge/venv/bin/python /home/dviha/dvi-bridge/sidecar/registration.py"
echo ""
echo "3. Enable startup sequence (tunnels + readiness gate + timers):"
echo "   sudo systemctl enable dvi-tunnel device-channel dvi-tunnels-ready"
echo "   sudo systemctl enable dvi-heartbeat.timer dvi-tunnel-watchdog.timer"
echo "   sudo systemctl start dvi-tunnel device-channel"
echo "   sudo systemctl start dvi-tunnels-ready"
echo "   sudo systemctl start dvi-heartbeat.timer dvi-tunnel-watchdog.timer"
echo ""
echo "4. Check service status:"
echo "   sudo systemctl status dvi-tunnel device-channel dvi-tunnels-ready"
echo "   sudo systemctl status dvi-heartbeat.timer dvi-tunnel-watchdog.timer"
echo ""
echo "5. Start or restart your bridge and webbridge services"
echo ""
