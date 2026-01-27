#!/bin/bash
# setup-dvi-bridge.sh

set -e  # Exit on any error

echo "========================================="
echo "Setting up DVI Heatpump Bridge..."
echo "========================================="

# Install cloudflared
echo ""
echo "Step 1: Downloading cloudflared..."
if curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 -o cloudflared; then
    echo "✅ Download successful"
else
    echo "❌ Failed to download cloudflared"
    exit 1
fi

echo ""
echo "Step 2: Installing cloudflared to /usr/local/bin/..."
if sudo install -m 755 cloudflared /usr/local/bin/; then
    echo "✅ Installation successful"
    rm -f cloudflared  # Clean up downloaded file
else
    echo "❌ Failed to install cloudflared"
    exit 1
fi

# Verify installation
if command -v cloudflared &> /dev/null; then
    echo "✅ cloudflared is now available: $(which cloudflared)"
    cloudflared --version
else
    echo "❌ cloudflared installation verification failed"
    exit 1
fi

# Install qrencode
echo ""
echo "Step 3: Installing qrencode for QR code generation..."
if command -v qrencode &> /dev/null; then
    echo "✅ qrencode already installed: $(which qrencode)"
else
    echo "⏳ Installing qrencode..."
    if sudo apt-get update && sudo apt-get install -y qrencode; then
        echo "✅ qrencode installed successfully"
    else
        echo "⚠️  Failed to install qrencode (QR code will be skipped)"
        echo "   You can install it later with: sudo apt-get install qrencode"
    fi
fi

# Create config directory if needed
echo ""
echo "Step 4: Creating config directory..."
if sudo mkdir -p /etc/dvi-bridge; then
    echo "✅ Config directory ready"
else
    echo "❌ Failed to create config directory"
    exit 1
fi

# Create quick tunnel (no login required!) and capture URL
echo ""
echo "Step 5: Creating Cloudflare tunnel..."
echo "⏳ This may take a moment, waiting for tunnel URL..."
echo "(Starting tunnel in background and capturing URL)"

# Start cloudflared in background and capture output
TUNNEL_LOG=$(mktemp)
cloudflared tunnel --url http://localhost:5000 > "$TUNNEL_LOG" 2>&1 &
TUNNEL_PID=$!

echo "Tunnel process started (PID: $TUNNEL_PID)"

# Wait for URL to appear (timeout after 30 seconds)
TIMEOUT=30
COUNTER=0
TUNNEL_URL=""

while [ $COUNTER -lt $TIMEOUT ]; do
    TUNNEL_URL=$(grep -oP 'https://[^/]+\.trycloudflare\.com' "$TUNNEL_LOG" | head -1)
    if [ -n "$TUNNEL_URL" ]; then
        break
    fi
    sleep 1
    COUNTER=$((COUNTER + 1))
    echo -n "."
done
echo ""

if [ -z "$TUNNEL_URL" ]; then
    echo "❌ Failed to get tunnel URL after ${TIMEOUT}s"
    echo "Tunnel log contents:"
    cat "$TUNNEL_LOG"
    kill $TUNNEL_PID 2>/dev/null || true
    rm -f "$TUNNEL_LOG"
    exit 1
fi

echo "✅ Tunnel URL obtained: $TUNNEL_URL"

# Save tunnel URL to config
echo ""
echo "Step 6: Saving tunnel configuration..."
if echo "TUNNEL_URL=$TUNNEL_URL" | sudo tee /etc/dvi-bridge/tunnel.conf > /dev/null; then
    echo "✅ Configuration saved to /etc/dvi-bridge/tunnel.conf"
else
    echo "❌ Failed to save configuration"
    exit 1
fi

# Broadcast via mDNS with tunnel URL
echo ""
echo "Step 7: Setting up mDNS broadcast..."
if sudo mkdir -p /etc/avahi/services; then
    echo "✅ Avahi services directory ready"
else
    echo "⚠️  Warning: Could not create Avahi directory (may not be installed)"
fi

if sudo tee /etc/avahi/services/dvi-bridge.service > /dev/null <<EOF
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name>DVI Bridge</name>
  <service>
    <type>_dvi-bridge._tcp</type>
    <port>5000</port>
    <txt-record>tunnel=$TUNNEL_URL</txt-record>
  </service>
</service-group>
EOF
then
    echo "✅ mDNS service configured"
else
    echo "⚠️  Warning: Could not configure mDNS (Avahi may not be installed)"
fi

# Show QR code for easy mobile pairing
echo ""
echo "Step 8: Generating QR code..."
if command -v qrencode &> /dev/null; then
    echo "qrencode found at: $(which qrencode)"
    echo "Generating QR code for URL: $TUNNEL_URL"
    echo ""
    echo "=== QR CODE START ==="
    if qrencode -t ANSIUTF8 "$TUNNEL_URL" 2>&1; then
        echo "=== QR CODE END ==="
        echo "✅ QR code generated above"
    else
        echo "=== QR CODE END ==="
        echo "❌ QR code generation failed"
        echo "Trying alternative ASCII format..."
        qrencode -t ASCII "$TUNNEL_URL" 2>&1 || echo "Alternative format also failed"
    fi
else
    echo "❌ qrencode not available (installation may have failed)"
fi
echo ""
echo "Alternative: Visit this URL to see QR code:"
echo "https://api.qrserver.com/v1/create-qr-code/?size=300x300&data=$TUNNEL_URL"

# Clean up temp log
rm -f "$TUNNEL_LOG"

echo ""
echo "========================================="
echo "✅ Setup complete!"
echo "========================================="
echo ""
echo "Tunnel URL: $TUNNEL_URL"
echo "Tunnel PID: $TUNNEL_PID"
echo ""

# Now set up systemd services for permanent installation
echo "========================================="
echo "Setting up systemd services..."
echo "========================================="
echo ""

# Stop the temporary tunnel
echo "Stopping temporary tunnel (PID: $TUNNEL_PID)..."
kill $TUNNEL_PID 2>/dev/null || true
sleep 2

# Install monitoring script
echo "Step 9: Installing tunnel monitoring script..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if sudo install -m 755 "$SCRIPT_DIR/monitor-tunnel.sh" /usr/local/bin/; then
    echo "✅ Monitoring script installed to /usr/local/bin/monitor-tunnel.sh"
else
    echo "❌ Failed to install monitoring script"
    exit 1
fi

if sudo install -m 755 "$SCRIPT_DIR/manage-tunnel.sh" /usr/local/bin/; then
    echo "✅ Management script installed to /usr/local/bin/manage-tunnel.sh"
else
    echo "❌ Failed to install management script"
    exit 1
fi

# Create log directory
echo ""
echo "Step 10: Creating log directory..."
if sudo mkdir -p /var/log/dvi-bridge /var/run/dvi-bridge; then
    echo "✅ Directories created"
else
    echo "❌ Failed to create directories"
    exit 1
fi

# Install systemd services
echo ""
echo "Step 11: Installing systemd services..."

# Determine the project root (parent of script directory)
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Copy service files from src/systemd/
CLOUDFLARED_SERVICE_SRC="$PROJECT_ROOT/src/systemd/cloudflared.service.example"
TUNNEL_MONITOR_SERVICE_SRC="$PROJECT_ROOT/src/systemd/tunnel-monitor.service.example"

if [ ! -f "$CLOUDFLARED_SERVICE_SRC" ]; then
    echo "❌ cloudflared.service.example not found at: $CLOUDFLARED_SERVICE_SRC"
    exit 1
fi

if [ ! -f "$TUNNEL_MONITOR_SERVICE_SRC" ]; then
    echo "❌ tunnel-monitor.service.example not found at: $TUNNEL_MONITOR_SERVICE_SRC"
    exit 1
fi

# Install cloudflared service
if sudo cp "$CLOUDFLARED_SERVICE_SRC" /etc/systemd/system/cloudflared.service; then
    echo "✅ cloudflared.service installed"
else
    echo "❌ Failed to install cloudflared.service"
    exit 1
fi

# Install tunnel monitor service
if sudo cp "$TUNNEL_MONITOR_SERVICE_SRC" /etc/systemd/system/tunnel-monitor.service; then
    echo "✅ tunnel-monitor.service installed"
else
    echo "❌ Failed to install tunnel-monitor.service"
    exit 1
fi

# Reload systemd
echo ""
echo "Step 12: Reloading systemd..."
if sudo systemctl daemon-reload; then
    echo "✅ Systemd reloaded"
else
    echo "❌ Failed to reload systemd"
    exit 1
fi

# Enable and start services
echo ""
echo "Step 13: Enabling and starting services..."

if sudo systemctl enable cloudflared.service; then
    echo "✅ cloudflared.service enabled (will start on boot)"
else
    echo "❌ Failed to enable cloudflared.service"
fi

if sudo systemctl enable tunnel-monitor.service; then
    echo "✅ tunnel-monitor.service enabled (will start on boot)"
else
    echo "❌ Failed to enable tunnel-monitor.service"
fi

echo ""
echo "Starting services..."

if sudo systemctl start cloudflared.service; then
    echo "✅ cloudflared.service started"
else
    echo "❌ Failed to start cloudflared.service"
    exit 1
fi

# Wait a moment for cloudflared to start
sleep 3

if sudo systemctl start tunnel-monitor.service; then
    echo "✅ tunnel-monitor.service started"
else
    echo "⚠️  tunnel-monitor.service failed to start (non-critical)"
fi

# Wait for new tunnel URL
echo ""
echo "⏳ Waiting for tunnel to establish (this may take 10-30 seconds)..."
COUNTER=0
TIMEOUT=60
NEW_TUNNEL_URL=""

while [ $COUNTER -lt $TIMEOUT ]; do
    if [ -f "/var/run/dvi-bridge/tunnel_url.txt" ]; then
        NEW_TUNNEL_URL=$(cat /var/run/dvi-bridge/tunnel_url.txt 2>/dev/null || echo "")
        if [ -n "$NEW_TUNNEL_URL" ]; then
            break
        fi
    fi
    sleep 1
    COUNTER=$((COUNTER + 1))
    echo -n "."
done
echo ""

if [ -z "$NEW_TUNNEL_URL" ]; then
    echo "⚠️  Tunnel URL not available yet, but service is running"
    echo "   Check status with: sudo systemctl status cloudflared.service"
    echo "   Or use: sudo /usr/local/bin/manage-tunnel.sh status"
    NEW_TUNNEL_URL="$TUNNEL_URL"  # Use old URL as fallback
else
    echo "✅ New tunnel URL: $NEW_TUNNEL_URL"
fi

echo ""
echo "========================================="
echo "✅ Installation Complete!"
echo "========================================="
echo ""
echo "Tunnel URL: $NEW_TUNNEL_URL"
echo ""
echo "Tunnel is now running as a system service and will:"
echo "  ✅ Start automatically on boot"
echo "  ✅ Restart automatically if it crashes"
echo "  ✅ Update URL automatically when it changes"
echo "  ✅ Expose URL via /api/tunnel endpoint"
echo ""
echo "Management commands:"
echo "  sudo /usr/local/bin/manage-tunnel.sh status   - Show status"
echo "  sudo /usr/local/bin/manage-tunnel.sh url      - Show URL & QR code"
echo "  sudo /usr/local/bin/manage-tunnel.sh restart  - Restart tunnel"
echo "  sudo /usr/local/bin/manage-tunnel.sh logs     - View logs"
echo ""
echo "Service commands:"
echo "  sudo systemctl status cloudflared           - Check service status"
echo "  sudo systemctl restart cloudflared          - Restart service"
echo "  sudo journalctl -u cloudflared -f           - View live logs"
echo ""
echo "Next steps:"
echo "  - Mobile app will auto-discover tunnel URL via /api/tunnel"
echo "  - Scan QR code for remote access setup"
echo "  - Test the API: curl http://localhost:5000/api/tunnel"
echo ""
echo "========================================="