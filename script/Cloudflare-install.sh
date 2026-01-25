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
echo "Next steps:"
echo "  - Scan QR code with mobile app (if shown above)"
echo "  - Or use auto-discovery on local WiFi"
echo "  - Access via: $TUNNEL_URL"
echo ""
echo "Note: Tunnel is running in background (PID $TUNNEL_PID)"
echo "      To stop: kill $TUNNEL_PID"
echo "========================================="