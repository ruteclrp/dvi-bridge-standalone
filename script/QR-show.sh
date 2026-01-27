#!/bin/bash
# QR-show.sh - Regenerate QR code for Cloudflare tunnel URL

set -e  # Exit on any error

echo "========================================="
echo "DVI Bridge - Show QR Code"
echo "========================================="

# Check if config file exists
CONFIG_FILE="/etc/dvi-bridge/tunnel.conf"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Configuration file not found: $CONFIG_FILE"
    echo "   Please run Cloudflare-install.sh first"
    exit 1
fi

# Read tunnel URL from config
echo ""
echo "Reading tunnel configuration..."
source "$CONFIG_FILE"

if [ -z "$TUNNEL_URL" ]; then
    echo "❌ No tunnel URL found in configuration"
    exit 1
fi

echo "✅ Tunnel URL: $TUNNEL_URL"

# Check if qrencode is available
if ! command -v qrencode &> /dev/null; then
    echo "❌ qrencode not installed"
    echo "   Install with: sudo apt-get install qrencode"
    echo ""
    echo "Alternative: Visit this URL to see QR code:"
    echo "https://api.qrserver.com/v1/create-qr-code/?size=300x300&data=$TUNNEL_URL"
    exit 1
fi

# Generate QR code
echo ""
echo "=== QR CODE START ==="
if qrencode -t ANSIUTF8 "$TUNNEL_URL" 2>&1; then
    echo "=== QR CODE END ==="
    echo "✅ QR code generated above"
else
    echo "=== QR CODE END ==="
    echo "❌ QR code generation failed with ANSIUTF8 format"
    echo "Trying alternative ASCII format..."
    qrencode -t ASCII "$TUNNEL_URL" 2>&1 || echo "Alternative format also failed"
fi

echo ""
echo "Alternative: Visit this URL to see QR code:"
echo "https://api.qrserver.com/v1/create-qr-code/?size=300x300&data=$TUNNEL_URL"
echo ""
echo "========================================="
