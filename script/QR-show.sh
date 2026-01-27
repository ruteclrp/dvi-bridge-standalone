#!/bin/bash
# QR-show.sh - Show current tunnel URL and QR code
#
# This script displays the current tunnel URL and generates a QR code.
# It reads from the live tunnel_url.txt file or falls back to config.

set -e  # Exit on any error

echo "========================================="
echo "DVI Bridge - Show Tunnel URL & QR Code"
echo "========================================="

# Try to read from live tunnel URL file first (most up-to-date)
TUNNEL_URL_FILE="/var/run/dvi-bridge/tunnel_url.txt"
CONFIG_FILE="/etc/dvi-bridge/tunnel.conf"

TUNNEL_URL=""

# Try runtime file first
if [ -f "$TUNNEL_URL_FILE" ]; then
    echo "Reading tunnel URL from runtime file..."
    TUNNEL_URL=$(cat "$TUNNEL_URL_FILE" 2>/dev/null || echo "")
fi

# Fall back to config file
if [ -z "$TUNNEL_URL" ] && [ -f "$CONFIG_FILE" ]; then
    echo "Reading tunnel URL from configuration..."
    source "$CONFIG_FILE"
fi

# Check if we got a URL
if [ -z "$TUNNEL_URL" ]; then
    echo "❌ No tunnel URL found"
    echo ""
    echo "Possible solutions:"
    echo "  1. Check if cloudflared service is running:"
    echo "     sudo systemctl status cloudflared.service"
    echo "  2. Run the installation script:"
    echo "     sudo ./Cloudflare-install.sh"
    echo "  3. Use the management script:"
    echo "     sudo manage-tunnel.sh url"
    exit 1
fi

echo ""
echo "✅ Current Tunnel URL:"
echo "   $TUNNEL_URL"

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
