#!/bin/bash
# manage-tunnel.sh
#
# Management script for cloudflared tunnel
# Provides commands to start, stop, restart, check status, and view tunnel URL

set -e

TUNNEL_URL_FILE="/var/run/dvi-bridge/tunnel_url.txt"
TUNNEL_CONFIG="/etc/dvi-bridge/tunnel.conf"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to check if services are installed
check_installation() {
    if ! systemctl list-unit-files | grep -q cloudflared.service; then
        print_status "$RED" "❌ cloudflared.service not installed"
        echo "   Run the installation script first"
        return 1
    fi
    return 0
}

# Function to get tunnel URL
get_tunnel_url() {
    if [ -f "$TUNNEL_URL_FILE" ]; then
        cat "$TUNNEL_URL_FILE"
    elif [ -f "$TUNNEL_CONFIG" ]; then
        grep "TUNNEL_URL=" "$TUNNEL_CONFIG" 2>/dev/null | cut -d'=' -f2
    else
        echo "unknown"
    fi
}

# Function to show status
show_status() {
    echo ""
    print_status "$BLUE" "═══════════════════════════════════════"
    print_status "$BLUE" "  DVI Bridge Tunnel Status"
    print_status "$BLUE" "═══════════════════════════════════════"
    echo ""
    
    # Check cloudflared service
    echo "Cloudflared Service:"
    if systemctl is-active --quiet cloudflared.service; then
        print_status "$GREEN" "  ✅ Running"
        systemctl status cloudflared.service --no-pager -l | grep -E "Active:|Memory:|CPU:" || true
    else
        print_status "$RED" "  ❌ Not running"
    fi
    echo ""
    
    # Check monitor service
    echo "Tunnel Monitor Service:"
    if systemctl is-active --quiet tunnel-monitor.service; then
        print_status "$GREEN" "  ✅ Running"
    else
        print_status "$YELLOW" "  ⚠️  Not running"
    fi
    echo ""
    
    # Show current tunnel URL
    TUNNEL_URL=$(get_tunnel_url)
    echo "Current Tunnel URL:"
    if [ "$TUNNEL_URL" != "unknown" ] && [ -n "$TUNNEL_URL" ]; then
        print_status "$GREEN" "  $TUNNEL_URL"
    else
        print_status "$YELLOW" "  ⚠️  URL not available yet"
    fi
    echo ""
    
    # Show uptime if running
    if systemctl is-active --quiet cloudflared.service; then
        echo "Uptime:"
        systemctl show cloudflared.service --property=ActiveEnterTimestamp --no-pager | cut -d'=' -f2
        echo ""
    fi
    
    print_status "$BLUE" "═══════════════════════════════════════"
}

# Function to show tunnel URL and QR code
show_url() {
    TUNNEL_URL=$(get_tunnel_url)
    
    echo ""
    print_status "$BLUE" "═══════════════════════════════════════"
    print_status "$BLUE" "  Current Tunnel URL"
    print_status "$BLUE" "═══════════════════════════════════════"
    echo ""
    
    if [ "$TUNNEL_URL" != "unknown" ] && [ -n "$TUNNEL_URL" ]; then
        print_status "$GREEN" "$TUNNEL_URL"
        echo ""
        
        # Generate QR code if qrencode is available
        if command -v qrencode &> /dev/null; then
            echo "QR Code:"
            echo ""
            qrencode -t ANSIUTF8 "$TUNNEL_URL" 2>/dev/null || qrencode -t ASCII "$TUNNEL_URL"
            echo ""
        else
            echo "Install qrencode to see QR code:"
            echo "  sudo apt-get install qrencode"
            echo ""
        fi
        
        echo "Or visit:"
        echo "  https://api.qrserver.com/v1/create-qr-code/?size=300x300&data=$TUNNEL_URL"
    else
        print_status "$RED" "❌ Tunnel URL not available"
        echo ""
        echo "Possible reasons:"
        echo "  - Tunnel is still starting up"
        echo "  - Tunnel service is not running"
        echo "  - Network connection issues"
        echo ""
        echo "Try: $0 status"
    fi
    
    echo ""
    print_status "$BLUE" "═══════════════════════════════════════"
}

# Function to start tunnel
start_tunnel() {
    print_status "$BLUE" "Starting cloudflared tunnel..."
    
    if sudo systemctl start cloudflared.service; then
        print_status "$GREEN" "✅ Cloudflared service started"
    else
        print_status "$RED" "❌ Failed to start cloudflared service"
        return 1
    fi
    
    if sudo systemctl start tunnel-monitor.service; then
        print_status "$GREEN" "✅ Tunnel monitor started"
    else
        print_status "$YELLOW" "⚠️  Tunnel monitor failed to start (non-critical)"
    fi
    
    echo ""
    print_status "$BLUE" "⏳ Waiting for tunnel URL (this may take 10-30 seconds)..."
    
    # Wait up to 60 seconds for tunnel URL
    for i in {1..60}; do
        sleep 1
        TUNNEL_URL=$(get_tunnel_url)
        if [ "$TUNNEL_URL" != "unknown" ] && [ -n "$TUNNEL_URL" ]; then
            echo ""
            print_status "$GREEN" "✅ Tunnel established!"
            echo ""
            show_url
            return 0
        fi
        echo -n "."
    done
    
    echo ""
    print_status "$YELLOW" "⚠️  Tunnel URL not available yet, but service is running"
    echo "Check logs with: journalctl -u cloudflared.service -f"
}

# Function to stop tunnel
stop_tunnel() {
    print_status "$BLUE" "Stopping cloudflared tunnel..."
    
    if sudo systemctl stop tunnel-monitor.service 2>/dev/null; then
        print_status "$GREEN" "✅ Tunnel monitor stopped"
    fi
    
    if sudo systemctl stop cloudflared.service; then
        print_status "$GREEN" "✅ Cloudflared service stopped"
    else
        print_status "$RED" "❌ Failed to stop cloudflared service"
        return 1
    fi
}

# Function to restart tunnel
restart_tunnel() {
    print_status "$BLUE" "Restarting cloudflared tunnel..."
    stop_tunnel
    sleep 2
    start_tunnel
}

# Function to show logs
show_logs() {
    echo ""
    print_status "$BLUE" "Showing cloudflared logs (Ctrl+C to exit)..."
    echo ""
    journalctl -u cloudflared.service -f
}

# Function to enable/disable autostart
enable_autostart() {
    print_status "$BLUE" "Enabling tunnel autostart on boot..."
    
    if sudo systemctl enable cloudflared.service; then
        print_status "$GREEN" "✅ Cloudflared will start on boot"
    fi
    
    if sudo systemctl enable tunnel-monitor.service; then
        print_status "$GREEN" "✅ Tunnel monitor will start on boot"
    fi
}

disable_autostart() {
    print_status "$BLUE" "Disabling tunnel autostart..."
    
    if sudo systemctl disable cloudflared.service; then
        print_status "$GREEN" "✅ Cloudflared autostart disabled"
    fi
    
    if sudo systemctl disable tunnel-monitor.service; then
        print_status "$GREEN" "✅ Tunnel monitor autostart disabled"
    fi
}

# Function to test API endpoint
test_api() {
    echo ""
    print_status "$BLUE" "Testing /api/tunnel endpoint..."
    echo ""
    
    if command -v curl &> /dev/null; then
        curl -s http://localhost:5000/api/tunnel | python3 -m json.tool || echo "Response is not JSON"
    else
        print_status "$RED" "❌ curl not installed"
        echo "Install with: sudo apt-get install curl"
    fi
    echo ""
}

# Main command handler
case "${1:-}" in
    start)
        check_installation && start_tunnel
        ;;
    stop)
        check_installation && stop_tunnel
        ;;
    restart)
        check_installation && restart_tunnel
        ;;
    status)
        check_installation && show_status
        ;;
    url|qr)
        show_url
        ;;
    logs)
        check_installation && show_logs
        ;;
    enable)
        check_installation && enable_autostart
        ;;
    disable)
        check_installation && disable_autostart
        ;;
    test)
        test_api
        ;;
    *)
        echo "DVI Bridge Tunnel Management"
        echo ""
        echo "Usage: $0 {start|stop|restart|status|url|logs|enable|disable|test}"
        echo ""
        echo "Commands:"
        echo "  start      - Start the cloudflared tunnel"
        echo "  stop       - Stop the cloudflared tunnel"
        echo "  restart    - Restart the cloudflared tunnel"
        echo "  status     - Show tunnel and service status"
        echo "  url (qr)   - Show current tunnel URL and QR code"
        echo "  logs       - Show live cloudflared logs"
        echo "  enable     - Enable tunnel autostart on boot"
        echo "  disable    - Disable tunnel autostart"
        echo "  test       - Test the /api/tunnel endpoint"
        echo ""
        exit 1
        ;;
esac
