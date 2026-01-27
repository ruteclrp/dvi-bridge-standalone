#!/bin/bash
# verify-tunnel-setup.sh
#
# Verification script to check if tunnel discovery is properly configured

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}=========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}=========================================${NC}"
    echo ""
}

print_pass() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_fail() {
    echo -e "${RED}❌ $1${NC}"
}

print_warn() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

PASSED=0
FAILED=0
WARNINGS=0

check_pass() {
    PASSED=$((PASSED + 1))
    print_pass "$1"
}

check_fail() {
    FAILED=$((FAILED + 1))
    print_fail "$1"
}

check_warn() {
    WARNINGS=$((WARNINGS + 1))
    print_warn "$1"
}

print_header "DVI Bridge Tunnel Setup Verification"

echo "This script checks if tunnel discovery is properly configured."
echo ""

# Check 1: cloudflared binary
print_info "Checking cloudflared installation..."
if command -v cloudflared &> /dev/null; then
    VERSION=$(cloudflared --version 2>&1 | head -1)
    check_pass "cloudflared is installed: $VERSION"
else
    check_fail "cloudflared is not installed"
    echo "   Install with: curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 -o cloudflared && sudo install -m 755 cloudflared /usr/local/bin/"
fi
echo ""

# Check 2: Scripts installed
print_info "Checking installed scripts..."
if [ -f "/usr/local/bin/monitor-tunnel.sh" ]; then
    check_pass "monitor-tunnel.sh is installed"
else
    check_fail "monitor-tunnel.sh is not installed"
    echo "   Should be at: /usr/local/bin/monitor-tunnel.sh"
fi

if [ -f "/usr/local/bin/manage-tunnel.sh" ]; then
    check_pass "manage-tunnel.sh is installed"
else
    check_fail "manage-tunnel.sh is not installed"
    echo "   Should be at: /usr/local/bin/manage-tunnel.sh"
fi
echo ""

# Check 3: Directories
print_info "Checking directories..."
for dir in "/var/run/dvi-bridge" "/var/log/dvi-bridge" "/etc/dvi-bridge"; do
    if [ -d "$dir" ]; then
        check_pass "Directory exists: $dir"
    else
        check_fail "Directory missing: $dir"
        echo "   Create with: sudo mkdir -p $dir"
    fi
done
echo ""

# Check 4: systemd services
print_info "Checking systemd services..."
if systemctl list-unit-files | grep -q "cloudflared.service"; then
    check_pass "cloudflared.service is installed"
    
    if systemctl is-enabled --quiet cloudflared.service 2>/dev/null; then
        check_pass "cloudflared.service is enabled"
    else
        check_warn "cloudflared.service is not enabled (won't start on boot)"
        echo "   Enable with: sudo systemctl enable cloudflared.service"
    fi
    
    if systemctl is-active --quiet cloudflared.service; then
        check_pass "cloudflared.service is running"
    else
        check_fail "cloudflared.service is not running"
        echo "   Start with: sudo systemctl start cloudflared.service"
    fi
else
    check_fail "cloudflared.service is not installed"
    echo "   Install from: src/systemd/cloudflared.service.example"
fi
echo ""

if systemctl list-unit-files | grep -q "tunnel-monitor.service"; then
    check_pass "tunnel-monitor.service is installed"
    
    if systemctl is-enabled --quiet tunnel-monitor.service 2>/dev/null; then
        check_pass "tunnel-monitor.service is enabled"
    else
        check_warn "tunnel-monitor.service is not enabled"
        echo "   Enable with: sudo systemctl enable tunnel-monitor.service"
    fi
    
    if systemctl is-active --quiet tunnel-monitor.service; then
        check_pass "tunnel-monitor.service is running"
    else
        check_warn "tunnel-monitor.service is not running"
        echo "   Start with: sudo systemctl start tunnel-monitor.service"
    fi
else
    check_fail "tunnel-monitor.service is not installed"
    echo "   Install from: src/systemd/tunnel-monitor.service.example"
fi
echo ""

# Check 5: Tunnel URL files
print_info "Checking tunnel URL availability..."
TUNNEL_URL=""

if [ -f "/var/run/dvi-bridge/tunnel_url.txt" ]; then
    TUNNEL_URL=$(cat /var/run/dvi-bridge/tunnel_url.txt 2>/dev/null || echo "")
    if [ -n "$TUNNEL_URL" ]; then
        check_pass "Tunnel URL file exists and has content"
        echo "   URL: $TUNNEL_URL"
    else
        check_warn "Tunnel URL file exists but is empty"
    fi
else
    check_fail "Tunnel URL file not found: /var/run/dvi-bridge/tunnel_url.txt"
fi

if [ -f "/etc/dvi-bridge/tunnel.conf" ]; then
    check_pass "Tunnel config file exists"
else
    check_warn "Tunnel config file not found: /etc/dvi-bridge/tunnel.conf"
fi
echo ""

# Check 6: API endpoint
print_info "Checking API endpoint..."
if command -v curl &> /dev/null; then
    if curl -s --connect-timeout 3 http://localhost:5000/api/tunnel > /dev/null 2>&1; then
        RESPONSE=$(curl -s http://localhost:5000/api/tunnel)
        if echo "$RESPONSE" | grep -q "tunnel_url"; then
            check_pass "API endpoint /api/tunnel is responding"
            API_URL=$(echo "$RESPONSE" | grep -oP '"tunnel_url":\s*"\K[^"]+' || echo "")
            if [ -n "$API_URL" ]; then
                echo "   API returned: $API_URL"
            fi
        else
            check_fail "API endpoint responded but with unexpected format"
            echo "   Response: $RESPONSE"
        fi
    else
        check_fail "API endpoint is not responding on http://localhost:5000/api/tunnel"
        echo "   Check if webbridge.py is running"
    fi
else
    check_warn "curl not installed, cannot test API endpoint"
    echo "   Install with: sudo apt-get install curl"
fi
echo ""

# Check 7: mDNS (optional)
print_info "Checking mDNS configuration (optional)..."
if command -v avahi-browse &> /dev/null; then
    if [ -f "/etc/avahi/services/dvi-bridge.service" ]; then
        check_pass "mDNS service file exists"
        
        if systemctl is-active --quiet avahi-daemon 2>/dev/null; then
            check_pass "Avahi daemon is running"
        else
            check_warn "Avahi daemon is not running"
        fi
    else
        check_warn "mDNS service file not found (optional)"
    fi
else
    check_warn "Avahi not installed (optional for mDNS discovery)"
fi
echo ""

# Check 8: Log files
print_info "Checking log files..."
if [ -f "/var/log/dvi-bridge/cloudflared.log" ]; then
    LOG_SIZE=$(du -h /var/log/dvi-bridge/cloudflared.log | cut -f1)
    check_pass "Cloudflared log exists: $LOG_SIZE"
    
    # Check if log has recent entries
    if [ -n "$(find /var/log/dvi-bridge/cloudflared.log -mmin -5 2>/dev/null)" ]; then
        check_pass "Log has recent entries (modified in last 5 minutes)"
    else
        check_warn "Log has no recent entries"
    fi
else
    check_warn "Cloudflared log not found (may not have started yet)"
fi
echo ""

# Summary
print_header "Verification Summary"
echo "Passed:   $PASSED"
echo "Failed:   $FAILED"
echo "Warnings: $WARNINGS"
echo ""

if [ $FAILED -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    print_pass "All checks passed! Tunnel discovery is properly configured."
    echo ""
    echo "Next steps:"
    echo "  1. Test the API: curl http://localhost:5000/api/tunnel"
    echo "  2. Check status: sudo manage-tunnel.sh status"
    echo "  3. View URL: sudo manage-tunnel.sh url"
    echo "  4. Configure mobile app to discover tunnel"
elif [ $FAILED -eq 0 ]; then
    print_warn "Setup is mostly complete, but there are some warnings."
    echo ""
    echo "Review the warnings above and fix if needed."
    echo "The tunnel should still work for basic functionality."
elif [ $FAILED -lt 3 ]; then
    check_warn "Setup is incomplete but may be partially functional."
    echo ""
    echo "Fix the failed checks above to ensure full functionality."
    echo "Run the installation script: sudo ./script/Cloudflare-install.sh"
else
    check_fail "Setup is incomplete. Please run the installation script."
    echo ""
    echo "Run: sudo ./script/Cloudflare-install.sh"
fi

echo ""
print_info "For detailed troubleshooting, see: TUNNEL_DISCOVERY.md"
echo ""

exit $FAILED
