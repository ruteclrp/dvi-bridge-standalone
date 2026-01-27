#!/bin/bash
# /usr/local/bin/monitor-tunnel.sh
#
# Monitors cloudflared logs and automatically updates tunnel URL
# when it changes. Also updates mDNS TXT record for discovery.
#
# This script should be run as a daemon alongside cloudflared.

set -e

# Configuration
TUNNEL_URL_FILE="/var/run/dvi-bridge/tunnel_url.txt"
TUNNEL_CONFIG="/etc/dvi-bridge/tunnel.conf"
CLOUDFLARED_LOG="/var/log/dvi-bridge/cloudflared.log"
MDNS_SERVICE="/etc/avahi/services/dvi-bridge.service"

# Ensure directory exists
mkdir -p "$(dirname "$TUNNEL_URL_FILE")"
mkdir -p "$(dirname "$CLOUDFLARED_LOG")"

echo "🔍 Starting tunnel URL monitor..."
echo "   Log file: $CLOUDFLARED_LOG"
echo "   URL file: $TUNNEL_URL_FILE"

# Function to update tunnel URL
update_tunnel_url() {
    local new_url="$1"
    local current_url=""
    
    # Read current URL if file exists
    if [ -f "$TUNNEL_URL_FILE" ]; then
        current_url=$(cat "$TUNNEL_URL_FILE" 2>/dev/null || echo "")
    fi
    
    # Only update if URL has changed
    if [ "$new_url" != "$current_url" ]; then
        echo "✅ New tunnel URL detected: $new_url"
        
        # Write to runtime file (atomic)
        echo "$new_url" > "${TUNNEL_URL_FILE}.tmp"
        mv "${TUNNEL_URL_FILE}.tmp" "$TUNNEL_URL_FILE"
        
        # Update config file
        if [ -f "$TUNNEL_CONFIG" ]; then
            sed -i "s|TUNNEL_URL=.*|TUNNEL_URL=$new_url|" "$TUNNEL_CONFIG"
        else
            mkdir -p "$(dirname "$TUNNEL_CONFIG")"
            echo "TUNNEL_URL=$new_url" > "$TUNNEL_CONFIG"
        fi
        
        # Update mDNS TXT record if Avahi is available
        if command -v avahi-publish &> /dev/null && [ -d "/etc/avahi/services" ]; then
            echo "📡 Updating mDNS service..."
            
            # Update mDNS service file
            cat > "$MDNS_SERVICE" <<EOF
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name>DVI Bridge</name>
  <service>
    <type>_dvi-bridge._tcp</type>
    <port>5000</port>
    <txt-record>tunnel=$new_url</txt-record>
  </service>
</service-group>
EOF
            
            # Reload avahi daemon to pick up changes
            if systemctl is-active --quiet avahi-daemon; then
                systemctl reload avahi-daemon || true
            fi
            
            echo "✅ mDNS service updated"
        fi
        
        echo "✅ Tunnel URL updated successfully"
        echo "   URL: $new_url"
        echo "   Updated at: $(date)"
    fi
}

# Monitor log file
echo "👀 Monitoring cloudflared logs for tunnel URL..."

# If log file doesn't exist yet, wait for it
while [ ! -f "$CLOUDFLARED_LOG" ]; do
    echo "⏳ Waiting for cloudflared log file to be created..."
    sleep 2
done

# Tail the log file and process each line
tail -F "$CLOUDFLARED_LOG" 2>/dev/null | while read -r line; do
    # Extract trycloudflare URL from log line
    # Format: "https://something-random-word.trycloudflare.com"
    if [[ $line =~ (https://[a-z0-9-]+\.trycloudflare\.com) ]]; then
        TUNNEL_URL="${BASH_REMATCH[1]}"
        update_tunnel_url "$TUNNEL_URL"
    fi
done
