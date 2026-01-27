# Cloudflare Tunnel Management & Auto-Discovery

This document explains how the DVI Bridge implements automatic tunnel URL discovery and management for the mobile app.

## Overview

The DVI Bridge now includes a robust tunnel management system that:
- ✅ Automatically monitors and updates tunnel URLs
- ✅ Exposes tunnel URL via HTTP API for app discovery
- ✅ Restarts tunnel automatically if it fails
- ✅ Updates mDNS records when tunnel changes
- ✅ Runs as systemd services for reliability

## Architecture

### Components

1. **cloudflared.service** - Systemd service that runs the Cloudflare tunnel
2. **tunnel-monitor.service** - Monitors cloudflared logs and updates tunnel URL
3. **monitor-tunnel.sh** - Script that watches logs and updates URL files
4. **manage-tunnel.sh** - Management script for controlling the tunnel
5. **/api/tunnel** - HTTP endpoint in webbridge.py that returns current URL

### How It Works

```
┌─────────────────┐
│  cloudflared    │──┐
│    service      │  │ Writes logs
└─────────────────┘  │
                     ▼
                ┌──────────────────┐
                │  cloudflared.log │
                └──────────────────┘
                     │
                     │ Monitors
                     ▼
          ┌──────────────────────┐
          │ tunnel-monitor.sh    │
          │   (tail -F logs)     │
          └──────────────────────┘
                     │
       ┌─────────────┼─────────────┐
       │             │             │
       ▼             ▼             ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│tunnel_url.txt│ │tunnel.conf  │ │mDNS service │
└─────────────┘ └─────────────┘ └─────────────┘
       │
       │ Reads
       ▼
┌─────────────────────┐
│  /api/tunnel        │
│  (webbridge.py)     │
└─────────────────────┘
       │
       │ HTTP GET
       ▼
┌─────────────────────┐
│  Mobile App         │
│  Auto-Discovery     │
└─────────────────────┘
```

## Installation

### Automatic (Recommended)

Run the Cloudflare installation script, which now sets up everything:

```bash
cd dvi-bridge-standalone/script
sudo ./Cloudflare-install.sh
```

This script will:
1. Download and install cloudflared
2. Install qrencode for QR codes
3. Create necessary directories
4. Install monitoring and management scripts
5. Set up systemd services
6. Start the tunnel and monitoring
7. Display the tunnel URL and QR code

### Manual Installation

If you need to install components separately:

```bash
# Install cloudflared
curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 -o cloudflared
sudo install -m 755 cloudflared /usr/local/bin/

# Install scripts
sudo install -m 755 script/monitor-tunnel.sh /usr/local/bin/
sudo install -m 755 script/manage-tunnel.sh /usr/local/bin/

# Create directories
sudo mkdir -p /var/log/dvi-bridge /var/run/dvi-bridge /etc/dvi-bridge

# Install systemd services
sudo cp src/systemd/cloudflared.service.example /etc/systemd/system/cloudflared.service
sudo cp src/systemd/tunnel-monitor.service.example /etc/systemd/system/tunnel-monitor.service

# Enable and start services
sudo systemctl daemon-reload
sudo systemctl enable --now cloudflared.service
sudo systemctl enable --now tunnel-monitor.service
```

## Management

### Using the Management Script

The `manage-tunnel.sh` script provides easy control:

```bash
# Show tunnel status
sudo manage-tunnel.sh status

# Show tunnel URL and QR code
sudo manage-tunnel.sh url

# Start/stop/restart tunnel
sudo manage-tunnel.sh start
sudo manage-tunnel.sh stop
sudo manage-tunnel.sh restart

# View live logs
sudo manage-tunnel.sh logs

# Enable/disable autostart
sudo manage-tunnel.sh enable
sudo manage-tunnel.sh disable

# Test the API endpoint
sudo manage-tunnel.sh test
```

### Using systemctl Directly

```bash
# Check service status
sudo systemctl status cloudflared.service
sudo systemctl status tunnel-monitor.service

# Restart services
sudo systemctl restart cloudflared.service
sudo systemctl restart tunnel-monitor.service

# View logs
sudo journalctl -u cloudflared.service -f
sudo journalctl -u tunnel-monitor.service -f

# Enable/disable autostart
sudo systemctl enable cloudflared.service
sudo systemctl disable cloudflared.service
```

## API Endpoint: /api/tunnel

### Request

```http
GET /api/tunnel HTTP/1.1
Host: bridge-address:5000
```

### Response (Success)

```json
{
  "tunnel_url": "https://example-word-1234.trycloudflare.com",
  "status": "active"
}
```

**Status Code:** 200 OK

### Response (Unavailable)

```json
{
  "status": "unavailable",
  "message": "Tunnel is currently being established or not running"
}
```

**Status Code:** 503 Service Unavailable

### Response (Error)

```json
{
  "status": "error",
  "message": "Error details..."
}
```

**Status Code:** 503 Service Unavailable

### Testing the Endpoint

```bash
# Test from bridge
curl http://localhost:5000/api/tunnel | python3 -m json.tool

# Test from another device on same network
curl http://bridge-ip:5000/api/tunnel | python3 -m json.tool

# Or use the management script
sudo manage-tunnel.sh test
```

## Mobile App Integration

### Automatic Discovery During mDNS

When the app discovers the bridge via mDNS:

1. App reads tunnel URL from mDNS TXT record (existing behavior)
2. App makes HTTP GET request to `/api/tunnel`
3. App saves the latest tunnel URL
4. App can now use this URL for remote access

```swift
// Example Swift code (already implemented in BridgeConfig.swift)
func fetchTunnelURLFromBridge(bridgeURL: String) {
    let url = URL(string: "\(bridgeURL)/api/tunnel")!
    
    URLSession.shared.dataTask(with: url) { data, response, error in
        guard let data = data,
              let json = try? JSONDecoder().decode(TunnelResponse.self, from: data),
              json.status == "active" else { return }
        
        // Save tunnel URL
        self.savedTunnelURL = json.tunnel_url
    }.resume()
}
```

### Manual Refresh

Users can manually refresh the tunnel URL:

```swift
// Triggered by "Update Tunnel URL" button in app
bridgeConfig.refreshTunnelURLIfLocal()
```

### Automatic Background Refresh

The app automatically refreshes when:
- Connecting to home network
- Network transitions occur
- App becomes active

## File Locations

### Runtime Files

- `/var/run/dvi-bridge/tunnel_url.txt` - Current tunnel URL (updated by monitor)
- `/var/log/dvi-bridge/cloudflared.log` - Cloudflared logs

### Configuration Files

- `/etc/dvi-bridge/tunnel.conf` - Persistent tunnel configuration
- `/etc/avahi/services/dvi-bridge.service` - mDNS service definition
- `/etc/systemd/system/cloudflared.service` - Cloudflared systemd service
- `/etc/systemd/system/tunnel-monitor.service` - Monitor systemd service

### Scripts

- `/usr/local/bin/monitor-tunnel.sh` - Tunnel monitoring script
- `/usr/local/bin/manage-tunnel.sh` - Tunnel management script

## Troubleshooting

### Tunnel URL Not Updating

```bash
# Check if monitoring service is running
sudo systemctl status tunnel-monitor.service

# Check monitor logs
sudo journalctl -u tunnel-monitor.service -f

# Restart monitor
sudo systemctl restart tunnel-monitor.service
```

### Tunnel Service Failing

```bash
# Check cloudflared status
sudo systemctl status cloudflared.service

# View recent logs
sudo journalctl -u cloudflared.service -n 50

# Check network connectivity
ping -c 4 1.1.1.1

# Restart tunnel
sudo manage-tunnel.sh restart
```

### API Endpoint Not Working

```bash
# Check if webbridge is running
sudo systemctl status webbridge.service

# Test endpoint directly
curl -v http://localhost:5000/api/tunnel

# Check file permissions
ls -la /var/run/dvi-bridge/tunnel_url.txt
ls -la /etc/dvi-bridge/tunnel.conf
```

### Tunnel URL File Not Created

```bash
# Check if monitor has write permissions
sudo ls -la /var/run/dvi-bridge/

# Create directory manually if needed
sudo mkdir -p /var/run/dvi-bridge
sudo chmod 755 /var/run/dvi-bridge

# Restart monitoring service
sudo systemctl restart tunnel-monitor.service
```

### mDNS Not Broadcasting

```bash
# Check if Avahi is running
sudo systemctl status avahi-daemon

# Restart Avahi
sudo systemctl restart avahi-daemon

# Check mDNS service file
cat /etc/avahi/services/dvi-bridge.service

# Test mDNS discovery (from another device)
avahi-browse -r _dvi-bridge._tcp
```

## Monitoring & Logs

### View Live Logs

```bash
# Cloudflared logs
sudo journalctl -u cloudflared.service -f

# Tunnel monitor logs
sudo journalctl -u tunnel-monitor.service -f

# Both services
sudo journalctl -u cloudflared.service -u tunnel-monitor.service -f
```

### Check Service Health

```bash
# Quick status check
sudo manage-tunnel.sh status

# Detailed systemctl status
sudo systemctl status cloudflared.service
sudo systemctl status tunnel-monitor.service

# Check if services are enabled
sudo systemctl is-enabled cloudflared.service
sudo systemctl is-enabled tunnel-monitor.service
```

### Resource Usage

```bash
# Check CPU and memory usage
sudo systemctl status cloudflared.service | grep -E "CPU:|Memory:"

# Use top to monitor
top -p $(pgrep cloudflared)
```

## Security Considerations

### Tunnel Security

- Cloudflare tunnels use HTTPS encryption
- No port forwarding required
- No firewall changes needed
- Tunnel URL is randomly generated each time

### API Security

- API endpoint is only accessible on local network (port 5000)
- No authentication required (relies on network security)
- Consider adding authentication if exposing publicly

### File Permissions

- Tunnel URL files are readable by all users
- Service runs as root (for system-level access)
- Consider restricting file permissions if needed:

```bash
sudo chmod 600 /var/run/dvi-bridge/tunnel_url.txt
sudo chmod 600 /etc/dvi-bridge/tunnel.conf
```

## Advanced Configuration

### Change Tunnel Port

Edit `/etc/systemd/system/cloudflared.service`:

```ini
ExecStart=/usr/local/bin/cloudflared tunnel --url http://localhost:8080 --logfile /var/log/dvi-bridge/cloudflared.log
```

Then reload and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart cloudflared.service
```

### Adjust Restart Behavior

Edit `/etc/systemd/system/cloudflared.service`:

```ini
[Service]
Restart=always
RestartSec=10          # Wait 10 seconds before restart
StartLimitInterval=300 # Reset failure count every 5 minutes
StartLimitBurst=5      # Allow up to 5 restarts in interval
```

### Custom Log Locations

Edit both service files to change log locations:

```bash
sudo nano /etc/systemd/system/cloudflared.service
sudo nano /usr/local/bin/monitor-tunnel.sh
```

## Benefits

### For Users

- ✅ No need to scan QR codes every time tunnel changes
- ✅ Automatic update when on home network
- ✅ Seamless experience between local and remote access
- ✅ No manual intervention needed

### For Bridge

- ✅ Automatic tunnel recreation if it fails
- ✅ Self-healing system
- ✅ Always-up-to-date URL in app
- ✅ Reliable remote access

### For Developers

- ✅ Clean API for tunnel discovery
- ✅ Standard systemd integration
- ✅ Easy to debug and maintain
- ✅ Extensible architecture

## Future Enhancements

Potential improvements:

1. **Named Tunnels** - Use Cloudflare named tunnels for persistent URLs
2. **Webhook Notifications** - Notify external services when URL changes
3. **HTTPS for API** - Add TLS support to webbridge
4. **Authentication** - Add API key authentication for /api/tunnel
5. **Multi-bridge Support** - Support multiple bridges with different tunnels
6. **Health Checks** - Add endpoint for tunnel connectivity testing
7. **Metrics** - Collect and expose tunnel uptime and traffic metrics

## Support

For issues or questions:

1. Check the troubleshooting section above
2. View logs: `sudo manage-tunnel.sh logs`
3. Check status: `sudo manage-tunnel.sh status`
4. Review systemd logs: `sudo journalctl -u cloudflared.service -xe`
5. Open an issue on GitHub with logs and configuration details
