# Tunnel URL Discovery - Implementation Summary

## Overview

This implementation adds automatic tunnel URL discovery to the DVI Bridge, allowing the mobile app to fetch the latest tunnel URL without requiring users to scan QR codes every time the tunnel changes.

## Changes Made

### 1. Web API Endpoint (`src/sidecar/webbridge.py`)

**Added `/api/tunnel` endpoint:**
- Returns current tunnel URL in JSON format
- Reads from `/var/run/dvi-bridge/tunnel_url.txt` (live) or `/etc/dvi-bridge/tunnel.conf` (fallback)
- Returns 200 OK when tunnel is active
- Returns 503 Service Unavailable when tunnel is down

**Response format:**
```json
{
  "tunnel_url": "https://example.trycloudflare.com",
  "status": "active"
}
```

### 2. Tunnel Monitoring (`script/monitor-tunnel.sh`)

**New monitoring script that:**
- Watches cloudflared logs using `tail -F`
- Extracts tunnel URLs using regex pattern matching
- Updates `/var/run/dvi-bridge/tunnel_url.txt` atomically
- Updates `/etc/dvi-bridge/tunnel.conf` for persistence
- Updates mDNS TXT record when URL changes
- Reloads Avahi daemon to broadcast new URL

### 3. Systemd Services

**Created two new service files:**

**`src/systemd/cloudflared.service.example`:**
- Runs cloudflared as a system service
- Automatically restarts on failure
- Logs to `/var/log/dvi-bridge/cloudflared.log`
- Starts after network is online
- Resource limits: 256MB memory, 50% CPU

**`src/systemd/tunnel-monitor.service.example`:**
- Runs monitor-tunnel.sh as a service
- Depends on cloudflared.service
- Automatically restarts if it fails
- Logs to systemd journal

### 4. Management Script (`script/manage-tunnel.sh`)

**Comprehensive tunnel management tool with commands:**
- `start` - Start the tunnel
- `stop` - Stop the tunnel
- `restart` - Restart the tunnel
- `status` - Show detailed status
- `url` - Display URL and QR code
- `logs` - Show live logs
- `enable/disable` - Control autostart
- `test` - Test the API endpoint

**Features:**
- Colored output for better readability
- Service health checking
- Uptime information
- QR code generation
- API endpoint testing

### 5. Updated Installation (`script/Cloudflare-install.sh`)

**Enhanced installation script:**
- Installs cloudflared binary
- Installs monitoring and management scripts
- Creates necessary directories
- Sets up systemd services
- Enables and starts services
- Waits for tunnel to establish
- Shows final URL and instructions

**New steps added:**
- Install monitor-tunnel.sh to /usr/local/bin
- Install manage-tunnel.sh to /usr/local/bin
- Create log and runtime directories
- Copy service files to /etc/systemd/system
- Reload systemd daemon
- Enable services for autostart
- Start services and wait for URL

### 6. Updated QR Display (`script/QR-show.sh`)

**Improved QR code display:**
- Reads from live tunnel_url.txt first
- Falls back to config file
- Better error messages
- Suggests using manage-tunnel.sh

### 7. Documentation (`TUNNEL_DISCOVERY.md`)

**Comprehensive documentation covering:**
- Architecture overview with diagram
- Installation instructions (automatic and manual)
- Management commands and examples
- API endpoint specification
- Mobile app integration guide
- File locations reference
- Troubleshooting guide
- Monitoring and logging instructions
- Security considerations
- Advanced configuration options

## File Structure

```
dvi-bridge-standalone/
├── script/
│   ├── Cloudflare-install.sh      # Updated: systemd setup
│   ├── monitor-tunnel.sh          # New: tunnel monitoring
│   ├── manage-tunnel.sh           # New: tunnel management
│   └── QR-show.sh                 # Updated: live URL reading
├── src/
│   ├── sidecar/
│   │   └── webbridge.py           # Updated: /api/tunnel endpoint
│   └── systemd/
│       ├── cloudflared.service.example         # New
│       └── tunnel-monitor.service.example      # New
└── TUNNEL_DISCOVERY.md            # New: documentation
```

## Runtime Files Created

```
/var/run/dvi-bridge/
└── tunnel_url.txt                 # Live tunnel URL (updated by monitor)

/var/log/dvi-bridge/
└── cloudflared.log                # Cloudflared logs

/etc/dvi-bridge/
└── tunnel.conf                    # Persistent tunnel configuration

/etc/avahi/services/
└── dvi-bridge.service             # mDNS service definition

/etc/systemd/system/
├── cloudflared.service            # Cloudflared systemd service
└── tunnel-monitor.service         # Monitor systemd service

/usr/local/bin/
├── monitor-tunnel.sh              # Monitoring script
└── manage-tunnel.sh               # Management script
```

## How It Works

### Startup Sequence

1. **systemd** starts `cloudflared.service`
2. **cloudflared** creates tunnel and writes logs
3. **systemd** starts `tunnel-monitor.service`
4. **monitor-tunnel.sh** tails cloudflared logs
5. **monitor-tunnel.sh** extracts URL and writes to files
6. **monitor-tunnel.sh** updates mDNS service
7. **webbridge.py** reads URL from files
8. **Mobile app** fetches URL via `/api/tunnel`

### URL Update Flow

```
Tunnel recreates → cloudflared logs new URL → 
monitor-tunnel.sh detects change → 
Updates tunnel_url.txt + tunnel.conf + mDNS → 
webbridge.py serves new URL → 
Mobile app fetches new URL
```

### Service Dependencies

```
network-online.target
    ↓
cloudflared.service
    ↓
tunnel-monitor.service
```

## Testing

### Test API Endpoint

```bash
# Local test
curl http://localhost:5000/api/tunnel | python3 -m json.tool

# From another device
curl http://bridge-ip:5000/api/tunnel | python3 -m json.tool

# Using management script
sudo manage-tunnel.sh test
```

### Test Service Restart

```bash
# Restart tunnel
sudo systemctl restart cloudflared.service

# Wait 10-30 seconds
sleep 30

# Check new URL
sudo manage-tunnel.sh url

# Verify API returns new URL
curl http://localhost:5000/api/tunnel | python3 -m json.tool
```

### Test Mobile App Discovery

1. Ensure app is connected to home WiFi
2. Restart tunnel: `sudo manage-tunnel.sh restart`
3. In app, tap "Update Tunnel URL" button
4. Verify new URL is fetched and saved

## Benefits

### Reliability
- Automatic tunnel restart on failure
- Self-healing system
- No manual intervention needed

### User Experience
- No QR code scanning for URL updates
- Automatic discovery when on home network
- Seamless local/remote switching

### Maintainability
- Standard systemd integration
- Clean API design
- Comprehensive logging
- Easy troubleshooting

## Compatibility

### Requirements
- Raspberry Pi or Linux system
- systemd (most modern Linux distributions)
- Python 3.6+ (for webbridge.py)
- Avahi daemon (for mDNS, optional)
- qrencode (for QR codes, optional)

### Tested On
- Raspberry Pi OS (Bullseye)
- Ubuntu 20.04+
- Debian 11+

## Migration from Manual Setup

If you previously installed cloudflared manually:

1. Stop existing cloudflared process:
   ```bash
   pkill cloudflared
   ```

2. Run the new installation:
   ```bash
   sudo ./script/Cloudflare-install.sh
   ```

3. Verify services are running:
   ```bash
   sudo manage-tunnel.sh status
   ```

4. Update mobile app with new URL (automatic if on home network)

## Known Limitations

1. **URL Changes**: Tunnel URL changes each time cloudflared restarts
2. **Named Tunnels**: Currently uses quick tunnels, not named tunnels
3. **Authentication**: API endpoint has no authentication (local network only)
4. **IPv6**: Not tested with IPv6-only networks

## Future Improvements

1. **Named Tunnels**: Use Cloudflare named tunnels for persistent URLs
2. **HTTPS API**: Add TLS support to webbridge
3. **Authentication**: Add API key for /api/tunnel endpoint
4. **Push Notifications**: Notify app when URL changes
5. **Health Monitoring**: Add tunnel connectivity checks
6. **Metrics**: Collect uptime and traffic statistics

## Support

For issues:
1. Check logs: `sudo manage-tunnel.sh logs`
2. Check status: `sudo manage-tunnel.sh status`
3. Review TUNNEL_DISCOVERY.md troubleshooting section
4. Open GitHub issue with logs and configuration

## Credits

Implementation based on:
- Cloudflare's trycloudflare quick tunnels
- systemd service management
- Flask web framework
- Avahi mDNS implementation
