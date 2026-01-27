# Changelog - Tunnel Discovery Feature

## Version 1.1.0 - Automatic Tunnel URL Discovery (January 2026)

### 🎉 New Features

#### Automatic Tunnel URL Discovery
- Mobile apps can now automatically fetch the latest tunnel URL from the bridge
- No need to manually scan QR codes every time the tunnel changes
- Seamless experience when switching between local and remote access

#### HTTP API Endpoint
- **New Endpoint**: `GET /api/tunnel` returns current tunnel URL
- Returns JSON with tunnel URL and status
- Automatically reads from live tunnel tracking file
- Falls back to config file if needed

#### Tunnel Monitoring System
- **New Script**: `monitor-tunnel.sh` watches cloudflared logs
- Automatically extracts and updates tunnel URL when it changes
- Updates runtime file, config file, and mDNS records
- Runs as systemd service for reliability

#### Systemd Integration
- **New Service**: `cloudflared.service` runs tunnel with automatic restart
- **New Service**: `tunnel-monitor.service` monitors tunnel status
- Both services start on boot
- Self-healing: automatic restart on failure
- Resource limits to prevent system overload

#### Tunnel Management
- **New Script**: `manage-tunnel.sh` provides easy tunnel control
- Commands: start, stop, restart, status, url, logs, enable, disable, test
- Colored output for better readability
- Shows tunnel URL, QR code, and service status
- Built-in API endpoint testing

#### Enhanced Installation
- **Updated**: `Cloudflare-install.sh` now sets up complete system
- Installs and configures all components
- Sets up systemd services automatically
- Waits for tunnel to establish before completing
- Shows final URL and management instructions

#### Verification Tool
- **New Script**: `verify-tunnel-setup.sh` checks installation
- Validates all components and services
- Reports passed, failed, and warning checks
- Provides detailed troubleshooting information

### 📝 File Changes

#### Modified Files
1. **src/sidecar/webbridge.py**
   - Added `TUNNEL_URL_PATH` constant
   - Added `/api/tunnel` endpoint
   - Reads from runtime file with config fallback
   - Proper error handling and status codes

2. **script/Cloudflare-install.sh**
   - Added systemd service installation
   - Added monitoring script installation
   - Added directory creation
   - Added service enable and start
   - Enhanced output and instructions

3. **script/QR-show.sh**
   - Updated to read from live tunnel URL file
   - Better error messages and fallback
   - Suggests using management script

#### New Files
1. **script/monitor-tunnel.sh**
   - Monitors cloudflared logs
   - Extracts and updates tunnel URLs
   - Updates mDNS records
   - Runs continuously as daemon

2. **script/manage-tunnel.sh**
   - Management interface for tunnel
   - Multiple commands for control
   - Status display with colors
   - QR code generation
   - API testing

3. **script/verify-tunnel-setup.sh**
   - Comprehensive setup verification
   - Checks all components
   - Reports status and issues
   - Provides troubleshooting suggestions

4. **src/systemd/cloudflared.service.example**
   - Systemd service for cloudflared
   - Automatic restart configuration
   - Resource limits
   - Logging setup

5. **src/systemd/tunnel-monitor.service.example**
   - Systemd service for monitoring
   - Depends on cloudflared service
   - Automatic restart
   - Journal logging

6. **TUNNEL_DISCOVERY.md**
   - Comprehensive documentation
   - Architecture overview
   - Installation instructions
   - Management guide
   - Troubleshooting section
   - API specification

7. **TUNNEL_IMPLEMENTATION.md**
   - Implementation summary
   - File structure overview
   - How it works explanations
   - Testing procedures
   - Migration guide

8. **TUNNEL_QUICKREF.md**
   - Quick reference guide
   - Common commands
   - API reference
   - Troubleshooting tips
   - File locations

### 🔧 Technical Details

#### Architecture
```
Tunnel Service → Logs → Monitor → URL Files → API → Mobile App
                             ↓
                         mDNS Update
```

#### File Locations
- `/var/run/dvi-bridge/tunnel_url.txt` - Live tunnel URL
- `/var/log/dvi-bridge/cloudflared.log` - Cloudflared logs
- `/etc/dvi-bridge/tunnel.conf` - Persistent configuration
- `/etc/avahi/services/dvi-bridge.service` - mDNS definition
- `/etc/systemd/system/cloudflared.service` - Tunnel service
- `/etc/systemd/system/tunnel-monitor.service` - Monitor service
- `/usr/local/bin/monitor-tunnel.sh` - Monitoring script
- `/usr/local/bin/manage-tunnel.sh` - Management script

#### Dependencies
- cloudflared (required)
- systemd (required)
- Python 3.6+ (required for webbridge)
- avahi-daemon (optional for mDNS)
- qrencode (optional for QR codes)
- curl (optional for testing)

### 📱 Mobile App Integration

#### Automatic Discovery
When app discovers bridge via mDNS:
1. Reads tunnel URL from mDNS TXT record
2. Calls `/api/tunnel` endpoint
3. Saves latest tunnel URL
4. Uses URL for remote access

#### Manual Refresh
- "Update Tunnel URL" button in app
- Fetches latest URL from bridge
- Updates saved URL immediately

#### Background Refresh
- Automatic when connecting to home network
- Triggered on network transitions
- Silent updates in background

### 🎯 Benefits

#### For Users
- ✅ No manual QR code scanning needed
- ✅ Automatic URL updates on home network
- ✅ Seamless local/remote switching
- ✅ Works even if tunnel restarts

#### For System
- ✅ Self-healing tunnel
- ✅ Automatic restart on failure
- ✅ Always-up-to-date URLs
- ✅ Reliable remote access

#### For Developers
- ✅ Clean REST API
- ✅ Standard systemd integration
- ✅ Easy to debug and maintain
- ✅ Well-documented

### 🔄 Migration from Manual Setup

If you previously ran cloudflared manually:

1. Stop existing cloudflared process:
   ```bash
   pkill cloudflared
   ```

2. Run new installation:
   ```bash
   sudo ./script/Cloudflare-install.sh
   ```

3. Verify setup:
   ```bash
   sudo ./script/verify-tunnel-setup.sh
   ```

4. Mobile app will auto-discover new URL (when on home network)

### ⚠️ Breaking Changes
None - This is a backward-compatible addition

### 🐛 Known Issues
1. Tunnel URL changes each time cloudflared restarts (by design)
2. API endpoint has no authentication (local network only)
3. Not tested with IPv6-only networks

### 🔮 Future Enhancements
- Named Cloudflare tunnels for persistent URLs
- HTTPS support for API endpoint
- API key authentication
- Push notifications for URL changes
- Health monitoring dashboard
- Uptime and traffic metrics

### 📚 Documentation
- [TUNNEL_DISCOVERY.md](TUNNEL_DISCOVERY.md) - Complete guide
- [TUNNEL_IMPLEMENTATION.md](TUNNEL_IMPLEMENTATION.md) - Technical details
- [TUNNEL_QUICKREF.md](TUNNEL_QUICKREF.md) - Quick reference
- [README.md](README.md) - Updated main README

### 🙏 Credits
- Cloudflare for free tunnel service
- systemd for reliable service management
- Flask for lightweight web framework
- Avahi for mDNS implementation

### 📝 Notes
This feature was developed to enhance the mobile app experience by eliminating the need to manually update tunnel URLs when the Cloudflare quick tunnel is recreated. The implementation follows best practices for systemd services and provides a robust, self-healing system.

---

## Installation Instructions

### New Installation
```bash
cd dvi-bridge-standalone/script
sudo ./Cloudflare-install.sh
```

### Verify Installation
```bash
sudo ./script/verify-tunnel-setup.sh
```

### Check Status
```bash
sudo manage-tunnel.sh status
```

### View Tunnel URL
```bash
sudo manage-tunnel.sh url
```

For detailed instructions, see [TUNNEL_DISCOVERY.md](TUNNEL_DISCOVERY.md).
