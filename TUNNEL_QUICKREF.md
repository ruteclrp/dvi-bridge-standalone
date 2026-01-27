# Tunnel Discovery Quick Reference

## Installation

```bash
cd dvi-bridge-standalone/script
sudo ./Cloudflare-install.sh
```

## Management Commands

```bash
# Show status
sudo manage-tunnel.sh status

# Show URL & QR code
sudo manage-tunnel.sh url

# Start/Stop/Restart
sudo manage-tunnel.sh start
sudo manage-tunnel.sh stop
sudo manage-tunnel.sh restart

# View logs
sudo manage-tunnel.sh logs

# Test API
sudo manage-tunnel.sh test
```

## API Endpoint

**URL:** `http://bridge-ip:5000/api/tunnel`

**Method:** GET

**Response (Success - 200 OK):**
```json
{
  "tunnel_url": "https://example.trycloudflare.com",
  "status": "active"
}
```

**Response (Unavailable - 503):**
```json
{
  "status": "unavailable",
  "message": "Tunnel is currently being established"
}
```

## File Locations

| File | Purpose |
|------|---------|
| `/var/run/dvi-bridge/tunnel_url.txt` | Current tunnel URL (live) |
| `/etc/dvi-bridge/tunnel.conf` | Persistent config |
| `/var/log/dvi-bridge/cloudflared.log` | Cloudflared logs |
| `/etc/systemd/system/cloudflared.service` | Tunnel service |
| `/etc/systemd/system/tunnel-monitor.service` | Monitor service |

## Systemd Commands

```bash
# Check status
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

## Troubleshooting

### Tunnel URL not available

```bash
# Check if service is running
sudo systemctl status cloudflared.service

# Restart service
sudo manage-tunnel.sh restart

# View logs for errors
sudo manage-tunnel.sh logs
```

### API endpoint not responding

```bash
# Test endpoint
curl http://localhost:5000/api/tunnel

# Check if webbridge is running
sudo systemctl status webbridge.service

# Restart webbridge
sudo systemctl restart webbridge.service
```

### Monitor not updating URL

```bash
# Check monitor service
sudo systemctl status tunnel-monitor.service

# View monitor logs
sudo journalctl -u tunnel-monitor.service -f

# Restart monitor
sudo systemctl restart tunnel-monitor.service
```

## Verification

```bash
# Run verification script
cd dvi-bridge-standalone/script
sudo ./verify-tunnel-setup.sh
```

This will check:
- ✅ cloudflared installation
- ✅ Scripts installed
- ✅ Directories created
- ✅ Services running
- ✅ Tunnel URL available
- ✅ API responding

## Mobile App Integration

### Automatic Discovery

When on home network:
1. App connects via mDNS
2. App calls `/api/tunnel` endpoint
3. App saves latest tunnel URL
4. App uses URL for remote access

### Manual Refresh

In app settings:
1. Tap "Update Tunnel URL"
2. App fetches latest URL from bridge
3. New URL saved for remote access

### Remote Access

When away from home:
- App automatically uses tunnel URL
- No manual configuration needed
- Works seamlessly

## Key Benefits

- ✅ **Automatic Updates**: URL updates tracked automatically
- ✅ **Self-Healing**: Tunnel restarts if it fails
- ✅ **No QR Scanning**: App fetches URL via API
- ✅ **Secure**: HTTPS encryption via Cloudflare
- ✅ **No Port Forwarding**: No router configuration needed
- ✅ **Easy Management**: Simple commands for control

## Learn More

- 📖 **Full Documentation**: [TUNNEL_DISCOVERY.md](TUNNEL_DISCOVERY.md)
- 🔧 **Implementation Details**: [TUNNEL_IMPLEMENTATION.md](TUNNEL_IMPLEMENTATION.md)
- 📝 **Main README**: [README.md](README.md)

## Support

Having issues? Check:
1. Run verification: `sudo ./verify-tunnel-setup.sh`
2. Check status: `sudo manage-tunnel.sh status`
3. View logs: `sudo manage-tunnel.sh logs`
4. Read troubleshooting: [TUNNEL_DISCOVERY.md](TUNNEL_DISCOVERY.md#troubleshooting)
5. Open GitHub issue with logs
