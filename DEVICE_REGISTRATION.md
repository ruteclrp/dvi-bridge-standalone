# Device Registration Setup Guide

This guide explains how to set up device registration for the DVI Bridge RPi sidecar to connect to the Maker backend via Cloudflare Access while keeping the owner tunnel and hostname.

## Overview

Each RPi sidecar registers with the Maker backend to receive:
- Owner tunnel configuration (owner-only hostname)
- A device-scoped Cloudflare Access service token
- A device-scoped credential (e.g. JWT) for backend sessions
- Backend URL for the device channel

## Architecture

```
RPi (Owner's Network)
  ↓
  HTTPS POST /device/register (with CF-Access bootstrap headers)
  ↓
Maker Backend (issues owner tunnel + device Access token)
  ↓
  Returns: owner tunnel token + owner hostname + cf_access_client_id + cf_access_client_secret + backend_url
  ↓
RPi stores credentials locally and configures cloudflared
  ↓
Owner connects via https://<owner-hostname>
  ↓
RPi also connects outbound to Access-protected backend for maker support
```

## Prerequisites

- Raspberry Pi with network connectivity
- Python 3.7+
- Root/sudo access

## Installation

### 1. Configure Backend URL

Copy the example configuration and update with your backend URL:

```bash
cd src/sidecar
cp .env.example .env
nano .env
```

Update the `MAKER_BACKEND_URL` to your actual backend URL:
```env
MAKER_BACKEND_URL=https://your-backend.yourdomain.com
```

### 2. Run Setup Script

```bash
sudo bash script/setup-registration.sh
```

This will:
- Install Python dependencies
- Create configuration directories
- Copy .env to /etc/dvi-bridge/
- Install `systemd` units for tunnel sequencing
- Install heartbeat timer (every 2 minutes)
- Install tunnel watchdog timer (every 1 minute)

### 3. Register Device

```bash
sudo python3 src/sidecar/registration.py
```

This will:
- Determine device ID from FABNR in `.env`
- Contact the backend to register
- Receive owner tunnel configuration
- Receive device-scoped Access credentials
- Store credentials locally

### 4. Start Services

Enable and start services in this order:

```bash
sudo systemctl enable dvi-tunnel device-channel dvi-tunnels-ready
sudo systemctl enable dvi-heartbeat.timer dvi-tunnel-watchdog.timer

sudo systemctl start dvi-tunnel device-channel
sudo systemctl start dvi-tunnels-ready
sudo systemctl start dvi-heartbeat.timer dvi-tunnel-watchdog.timer
```

This ensures heartbeat waits until both tunnel services are active.

## Configuration Files

### /etc/dvi-bridge/.env
Main configuration file with backend URL and credentials:
```env
MAKER_BACKEND_URL=https://your-backend.yourdomain.com
CF_ACCESS_BOOTSTRAP_CLIENT_ID=xxx
CF_ACCESS_BOOTSTRAP_CLIENT_SECRET=xxx
CF_ACCESS_CLIENT_ID=xxx
CF_ACCESS_CLIENT_SECRET=xxx
REGISTRATION_API_TOKEN=xxx
```

### /home/dviha/dvi-bridge/device_registration.json
Stores received registration details:
```json
{
  "device_id": "pump-123456",
  "pump_id": "pump-123456",
  "backend_url": "https://backend.example.com",
  "mode": "backend_access",
  "registered_at": "2026-02-07T12:00:00Z"
}
```

### /home/dviha/dvi-bridge/tunnel_config.json
Stores the owner tunnel configuration:
```json
{
  "tunnel_id": "xxx",
  "tunnel_token": "xxx",
  "owner_hostname": "pump-xxx-owner.yourdomain.com"
}
```

## Device ID Generation

The device ID is derived from the FABNR value in `.env`.

Format: `pump-<FABNR>`

## Manual Operations

### Re-register Device
This creates a new device session and re-issues credentials (and owner tunnel):
```bash
sudo python3 src/sidecar/registration.py
```

## Heartbeat Keep-Alive

The sidecar periodically notifies the Maker backend to update the device `last_seen` value.

**Endpoint:** `POST /device/heartbeat`

**Body:**
```
pump_id (required)
status (optional)
```

The setup script installs `systemd` timer `dvi-heartbeat.timer` which triggers:

```bash
python /home/dviha/dvi-bridge/sidecar/heartbeat.py
```

This runs every 2 minutes and uses the same Cloudflare Access headers as registration.

To point heartbeat at a different base URL (for example, a server running on the Pi at port 8000), set:

```env
HEARTBEAT_BASE_URL=http://<pi-ip>:8000
```

## Troubleshooting

### Registration fails with 401 Unauthorized
- Check CF_ACCESS_BOOTSTRAP_CLIENT_ID and CF_ACCESS_BOOTSTRAP_CLIENT_SECRET in .env
- Verify backend allows Access service tokens for the registration endpoint

### Registration fails with connection error
- Check MAKER_BACKEND_URL is correct and reachable
- Verify network connectivity: `ping your-backend.yourdomain.com`

### Tunnel starts but owner can't access
- Wait 1-2 minutes for DNS propagation
- Check cloudflared logs: `sudo journalctl -u dvi-tunnel -f`
- Verify heatpump web interface is running on localhost:8080

### Tunnel or channel becomes unstable later
- Check watchdog logs: `sudo journalctl -u dvi-tunnel-watchdog.service -f`
- Verify timers: `sudo systemctl status dvi-heartbeat.timer dvi-tunnel-watchdog.timer`

### Device ID changes between reboots
- MAC address method failed, falling back to UUID
- Check network interfaces: `ip link show`
- Manually set device UUID in `/etc/dvi-bridge/device_uuid.txt`

## Security Notes

1. **RPi never has Cloudflare API credentials** - only the backend does
2. **Tunnel token is scoped** - only works for that specific tunnel
3. **CF-Access credentials** are pre-shared (like a registration key)
4. **Credentials file** has restricted permissions (0600)

## Integration with Existing Setup

The registration system works alongside the existing bridge and webbridge services:

- **bridge.py**: Continues to run and communicate with heatpump
- **webbridge.py**: Serves web interface on localhost:8080
- **cloudflared**: Tunnels localhost:8080 to the internet

All three services should run simultaneously.

## Files Created

```
/etc/dvi-bridge/
  └── .env                      # Configuration

/home/dviha/dvi-bridge/
  ├── device_registration.json  # Access/device registration
  ├── tunnel_config.json        # Owner tunnel config
  └── cloudflared/
      ├── config.yml            # Cloudflared config
      └── credentials.json      # Tunnel credentials

/etc/systemd/system/
  ├── dvi-tunnel.service                # Owner tunnel
  ├── device-channel.service            # Device backend channel
  ├── dvi-tunnels-ready.service         # Readiness gate
  ├── dvi-heartbeat.service             # Heartbeat sender
  ├── dvi-heartbeat.timer               # Heartbeat scheduler
  ├── dvi-tunnel-watchdog.service       # Tunnel watchdog
  └── dvi-tunnel-watchdog.timer         # Watchdog scheduler

/usr/local/bin/
  └── cloudflared               # Cloudflared binary
```

## Next Steps

After successful registration:
1. Access your heatpump at the provided owner URL
2. Monitor tunnel health via `dvi-tunnel` and `dvi-tunnel-watchdog` logs
3. Set up Cloudflare Access policies (optional) for additional security

## Support

For issues:
1. Check logs: `sudo journalctl -u dvi-tunnel -f`
2. Verify configuration in /etc/dvi-bridge/.env
3. Test backend connectivity manually with curl
4. Check if all services are running (bridge, webbridge, tunnel)
