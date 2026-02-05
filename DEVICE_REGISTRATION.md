# Device Registration Setup Guide

This guide explains how to set up device registration for the DVI Bridge RPi sidecar to connect to the Maker backend via Cloudflare tunnels.

## Overview

Each RPi sidecar registers with the Maker backend to receive:
- Unique Cloudflare tunnel configuration
- Tunnel token (for secure connection)
- Owner and Maker hostnames for remote access

## Architecture

```
RPi (Owner's Network)
    ↓
    HTTPS POST /device/register (with CF-Access headers)
    ↓
Maker Backend (creates tunnel via Cloudflare API)
    ↓
    Returns: tunnel_token, hostnames
    ↓
RPi configures and starts cloudflared
    ↓
Tunnel established to Cloudflare
    ↓
Owner/Maker can access heatpump via https://pump-xxx-owner.yourdomain.com
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
- Install cloudflared
- Install Python dependencies
- Create configuration directories
- Copy .env to /etc/dvi-bridge/
- Install systemd service

### 3. Register Device

```bash
sudo python3 src/sidecar/registration.py
```

This will:
- Generate or retrieve unique device ID (based on MAC address)
- Contact the backend to register
- Receive tunnel configuration
- Configure cloudflared
- Start the tunnel

### 4. Enable Automatic Startup

```bash
sudo systemctl enable dvi-tunnel
sudo systemctl start dvi-tunnel
```

## Configuration Files

### /etc/dvi-bridge/.env
Main configuration file with backend URL and credentials:
```env
MAKER_BACKEND_URL=https://your-backend.yourdomain.com
CF_ACCESS_CLIENT_ID=xxx
CF_ACCESS_CLIENT_SECRET=xxx
CLOUDFLARE_ACCOUNT_ID=xxx
```

### /etc/dvi-bridge/tunnel_config.json
Stores received tunnel configuration:
```json
{
  "tunnel_id": "xxx",
  "tunnel_token": "xxx",
  "owner_hostname": "pump-xxx-owner.yourdomain.com",
  "maker_hostname": "pump-xxx-maker.yourdomain.com"
}
```

### /etc/cloudflared/config.yml
Cloudflared configuration:
```yaml
tunnel: <tunnel-id>
credentials-file: /etc/cloudflared/credentials.json

ingress:
  - hostname: pump-xxx-owner.yourdomain.com
    service: http://localhost:8080
  - hostname: pump-xxx-maker.yourdomain.com
    service: http://localhost:8080
  - service: http_status:404
```

## Systemd Service

The `dvi-tunnel.service` runs cloudflared in the background:

```bash
# Check status
sudo systemctl status dvi-tunnel

# View logs
sudo journalctl -u dvi-tunnel -f

# Restart tunnel
sudo systemctl restart dvi-tunnel
```

## Device ID Generation

The device ID is generated based on:
1. **Primary**: MAC address of eth0
2. **Fallback 1**: MAC address of wlan0
3. **Fallback 2**: Generated UUID (saved to /etc/dvi-bridge/device_uuid.txt)

Format: `pump-<mac-address-without-colons>` or `pump-<uuid>`

## Manual Operations

### Re-register Device
This creates a new tunnel:
```bash
sudo python3 src/sidecar/registration.py
```

### Manually Start Tunnel (without systemd)
```bash
cloudflared tunnel run --token <tunnel-token>
```

### Check Tunnel Status
```bash
cloudflared tunnel info
```

## Troubleshooting

### Registration fails with 401 Unauthorized
- Check CF_ACCESS_CLIENT_ID and CF_ACCESS_CLIENT_SECRET in .env
- Verify backend is using the same credentials

### Registration fails with connection error
- Check MAKER_BACKEND_URL is correct and reachable
- Verify network connectivity: `ping your-backend.yourdomain.com`

### Tunnel starts but owner can't access
- Wait 1-2 minutes for DNS propagation
- Check cloudflared logs: `sudo journalctl -u dvi-tunnel -f`
- Verify heatpump web interface is running on localhost:8080

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
  ├── .env                      # Configuration
  ├── tunnel_config.json        # Received tunnel config
  └── device_uuid.txt           # Generated device ID (if needed)

/etc/cloudflared/
  ├── config.yml                # Cloudflared config
  └── credentials.json          # Tunnel credentials

/etc/systemd/system/
  └── dvi-tunnel.service        # Systemd service

/usr/local/bin/
  └── cloudflared               # Cloudflared binary
```

## Next Steps

After successful registration:
1. Access your heatpump at the provided owner URL
2. Share maker URL with service technicians if needed
3. Monitor tunnel health via systemd logs
4. Set up Cloudflare Access policies (optional) for additional security

## Support

For issues:
1. Check logs: `sudo journalctl -u dvi-tunnel -f`
2. Verify configuration in /etc/dvi-bridge/.env
3. Test backend connectivity manually with curl
4. Check if all services are running (bridge, webbridge, tunnel)
