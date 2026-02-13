# Agent Node (NUC)

## Overview
The agent runs the FastAPI service, optimization logic, and chess engine integration.

## Install (Ubuntu 22.04 Server)
1. Copy this folder to `/opt/agent`.
2. Create an agent user:
   - `sudo useradd -m agent`
3. Run the installer:
   - `sudo /opt/agent/scripts/install_agent.sh`
4. Edit config:
   - `/etc/agent/agent.yaml`

## Firewall (ufw example)
Allow only Robotik LAN:
- `sudo ufw default deny incoming`
- `sudo ufw default allow outgoing`
- `sudo ufw allow from 192.168.50.0/24 to any port 8000 proto tcp`
- `sudo ufw enable`

## Health Check
- `curl http://127.0.0.1:8000/v1/health`
- `curl -H "X-PSK: REPLACE_ME" "http://127.0.0.1:8000/v1/learn/next?context={}" `

## Troubleshooting
- Check logs: `journalctl -u agent-api.service -f`
- Verify DB: `/var/lib/agent/agent.db`
