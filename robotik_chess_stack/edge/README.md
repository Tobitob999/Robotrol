# Edge Node (Moveo + FluidNC)

## Overview
The edge node handles safety, vision, and deterministic skills. It runs without internet by default and only pulls small residual parameters from the agent.

## Install (Ubuntu 22.04)
1. Copy this folder to `/opt/edge`.
2. Create an edge user and grant serial/video access:
   - `sudo useradd -m edge`
   - `sudo usermod -aG dialout,video edge`
3. Run the installer:
   - `sudo /opt/edge/scripts/install_edge.sh`
4. Edit config:
   - `/etc/edge/edge.yaml`

## Firewall (nftables)
Create `/etc/nftables.conf`:
```nft
table inet filter {
  chain input {
    type filter hook input priority 0;
    policy drop;
    iif lo accept
    ct state established,related accept
    ip saddr 192.168.50.0/24 tcp dport 22 accept
  }

  chain output {
    type filter hook output priority 0;
    policy drop;
    oif lo accept
    ct state established,related accept
    ip daddr 192.168.50.1 udp dport 53 accept
    ip daddr 192.168.50.1 tcp dport 53 accept
    udp dport 123 accept
    ip daddr 192.168.50.11 tcp dport 443 accept
    tcp dport 443 accept
  }
}
```
Enable:
- `sudo systemctl enable nftables`
- `sudo systemctl restart nftables`

## Calibration
1. Place ArUco markers at the board corners (IDs match `edge.yaml`).
2. Run board pose calibration:
   - `/opt/edge/venv/bin/python /opt/edge/scripts/calibrate_board.py --config /etc/edge/edge.yaml`
3. Jog the robot to corners and record:
   - `/opt/edge/venv/bin/python /opt/edge/scripts/calibrate_robot_board.py --config /etc/edge/edge.yaml`

## Bring-up
1. Connect FluidNC USB and cameras.
2. Run diagnostics:
   - `/opt/edge/venv/bin/python /opt/edge/scripts/diagnostics.py --config /etc/edge/edge.yaml`
3. Start service:
   - `sudo systemctl start edge-core.service`

## Troubleshooting
- Check logs: `journalctl -u edge-core.service -f`
- Verify camera path: `/dev/v4l/by-id/*`
- Verify serial port: `/dev/ttyUSB*`
