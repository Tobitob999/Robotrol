# Router Setup Guide (FRITZ!Box 7490 + Fallback)

## Addressing
- Robotik LAN: `192.168.50.0/24`
- Edge: `192.168.50.10` (DHCP reservation)
- Agent: `192.168.50.11` (DHCP reservation)
- Router LAN IP: `192.168.50.1`

## Variante A: FRITZ!Box als Router hinter Gastnetz
1. WAN/Uplink ins Heim-Gastnetz (WLAN-Client oder Ethernet am Gastnetzport).
2. FRITZ!Box im Router-Modus mit eigenem Subnetz `192.168.50.0/24`.
3. DHCP aktiv, statische Leases für Edge/Agent.
4. NAT aktiv, keine Portforwards.
5. Firewall: WAN->LAN drop, optional RFC1918 Richtung WAN blocken.

## Variante B: OpenWrt Router (Fallback)
1. WAN Interface im Gastnetz (DHCP).
2. LAN Interface `192.168.50.1/24`.
3. NAT (Masquerading) von LAN->WAN.
4. DHCP Server auf LAN.
5. Keine Portforwards, WAN->LAN default drop.

## Tests
- Edge -> Agent: `curl http://192.168.50.11:8000/v1/health`
- Edge/Agent -> Internet: `curl https://example.com`
- Edge/Agent -> Heimnetz Gateway nicht erreichbar:
  - `ping 192.168.178.1` (oder 192.168.0.1) muss fehlschlagen
- Keine eingehenden Ports von außen auf Edge/Agent (Portscan von extern muss leer sein)
