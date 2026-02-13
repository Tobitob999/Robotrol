# FRITZ!Box 7490 Anleitung (Robotik-Gastnetz-Router)

Ziel: Die FRITZ!Box 7490 hängt mit ihrem WAN/Uplink im Heim‑Gastnetz und stellt ein eigenes Robotik‑LAN bereit.

## Ziel-Adressierung
- Robotik LAN: `192.168.50.0/24`
- Router LAN IP: `192.168.50.1`
- Edge: `192.168.50.10` (DHCP‑Lease)
- Agent: `192.168.50.11` (DHCP‑Lease)

## Variante A: FRITZ!Box als Router/NAT hinter Gastnetz
> Menünamen variieren je nach FRITZ!OS Version. Nutze die nächstliegende Option.

1. **Uplink ins Gastnetz**
   - Bevorzugt: FRITZ!Box als **WLAN‑Client** mit Gastnetz verbinden.
   - Alternativ: FRITZ!Box **WAN/LAN1** an einen Port im Gastnetz anschließen.
2. **Betriebsart**
   - FRITZ!Box als **Router mit eigenem Subnetz** konfigurieren (nicht als Repeater/Access‑Point).
3. **LAN IP setzen**
   - LAN IP: `192.168.50.1`
   - Subnetz: `255.255.255.0`
4. **DHCP aktivieren**
   - Bereich z.B. `192.168.50.100–192.168.50.200`
5. **Statische DHCP‑Leases**
   - Edge MAC → `192.168.50.10`
   - Agent MAC → `192.168.50.11`
6. **NAT / Masquerading**
   - Muss für LAN → WAN aktiv sein.
7. **Firewall**
   - WAN → LAN: **DROP**
   - **Keine Portforwards**
   - Optional: RFC1918 Richtung WAN blocken (Defense in depth).
8. **Optionales Robotik‑WLAN**
   - Eigenes SSID (nur lokal), getrennt vom Heimnetz.

## Variante B: Wenn FRITZ!Box keinen WAN‑Client sauber kann
Empfehlung: **OpenWrt‑Router** als Uplink ins Gastnetz.
1. WAN Interface per DHCP ins Gastnetz.
2. LAN Interface `192.168.50.1/24`.
3. NAT (Masquerading) von LAN → WAN.
4. DHCP Server auf LAN.
5. WAN → LAN default DROP, keine Portforwards.

## Tests (nach Einrichtung)
1. **Edge → Agent**
   - `curl http://192.168.50.11:8000/v1/health`
2. **Edge/Agent → Internet**
   - `curl https://example.com`
3. **Edge/Agent → Heimnetz NICHT erreichbar**
   - `ping 192.168.178.1` oder `ping 192.168.0.1` muss fehlschlagen
4. **Keine eingehenden Ports**
   - Externer Portscan: **keine** offenen Ports auf Edge/Agent

## Notizen
- Keine eingehenden Verbindungen zum Edge zulassen (kein Portforward, kein VPN, kein Reverse‑Tunnel).
- Edge bleibt auch ohne Internet funktionsfähig (Safety, Stop/Abort, Recovery).
