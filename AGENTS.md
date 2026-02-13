# AGENTS.md — Robotik-Schach (Moveo + FluidNC) | 2 Rechner + Gastnetz-Router
Version: 1.0

## 0) Ziel
Baue die komplette Software für ein Robotiksystem mit:
- Moveo 6DOF, gesteuert über FluidNC (GRBL kompatibel) via Serial/USB
- 2× USB Logitech Kameras (UVC)
- Zwei Rechner:
  - EDGE: Hardware-nah, deterministisch, Safety, Vision, Skill-Ausführung
  - AGENT: alter Intel NUC (4 Kerne), Schach/Optimierung/Policy-Training, REST API
- Netzwerk: eigenes Robotik-LAN hinter eigenem Router, Router hängt am Heim-Gastnetz (Internet ja, Heimnetz nein)
- Schach darf online bezogen werden; mechanisches Lernen lokal/NUC (CPU-only) ist ok.
- Kein OpenClaw.

## 1) Hard Constraints (nicht verhandelbar)
- Agent darf niemals direkte Motion/G-Code Trajektorien frei vorgeben. Nur Skill-Level + kleine Residualparameter innerhalb enger Grenzen.
- Edge muss ohne Internet sicher sein (Safety, Stop/Abort, Recovery, Timeout).
- Keine eingehenden Verbindungen von außen zum Edge (kein Portforward, kein VPN, kein Reverse-Tunnel).
- Vision ist primäres Feedback (kein Kraftsensor vorhanden).

## 2) Ziel-Repo Struktur (zu erzeugen)
/edge
  /src/edge_core
    main.py
    config.py
    state_machine.py
    safety.py
    verifier.py
    logger.py
    /hw
      fluidnc.py
      gripper.py (stub; optional)
    /vision
      cameras.py
      board_pose.py
      occupancy.py
    /skills
      pick.py
      place.py
      capture.py
      reset.py
    /net
      agent_client.py
  /config
    edge.yaml
  /systemd
    edge-core.service
  /scripts
    install_edge.sh
    calibrate_board.py
    calibrate_robot_board.py
    diagnostics.py
  README.md

/agent
  /src/agent_api
    app.py
    auth.py
    store.py
    chess.py
    optimizer.py
    policy.py (optional, behind flag)
    schemas.py
  /config
    agent.yaml
  /systemd
    agent-api.service
  /scripts
    install_agent.sh
    init_db.py
  README.md

/common
  schemas.md
  protocol.md
  failure_codes.md

/specs
  mvp.md
  acceptance_tests.md

## 3) OS/Setup Vorgaben
- Edge OS: Ubuntu 22.04 LTS Desktop oder Server (x86_64)
- Agent OS (NUC): Ubuntu 22.04 LTS Server (x86_64)
- Python: 3.10+
- Keine GPU vorausgesetzt.

## 4) Netzwerk / Router (eigene Komponenten + Konfiguration)
### 4.1 Zieltopologie
- Heimnetz: privat, nicht erreichbar
- Heim-Gastnetz: Internet-only SSID/VLAN (Uplink)
- Robotik-Router (eigener Router; z.B. FRITZ!Box 7490) hängt mit WAN/Uplink am Gastnetz
- Robotik-LAN (hinter Router): z.B. 192.168.50.0/24
  - Edge: 192.168.50.10 (DHCP reservation)
  - Agent (NUC): 192.168.50.11 (DHCP reservation)

### 4.2 Router-Komponenten (Pflichten)
- WAN/Uplink ins Gastnetz (Internet)
  - bevorzugt: WAN via WLAN-Client/Repeater-Brücke zum Gastnetz ODER Ethernet am Gastnetzport
- NAT (Masquerading) zwischen Robotik-LAN und Gastnetz
- DHCP + DNS (lokal) im Robotik-LAN
- Firewall:
  - inbound WAN->LAN: DROP
  - keine Portforwards
  - optional: block RFC1918 Richtung WAN (defense in depth)
- Statische DHCP Leases für Edge/Agent (MAC-basiert)
- Optional: eigenes Robotik-WLAN (SSID) für Service-Laptop; sonst nur LAN-Ports.

### 4.3 FRITZ!Box 7490 (Beispiel-Anforderungen)
Da FRITZ!OS je nach Version beim "WLAN-Repeater/Client" und "Router hinter Router" eingeschränkt ist:
- Implementiere die Router-Anleitung so, dass sie 2 Varianten abdeckt:
  A) FRITZ!Box als Router/NAT hinter Gastnetz (wenn WAN/Client Mode möglich)
  B) Alternative Empfehlung: OpenWrt Router (falls FRITZ!Box die WAN-Client-Anforderung nicht erfüllt)
Die Anleitung muss Tests enthalten:
- Von Edge/Agent: Internet erreichbar (HTTPS)
- Von Edge/Agent: Heimnetz-IP (z.B. 192.168.178.1 oder 192.168.0.1 je nach Heimnetz) NICHT erreichbar
- Keine eingehenden Ports von außen auf Edge/Agent.

## 5) Host-Firewalls
### 5.1 Edge nftables Policy (Pflicht)
- INPUT: default DROP; allow lo + established/related; optional SSH nur aus 192.168.50.0/24
- OUTPUT: default DROP; allow:
  - DNS (53 udp/tcp) zum Router (192.168.50.1)
  - NTP (123 udp) zum Router oder pool
  - HTTPS (443 tcp) nur zu Agent (192.168.50.11) + optional Internet (für Schach/updates wenn nötig)
- Optional defense: block outbound zu RFC1918 außer Robotik-LAN.

### 5.2 Agent (NUC) Policy
- INPUT: allow 8000/tcp (FastAPI) nur aus 192.168.50.0/24
- OUTPUT: allow DNS/NTP/HTTPS (für online chess falls genutzt)

## 6) Edge — Funktionsumfang (Pflichten)
### 6.1 Hardware I/O
- FluidNC Serial:
  - connect/disconnect robust
  - send G-code line by line with OK/ERROR handling
  - status polling "?" (configurable interval)
  - handle alarms: detect, stop, provide recovery sequence (unlock+home optional)
- Cameras:
  - enumerate UVC devices by stable identifier (by-id if possible)
  - capture frames at configurable rate
  - support selecting resolution/fps in config

### 6.2 Vision
- Board pose detection:
  - ArUco markers (IDs configurable) on 4 board corners
  - compute homography and board coordinate frame
- Occupancy:
  - per-square ROI -> "occupied/unoccupied" (binary)
  - method: background model + threshold OR simple classifier; must work with static lighting
- Verification:
  - after each skill step: compare expected board delta to observed
  - if mismatch: rescan N times, then failure code

### 6.3 Calibration Tools (scripts)
- calibrate_board.py:
  - guides user through placing/seeing markers, stores board transform
- calibrate_robot_board.py:
  - 4-point mapping a1, a8, h1, h8 to robot coordinates (manual jog + record)
  - computes square grid mapping and stores in edge.yaml
- diagnostics.py:
  - camera test (show marker detection + occupancy overlay)
  - serial test (connect, status, simple move dry-run without touching board)

### 6.4 Skills (deterministic; clamp residuals)
- pick(square, theta):
  - move to safe Z above square
  - approach Z_pick (clamped)
  - close gripper (if available; else placeholder)
  - lift to safe Z
  - verify square became empty (best effort)
- place(square, theta):
  - move safe Z above square
  - approach Z_place
  - open gripper
  - retreat
  - verify square became occupied
- capture(square, theta):
  - pick(square)
  - place(graveyard_slot)
- reset():
  - move to home/safe pose
  - optional: do nothing else (board reset is out of scope unless implemented later)

### 6.5 Safety / Recovery State Machine
- Global limits:
  - workspace bounds (min/max x,y,z)
  - max feedrate/accel
  - timeouts per phase
- Recovery:
  - retry pick/place up to N (configurable)
  - on GRBL alarm: stop, attempt unlock+home (if configured), abort_safe
  - always ensure final state: safe pose + motors stopped

### 6.6 Logging
- JSONL trial log:
  - trial_id, timestamp
  - context (square/region)
  - theta_id, theta used
  - outcome: success boolean
  - metrics: duration, center_error (if computed), retries, failure_code
- Optional ringbuffer of images (pre/post) capped by size in config

### 6.7 Networking (Edge -> Agent)
- Pull model:
  - GET /v1/learn/next?context=...
  - POST /v1/learn/report
  - POST /v1/chess/move (optional on Edge; can also be on Agent only)
- Auth via PSK header.

## 7) Agent (NUC) — Funktionsumfang (Pflichten)
### 7.1 REST API (FastAPI)
- Port: 8000 (configurable)
- Endpoints:
  - GET /v1/health
  - GET /v1/learn/next?context=...
  - POST /v1/learn/report
  - POST /v1/chess/move
- Auth:
  - X-PSK required for all endpoints except /health (optional)
  - reject unauthorized with 401

### 7.2 Store (SQLite)
- Tables:
  - trials(trial_id, ts, context, theta_id, theta_json, outcome, metrics_json, failure_code)
  - theta_sets(theta_id, context, theta_json, created_ts, notes, performance_json)
  - models(model_id, type, path, created_ts, metrics_json) (optional)
- Must support simple queries for latest/best theta per context.

### 7.3 Optimizer
- Implement CMA-ES (preferred) with constraints:
  - internally optimize unconstrained, then clamp to limits
  - choose next theta based on context bucket (region_id)
- Provide:
  - exploration initially
  - exploitation after enough trials
- Must be robust to noisy outcomes (binary success).

### 7.4 Policy Learning (optional but implement behind feature flag)
- Train lightweight model on CPU:
  - Input: context features (region_id) + optional numeric metrics; NO raw images mandatory
  - Output: theta residual proposal
- Must keep safety clamps identical.
- Must support versioning and rollback.

### 7.5 Schach
- Implement two modes (config):
  A) Online API (generic; user supplies URL/token in config)
  B) Local Stockfish binary as fallback (must ship install instructions and wrapper)
- Input: FEN
- Output: UCI move
- If online fails: fallback to stockfish, report source in response.

## 8) Common Protocol / Data Schemas
### 8.1 Theta schema (canonical)
theta = {
  dx_mm, dy_mm,
  dz_pick_mm, dz_place_mm,
  yaw_deg,
  v_approach, v_lift, v_place,
  dwell_close_ms, dwell_release_ms
}
clamps provide min/max for each field.

### 8.2 Context schema
context = {
  square: "a1".."h8" (optional),
  region_id: "r0".."r15" (4x4 regions),
  piece_class: string (optional; default "unknown")
}

### 8.3 Failure codes (must implement + document)
- VISION_NO_BOARD
- VISION_LOW_CONF
- PICK_NO_CHANGE
- PLACE_NO_CHANGE
- GRBL_ALARM
- TIMEOUT
- LIMIT_VIOLATION
- RETRIES_EXHAUSTED
- UNKNOWN

## 9) Installation / Setup Anleitungen (MUSS erstellt werden)
### 9.1 Edge Setup Guide (README + scripts)
Must include:
- Ubuntu install
- packages install
- add user to dialout
- camera permissions
- install_edge.sh (idempotent):
  - apt deps, python venv, pip deps
  - install systemd service
  - deploy default config to /etc/edge/edge.yaml
- firewall setup (nftables) + enable
- calibration steps:
  - board markers placement
  - run calibrate_board.py
  - run calibrate_robot_board.py
- bring-up:
  - connect fluidnc, cameras
  - run diagnostics.py
  - start edge-core.service
- troubleshooting section

### 9.2 Agent (NUC) Setup Guide
Must include:
- Ubuntu server install
- install_agent.sh (idempotent):
  - apt deps, python venv, pip deps
  - install stockfish
  - init_db.py
  - install systemd service agent-api.service
- firewall (ufw or nftables) allowing only robotik LAN
- config agent.yaml:
  - psk
  - optimizer settings
  - chess mode (online url/token optional)
- health check curl examples

### 9.3 Router Setup Guide (FRITZ!Box 7490 + fallback)
Must include:
- Desired addressing (192.168.50.0/24)
- Steps to:
  - connect router WAN to guest network
  - enable DHCP in robotik LAN
  - set static leases
  - disable port forwards
- Tests:
  - edge can reach agent (curl /v1/health)
  - edge/agent can reach internet
  - edge/agent cannot reach heimnetz gateway IP
- Fallback section:
  - if FRITZ!Box cannot do WAN-as-guest properly, recommend OpenWrt router and describe minimal required settings (WAN=guest, LAN=robotik, NAT, DHCP).

## 10) Acceptance Tests (automatisierbar wo möglich)
- Unit-level:
  - agent optimizer returns theta within clamps
  - agent stores trials and retrieves best theta
- Integration-level (no robot motion required):
  - edge starts, connects cameras, detects board markers (in front of camera)
  - edge can talk to agent API with PSK
- Hardware-level (requires robot):
  - dry-run move with safe Z only (no pick) executes without GRBL alarm
  - pick/place on single square shows expected occupancy delta (basic)

## 11) Implementation Notes (Codex execution rules)
- Prefer Python for both edge and agent.
- Edge should avoid heavy deps; keep OpenCV minimal.
- All config in YAML with clear defaults und comments.
- Provide clear error messages and deterministic retries.
- Do not invent hardware details; use config flags and safe defaults.

## 12) Deliverables Checklist (must complete)
- Repo structure with edge/agent/common/specs as above
- All scripts (install/calibrate/diagnostics) present and runnable
- systemd services for edge-core and agent-api
- Router setup guide (FRITZ!Box 7490 + fallback)
- Complete READMEs with step-by-step install and verification commands
- Default configs with placeholders for PSK and camera device paths
