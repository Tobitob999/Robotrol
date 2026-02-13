# MVP Scope

## Edge
- Serial connect + line-by-line G-code with OK/ERROR handling
- Camera capture and ArUco board pose
- Occupancy detection per square (binary)
- Deterministic pick/place/capture/reset skills
- Safety clamps and workspace limits
- JSONL trial logging
- PSK-auth client to agent

## Agent
- FastAPI REST API
- SQLite store for trials + theta sets
- CMA-ES-style optimizer with clamps
- Online chess API + Stockfish fallback

## Docs
- Edge setup guide
- Agent setup guide
- Router setup guide (FRITZ!Box + OpenWrt fallback)
