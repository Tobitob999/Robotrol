# Protocol

## Auth
- Header: `X-PSK`
- All endpoints except `/v1/health` require PSK.

## Endpoints
- `GET /v1/health`
  - Response: `{ "status": "ok" }`
- `GET /v1/learn/next?context=<json>`
  - Response: `{ "theta_id": "...", "theta": { ... }, "skill": "pick" }`
- `POST /v1/learn/report`
  - Body: `{ "theta_id": "...", "context": { ... }, "theta": { ... }, "outcome": true, "metrics": {}, "failure_code": "..." }`
- `POST /v1/chess/move`
  - Body: `{ "fen": "..." }`
  - Response: `{ "uci": "e2e4", "source": "online|stockfish" }`
