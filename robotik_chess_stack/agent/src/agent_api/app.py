from __future__ import annotations

import json
import os
import uuid
from pathlib import Path
from typing import Dict

import yaml
from fastapi import Depends, FastAPI, Request

from .auth import Auth
from .chess import ChessConfig, ChessEngine
from .optimizer import CMAESOptimizer, OptimizerConfig
from .schemas import ChessMoveRequest, ChessMoveResponse, HealthResponse, NextThetaResponse, ReportRequest
from .store import Store


def load_config(path: str) -> Dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Config root must be a mapping")
    return data


def parse_context(raw: str | None) -> Dict:
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {}


def create_app(config_path: str = "agent/config/agent.yaml") -> FastAPI:
    config_path = os.environ.get("AGENT_CONFIG", config_path)
    cfg = load_config(config_path)
    auth_cfg = cfg.get("auth", {})
    auth = Auth(auth_cfg.get("psk", ""), auth_cfg.get("header", "X-PSK"))

    store = Store(cfg["database"]["path"])
    opt_cfg = cfg.get("optimizer", {})
    optimizer = CMAESOptimizer(cfg.get("theta_clamps", {}), OptimizerConfig(**opt_cfg))

    chess_cfg = cfg.get("chess", {})
    chess = ChessEngine(
        ChessConfig(
            mode=chess_cfg.get("mode", "online"),
            online_url=chess_cfg.get("online", {}).get("url", ""),
            online_token=chess_cfg.get("online", {}).get("token", ""),
            stockfish_path=chess_cfg.get("stockfish", {}).get("path", "/usr/games/stockfish"),
        )
    )

    app = FastAPI(title="Robotik Chess Agent API")

    def require_auth(request: Request):
        auth.verify(request)

    @app.get("/v1/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        return HealthResponse()

    @app.get("/v1/learn/next", response_model=NextThetaResponse, dependencies=[Depends(require_auth)])
    def learn_next(context: str | None = None) -> NextThetaResponse:
        ctx = parse_context(context)
        payload = optimizer.next_theta(ctx, store)
        return NextThetaResponse(theta_id=payload["theta_id"], theta=payload["theta"], skill="pick")

    @app.post("/v1/learn/report", dependencies=[Depends(require_auth)])
    def learn_report(report: ReportRequest):
        trial_id = str(uuid.uuid4())
        store.record_trial(
            trial_id,
            report.context.dict(),
            report.theta_id,
            report.theta.dict(),
            report.outcome,
            report.metrics,
            report.failure_code,
        )
        optimizer.report(report.context.dict(), report.theta.dict(), report.outcome)
        return {"status": "ok"}

    @app.post("/v1/chess/move", response_model=ChessMoveResponse, dependencies=[Depends(require_auth)])
    def chess_move(req: ChessMoveRequest) -> ChessMoveResponse:
        uci, source = chess.move(req.fen)
        return ChessMoveResponse(uci=uci, source=source)

    return app


app = create_app()
