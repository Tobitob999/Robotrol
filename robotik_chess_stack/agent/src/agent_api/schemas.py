from __future__ import annotations

from typing import Dict, Optional

from pydantic import BaseModel, Field


class Context(BaseModel):
    square: Optional[str] = None
    region_id: Optional[str] = None
    piece_class: str = "unknown"


class Theta(BaseModel):
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    dz_pick_mm: float = 0.0
    dz_place_mm: float = 0.0
    yaw_deg: float = 0.0
    v_approach: float = 1500.0
    v_lift: float = 1500.0
    v_place: float = 1500.0
    dwell_close_ms: float = 200.0
    dwell_release_ms: float = 200.0


class NextThetaResponse(BaseModel):
    theta_id: str
    theta: Theta
    skill: str = "pick"


class ReportRequest(BaseModel):
    theta_id: str
    context: Context
    theta: Theta
    outcome: bool
    metrics: Dict = Field(default_factory=dict)
    failure_code: Optional[str] = None


class ChessMoveRequest(BaseModel):
    fen: str


class ChessMoveResponse(BaseModel):
    uci: str
    source: str


class HealthResponse(BaseModel):
    status: str = "ok"
