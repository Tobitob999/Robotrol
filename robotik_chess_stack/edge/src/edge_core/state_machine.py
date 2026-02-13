from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from .logger import TrialLogger
from .safety import LimitViolation, clamp_theta, check_workspace, enforce_feedrate
from .verifier import Verifier
from .hw.fluidnc import FluidNC, FluidNCAlarm, FluidNCError
from .hw.gripper import Gripper
from .vision.cameras import CameraManager, CameraSpec
from .vision.board_pose import BoardPose, detect_board_pose
from .vision.occupancy import OccupancyDetector
from .net.agent_client import AgentClient
from .skills import pick as pick_skill
from .skills import place as place_skill
from .skills import capture as capture_skill
from .skills import reset as reset_skill


@dataclass
class SkillContext:
    cfg: Dict
    fluid: FluidNC
    gripper: Gripper
    verifier: Verifier

    def square_to_xy(self, square: str) -> Tuple[float, float]:
        calib = self.cfg.get("calibration", {}).get("robot_board", {})
        a1 = np.array(calib.get("a1", [0.0, 0.0]), dtype=float)
        a8 = np.array(calib.get("a8", [0.0, 280.0]), dtype=float)
        h1 = np.array(calib.get("h1", [280.0, 0.0]), dtype=float)
        h8 = np.array(calib.get("h8", [280.0, 280.0]), dtype=float)
        file_idx = ord(square[0].lower()) - ord("a")
        rank_idx = int(square[1]) - 1
        u = file_idx / 7.0
        v = rank_idx / 7.0
        xy = (1 - u) * (1 - v) * a1 + u * (1 - v) * h1 + (1 - u) * v * a8 + u * v * h8
        return float(xy[0]), float(xy[1])

    def clamp_theta(self, theta: Dict[str, float]) -> Dict[str, float]:
        clamps = self.cfg.get("skills", {}).get("clamps", {})
        return clamp_theta(theta, clamps)


class EdgeCore:
    def __init__(self, cfg: Dict):
        self.cfg = cfg
        serial_cfg = cfg["serial"]
        self.fluid = FluidNC(serial_cfg["port"], serial_cfg["baudrate"], serial_cfg["timeout_s"])
        self.gripper = Gripper(enabled=cfg.get("gripper", {}).get("enabled", False))

        cam_specs = [
            CameraSpec(
                name=cam["name"],
                path=cam["path"],
                width=int(cam["width"]),
                height=int(cam["height"]),
                fps=int(cam["fps"]),
            )
            for cam in cfg.get("cameras", {}).get("devices", [])
        ]
        self.cameras = CameraManager(cam_specs)

        square_size_mm = cfg["vision"]["board"]["square_size_mm"]
        self.occupancy = OccupancyDetector(square_size_mm, cfg["vision"]["occupancy"]["threshold"])
        min_conf = cfg["vision"]["occupancy"].get("min_confidence", 0.0)
        self.verifier = Verifier(self._get_occupancy, min_confidence=min_conf)
        edge_cfg = cfg.get("edge", {})
        self.logger = TrialLogger(edge_cfg["trial_log"], edge_cfg.get("image_ring_dir"), edge_cfg.get("image_ring_max", 0))

        if cfg.get("agent", {}).get("enabled", False):
            self.agent = AgentClient(cfg["agent"]["base_url"], cfg["agent"]["psk"], cfg["agent"]["psk_header"])
        else:
            self.agent = None

    def start(self) -> None:
        self.fluid.connect()
        self.cameras.open_all()

    def stop(self) -> None:
        self.cameras.close_all()
        self.fluid.disconnect()

    def _get_occupancy(self):
        frames = self.cameras.read_all()
        frame = frames.get("board") or next(iter(frames.values()))
        pose = detect_board_pose(
            frame,
            self.cfg["vision"]["aruco"]["dict"],
            self.cfg["vision"]["aruco"]["ids"],
            self.cfg["vision"]["board"]["square_size_mm"],
        )
        if pose:
            self.occupancy.set_pose(pose)
        elif self.cfg["vision"]["board_pose"].get("homography"):
            H = np.array(self.cfg["vision"]["board_pose"]["homography"], dtype=float)
            H_inv = np.linalg.inv(H)
            corners = self.cfg["vision"]["board_pose"].get("corners_px") or []
            self.occupancy.set_pose(BoardPose(H, H_inv, corners, confidence=1.0))
        if self.occupancy.background_gray is None:
            self.occupancy.update_background(frame)
        return self.occupancy.classify(frame)

    def execute_skill(self, skill_name: str, context: Dict, theta: Dict) -> Dict:
        ctx = SkillContext(cfg=self.cfg, fluid=self.fluid, gripper=self.gripper, verifier=self.verifier)
        theta = ctx.clamp_theta(theta)
        skill_fn = {
            "pick": pick_skill.run,
            "place": place_skill.run,
            "capture": capture_skill.run,
            "reset": reset_skill.run,
        }.get(skill_name)
        if not skill_fn:
            return {"success": False, "failure_code": "UNKNOWN"}
        retries = int(self.cfg.get("recovery", {}).get("retries", 0))
        success, failure = False, "UNKNOWN"
        for attempt in range(retries + 1):
            try:
                success, failure = skill_fn(ctx, context, theta)
            except LimitViolation:
                success, failure = False, "LIMIT_VIOLATION"
            except FluidNCAlarm:
                self._recover_alarm()
                success, failure = False, "GRBL_ALARM"
            except FluidNCError:
                success, failure = False, "UNKNOWN"
            except TimeoutError:
                success, failure = False, "TIMEOUT"
            except Exception:
                success, failure = False, "UNKNOWN"
            if success:
                break
            if failure not in ("PICK_NO_CHANGE", "PLACE_NO_CHANGE", "VISION_LOW_CONF"):
                break
        return {"success": success, "failure_code": failure}

    def run_once_from_agent(self) -> Dict:
        if not self.agent:
            return {"success": False, "failure_code": "UNKNOWN"}
        default_square = self.cfg.get("edge", {}).get("default_square", "a2")
        context = {"square": default_square, "region_id": self._square_to_region(default_square)}
        payload = self.agent.get_next(context)
        theta = payload.get("theta", {})
        skill = payload.get("skill", "pick")
        result = self.execute_skill(skill, context, theta)
        theta_id = payload.get("theta_id", "unknown")
        self.agent.report_trial(theta_id, context, theta, result)
        self._log_trial(theta_id, context, theta, result)
        return result

    def _square_to_region(self, square: str) -> str:
        file_idx = max(0, min(7, ord(square[0].lower()) - ord("a")))
        rank_idx = max(0, min(7, int(square[1]) - 1))
        region_x = file_idx // 2
        region_y = rank_idx // 2
        return f"r{region_y * 4 + region_x}"

    def _recover_alarm(self) -> None:
        recovery_cfg = self.cfg.get("recovery", {})
        if recovery_cfg.get("unlock_on_alarm", False):
            self.fluid.unlock()
            if recovery_cfg.get("home_on_unlock", False):
                self.fluid.home()

    def _log_trial(self, theta_id: str, context: Dict, theta: Dict, result: Dict) -> None:
        record = {
            "trial_id": str(uuid.uuid4()),
            "theta_id": theta_id,
            "context": context,
            "theta": theta,
            "outcome": result.get("success", False),
            "metrics": {"failure_code": result.get("failure_code")},
            "failure_code": result.get("failure_code"),
        }
        self.logger.log_trial(record)
