"""
UDP Mirror — extracted from Robotrol_FluidNC_v7_3.py (SerialClient + ExecuteApp).

Sends joint positions, path data, fixed-frame info, and robot profiles
to an external 3D visualizer via UDP (default 127.0.0.1:9999).

JSON message formats (type field determines semantics):

  "abs"           — Joint positions  {"type":"abs", "X":..., "Y":..., ..., "timestamp":...}
  "path_fixed"    — Fixed TCP path   {"type":"path_fixed", "pts":[[x,y,z,a], ...]}
  "path_kin"      — Kinematic path   {"type":"path_kin",   "pts":[[x,y,z,a], ...]}
  "fixed_frame"   — Coordinate frame {"type":"fixed_frame","origin":[x,y,z],"x":[...],"y":[...],"z":[...]}
  "robot_profile" — Full profile     {"type":"robot_profile","profile":"...","geometry_mm":{...}, ...}
"""

import json
import socket
import time
from typing import Optional


class UDPMirror:
    """UDP mirror for the external 3D visualizer.

    Parameters
    ----------
    addr : tuple[str, int]
        Target (host, port).  Default ``("127.0.0.1", 9999)``.
    enabled : bool
        If *False*, all send methods are no-ops.
    """

    def __init__(self, addr: tuple[str, int] = ("127.0.0.1", 9999), enabled: bool = True):
        self.addr = addr
        self.enabled = enabled
        self.sock: Optional[socket.socket] = None
        if enabled:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ------------------------------------------------------------------
    #  Low-level
    # ------------------------------------------------------------------

    def _send_json(self, data: dict) -> None:
        """Encode *data* as JSON and send via UDP."""
        if not self.enabled or not self.sock:
            return
        try:
            payload = (json.dumps(data) + "\n").encode("utf-8")
            self.sock.sendto(payload, self.addr)
        except Exception:
            pass

    def _send_raw(self, line: str) -> None:
        """Send a raw text line (e.g. mirrored G-Code)."""
        if not self.enabled or not self.sock:
            return
        try:
            self.sock.sendto((line + "\n").encode("utf-8"), self.addr)
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  Joint positions  (type: "abs")
    # ------------------------------------------------------------------

    def send_joints(self, joints: dict) -> None:
        """Send current joint positions to the visualizer.

        Parameters
        ----------
        joints : dict
            Mapping of axis name to position value, e.g.
            ``{"X": 10.0, "Y": -5.3, "Z": 0.0, "A": 45.0, "B": 0.0, "C": -12.5}``.

        JSON format sent::

            {
                "type": "abs",
                "X": 10.0,
                "Y": -5.3,
                "Z": 0.0,
                "A": 45.0,
                "B": 0.0,
                "C": -12.5,
                "timestamp": 1741900000.123
            }
        """
        msg: dict = {"type": "abs", "timestamp": time.time()}
        for ax, val in joints.items():
            msg[ax] = float(val)
        self._send_json(msg)

    # ------------------------------------------------------------------
    #  Path data  (type: "path_fixed" / "path_kin")
    # ------------------------------------------------------------------

    def send_path(self, points: list, path_type: str = "path_fixed") -> None:
        """Send a list of path points to the visualizer.

        Parameters
        ----------
        points : list[tuple[float, float, float, float]]
            Each entry is ``(x, y, z, a)`` where *a* is an optional
            extra axis value (typically joint-A / rotation).
        path_type : str
            Either ``"path_fixed"`` or ``"path_kin"``.

        JSON format sent::

            {
                "type": "path_fixed",
                "pts": [[x, y, z, a], ...]
            }
        """
        msg = {
            "type": path_type,
            "pts": [[float(a), float(b), float(c), float(d)] for a, b, c, d in points],
        }
        self._send_json(msg)

    # ------------------------------------------------------------------
    #  Fixed coordinate frame  (type: "fixed_frame")
    # ------------------------------------------------------------------

    def send_fixed_frame(self, origin, u, v, n) -> None:
        """Send a fixed coordinate frame to the visualizer.

        Parameters
        ----------
        origin : sequence[float]
            Frame origin ``[x, y, z]``.
        u, v, n : sequence[float]
            Frame axes (x, y, z unit vectors) each ``[x, y, z]``.

        JSON format sent::

            {
                "type": "fixed_frame",
                "origin": [ox, oy, oz],
                "x": [ux, uy, uz],
                "y": [vx, vy, vz],
                "z": [nx, ny, nz]
            }
        """
        msg = {
            "type": "fixed_frame",
            "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
            "x": [float(u[0]), float(u[1]), float(u[2])],
            "y": [float(v[0]), float(v[1]), float(v[2])],
            "z": [float(n[0]), float(n[1]), float(n[2])],
        }
        self._send_json(msg)

    # ------------------------------------------------------------------
    #  Robot profile  (type: "robot_profile")
    # ------------------------------------------------------------------

    def send_robot_profile(self, profile_data: dict) -> None:
        """Send the full robot profile to the visualizer.

        Parameters
        ----------
        profile_data : dict
            Pre-built profile dict.  Expected keys::

                {
                    "type": "robot_profile",     # set automatically
                    "profile": "Moveo",
                    "has_endstops": True,
                    "joint_order": ["X", "Y", "Z", "A", "B", "C"],
                    "geometry_mm": {
                        "base_height_mm": 240.0,
                        "L1_mm": 230.0,
                        "L2_mm": 250.0,
                        "L_tool_mm": 180.0
                    },
                    "dh_rows": [
                        {
                            "axis": "X",
                            "alpha_deg": -90.0,
                            "a_mm": 0.0,
                            "d_mm": 240.0,
                            "theta_offset_deg": 0.0
                        },
                        ...
                    ],
                    "post_transform": {
                        "mirror_x": false,
                        "sim_theta_offset_deg": {"X": 0.0, ...},
                        "sim_theta_scale": {"X": 1.0, ...}
                    },
                    "limits_deg": {
                        "X": [-100.0, 100.0],
                        ...
                    }
                }
        """
        msg = dict(profile_data)
        msg["type"] = "robot_profile"
        self._send_json(msg)

    # ------------------------------------------------------------------
    #  Mirror raw G-Code line  (original _mirror_udp behaviour)
    # ------------------------------------------------------------------

    def mirror_line(self, line: str) -> None:
        """Mirror a raw G-Code line to the visualizer.

        Also parses ``M3 S<value>`` and sends an additional ``"abs"``
        message with the spindle value (original behaviour).
        """
        self._send_raw(line)

        # Parse M3 S-value (spindle speed → visualizer)
        if line.startswith("M3") and "S" in line:
            try:
                s_val = float(line.split("S", 1)[1].strip())
                msg = {
                    "type": "abs",
                    "S": s_val,
                    "timestamp": time.time(),
                }
                self._send_json(msg)
            except Exception:
                pass

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Close the UDP socket."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.enabled = False
