import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk

from control import config as config_loader
from control.app import build_pipeline
from learning.tnt_self_learning import TntSelfLearningManager
from control.transforms import (
    make_transform,
    matmul,
    invert_transform,
    normalize_vector,
    cross,
    extract_translation,
)
from perception.camera import Frame
from simulation.simulation_loop import SimulationRunner


class _CameraCaptureAdapter:
    def __init__(self, camera_capture):
        self._camera_capture = camera_capture

    def get_frame(self):
        image = None
        if self._camera_capture is not None:
            image = self._camera_capture.get_latest_frame()
        if image is None:
            raise RuntimeError("No camera frame available")
        return Frame(image=image, timestamp=time.time())


class PickPlaceTab(ttk.Frame):
    def __init__(self, master, logger=None, camera_capture=None, execute_app=None):
        super().__init__(master)
        self._logger = logger
        self._pipeline = None
        self._configs = None
        self._worker = None
        self._stop_event = threading.Event()
        self._base_dir = os.path.abspath(os.path.dirname(__file__))
        self._camera_capture = camera_capture

        self._status_var = tk.StringVar(value="Not initialized")
        self._stack_index_var = tk.StringVar(value="Stack index: n/a")
        self._run_count = tk.IntVar(value=1)
        self._sim_cycles = tk.IntVar(value=1000)
        self._detect_var = tk.StringVar(value="No detection yet.")
        self._execute_app = execute_app
        self._learner = None
        self._learning_enabled_var = tk.BooleanVar(value=False)
        self._learning_mode_var = tk.StringVar(value="shadow")
        self._learning_status_var = tk.StringVar(value="Learning: disabled")
        self._learning_event_count = 0
        self._last_cycle_ctx = {}

        self._pattern_cols_var = tk.IntVar(value=7)
        self._pattern_rows_var = tk.IntVar(value=7)
        self._square_size_var = tk.DoubleVar(value=32.0)
        self._marker_axis_var = tk.StringVar(value="z")
        self._marker_sign_var = tk.IntVar(value=1)
        self._tnt_h1_low_var = tk.IntVar(value=0)
        self._tnt_h1_high_var = tk.IntVar(value=12)
        self._tnt_h2_low_var = tk.IntVar(value=168)
        self._tnt_h2_high_var = tk.IntVar(value=180)
        self._tnt_s_min_var = tk.IntVar(value=40)
        self._tnt_v_min_var = tk.IntVar(value=30)
        self._tnt_min_area_var = tk.IntVar(value=500)
        self._tnt_kernel_var = tk.IntVar(value=3)

        self._base_T_board_vars = None
        self._base_T_cam_vars = None
        self._marker_T_obj_vars = None
        self._board_points = {}
        self._board_point_vars = {}
        self._cam_calib_objpoints = []
        self._cam_calib_imgpoints = []
        self._cam_calib_image_size = None
        self._cam_calib_pattern = None
        self._cam_calib_square = None
        self._cam_calib_result = None
        self._cam_calib_status = tk.StringVar(value="No samples")
        self._cam_calib_result_var = tk.StringVar(value="")

        self._build_ui()
        self._refresh_learning_from_manager()
        self._load_calibration_into_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        header.columnconfigure(3, weight=1)

        ttk.Button(header, text="Init Pipeline", command=self._init_pipeline).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(header, text="Stop", command=self._stop).grid(row=0, column=1, padx=(0, 6))
        ttk.Label(header, text="Status:").grid(row=0, column=2, sticky="w")
        ttk.Label(header, textvariable=self._status_var).grid(row=0, column=3, sticky="w")

        tabs = ttk.Notebook(self)
        tabs.grid(row=1, column=0, sticky="nsew", padx=6, pady=4)
        self.rowconfigure(1, weight=1)

        run_tab = ttk.Frame(tabs)
        sim_tab = ttk.Frame(tabs)
        cfg_tab = ttk.Frame(tabs)
        calib_tab = ttk.Frame(tabs)
        help_tab = ttk.Frame(tabs)
        tabs.add(run_tab, text="Run")
        tabs.add(sim_tab, text="Simulation")
        tabs.add(cfg_tab, text="Config")
        tabs.add(calib_tab, text="Calibration")
        tabs.add(help_tab, text="Help")

        self._build_run_tab(run_tab)
        self._build_sim_tab(sim_tab)
        self._build_cfg_tab(cfg_tab)
        self._build_calib_tab(calib_tab)
        self._build_help_tab(help_tab)

    def _build_run_tab(self, tab):
        tab.columnconfigure(1, weight=1)
        btns = ttk.Frame(tab)
        btns.grid(row=0, column=0, sticky="w", padx=6, pady=6)

        ttk.Button(btns, text="Run 1 Cycle", command=self._run_one_cycle).grid(row=0, column=0, padx=(0, 6))
        ttk.Label(btns, text="Cycles:").grid(row=0, column=1, padx=(0, 4))
        ttk.Entry(btns, textvariable=self._run_count, width=6).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(btns, text="Run N Cycles", command=self._run_n_cycles).grid(row=0, column=3)
        ttk.Button(btns, text="Reset Stack", command=self._reset_stack_index).grid(row=0, column=4, padx=(6, 0))
        ttk.Label(btns, textvariable=self._stack_index_var).grid(row=0, column=5, padx=(10, 0), sticky="w")

        learn = ttk.LabelFrame(tab, text="Self-Learning (TNT)")
        learn.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 6))
        ttk.Checkbutton(
            learn,
            text="Enable",
            variable=self._learning_enabled_var,
            command=self._on_learning_toggle,
        ).grid(row=0, column=0, padx=6, pady=4, sticky="w")
        ttk.Label(learn, text="Mode:").grid(row=0, column=1, padx=(6, 2), pady=4, sticky="w")
        mode = ttk.Combobox(learn, textvariable=self._learning_mode_var, values=["shadow", "active"], width=8, state="readonly")
        mode.grid(row=0, column=2, padx=2, pady=4, sticky="w")
        mode.bind("<<ComboboxSelected>>", lambda _e: self._on_learning_mode_change())
        ttk.Button(learn, text="Reset Policy", command=self._reset_learning_policy).grid(row=0, column=3, padx=6, pady=4)
        ttk.Button(learn, text="Show Status", command=self._show_learning_status).grid(row=0, column=4, padx=6, pady=4)
        ttk.Label(learn, textvariable=self._learning_status_var).grid(row=0, column=5, padx=6, pady=4, sticky="w")
        learn.columnconfigure(5, weight=1)

        log_frame = ttk.LabelFrame(tab, text="Pick & Place Log")
        log_frame.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self._log_text = tk.Text(log_frame, height=12, wrap="word")
        self._log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self._log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._log_text.configure(yscrollcommand=scrollbar.set)

        tab.rowconfigure(2, weight=1)

    def _build_sim_tab(self, tab):
        tab.columnconfigure(1, weight=1)
        row = ttk.Frame(tab)
        row.grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Label(row, text="Cycles:").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(row, textvariable=self._sim_cycles, width=8).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(row, text="Run Simulation", command=self._run_simulation).grid(row=0, column=2)

    def _build_cfg_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        self._cfg_text = tk.Text(tab, height=18, wrap="none")
        self._cfg_text.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

    def _build_calib_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        tabs = ttk.Notebook(tab)
        tabs.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        base_tab = ttk.Frame(tabs)
        camera_tab = ttk.Frame(tabs)
        marker_tab = ttk.Frame(tabs)
        detect_tab = ttk.Frame(tabs)
        tabs.add(base_tab, text="Base-Cam")
        tabs.add(camera_tab, text="Camera")
        tabs.add(marker_tab, text="Marker-Obj")
        tabs.add(detect_tab, text="Perception")

        self._build_base_cam_tab(base_tab)
        self._build_camera_tab(camera_tab)
        self._build_marker_tab(marker_tab)
        self._build_detect_tab(detect_tab)

    def _build_base_cam_tab(self, tab):
        info = ttk.Label(tab, text="Compute base_T_cam using a chessboard and a known base_T_board.")
        info.pack(anchor="w", padx=6, pady=(6, 2))

        params = ttk.Frame(tab)
        params.pack(anchor="w", padx=6, pady=4)
        ttk.Label(params, text="Pattern cols:").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._pattern_cols_var, width=6).grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(params, text="Pattern rows:").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._pattern_rows_var, width=6).grid(row=0, column=3, padx=4, pady=2)
        ttk.Label(params, text="Square size (mm):").grid(row=0, column=4, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._square_size_var, width=8).grid(row=0, column=5, padx=4, pady=2)

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Load Config", command=self._load_calibration_into_ui).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save Board Config", command=self._save_board_config).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Compute base_T_cam", command=self._compute_base_T_cam).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save base_T_cam", command=self._save_base_T_cam).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Validate base_T_cam", command=self._validate_base_T_cam_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Simulate calibration", command=self._simulate_base_cam_noise).pack(side=tk.LEFT, padx=4)

        grid = ttk.Frame(tab)
        grid.pack(fill="x", padx=6, pady=6)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        board_frame, self._base_T_board_vars = self._make_matrix_grid(grid, "base_T_board", readonly=False)
        cam_frame, self._base_T_cam_vars = self._make_matrix_grid(grid, "base_T_cam", readonly=True)
        board_frame.grid(row=0, column=0, sticky="n", padx=4)
        cam_frame.grid(row=0, column=1, sticky="n", padx=4)

        capture = ttk.LabelFrame(tab, text="Board -> Base (Robot TCP)")
        capture.pack(fill="x", padx=6, pady=6)
        ttk.Label(
            capture,
            text=(
                "Capture three inner corners: P0=(0,0), P1=(cols-1,0), P2=(0,rows-1). "
                "Use the same corner order as the camera view."
            ),
        ).pack(anchor="w", padx=6, pady=(4, 2))

        btn_row = ttk.Frame(capture)
        btn_row.pack(anchor="w", padx=6, pady=2)
        state = "normal" if self._execute_app is not None else "disabled"
        ttk.Button(btn_row, text="Capture P0 (0,0)", command=lambda: self._capture_board_point("p0"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Capture P1 (X+)", command=lambda: self._capture_board_point("p1"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Capture P2 (Y+)", command=lambda: self._capture_board_point("p2"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Compute base_T_board", command=self._compute_base_T_board_from_points).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Clear Points", command=self._clear_board_points).pack(side=tk.LEFT, padx=4)

        points = ttk.Frame(capture)
        points.pack(anchor="w", padx=6, pady=(2, 6))
        self._board_point_vars = {
            "p0": tk.StringVar(value="not set"),
            "p1": tk.StringVar(value="not set"),
            "p2": tk.StringVar(value="not set"),
        }
        ttk.Label(points, text="P0:").grid(row=0, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p0"]).grid(row=0, column=1, sticky="w")
        ttk.Label(points, text="P1:").grid(row=1, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p1"]).grid(row=1, column=1, sticky="w")
        ttk.Label(points, text="P2:").grid(row=2, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p2"]).grid(row=2, column=1, sticky="w")

    def _build_camera_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        info = ttk.Label(tab, text="Calibrate camera intrinsics using the same chessboard pattern.")
        info.pack(anchor="w", padx=6, pady=(6, 2))

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Capture Sample", command=self._capture_camera_sample).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Compute Intrinsics", command=self._compute_camera_intrinsics).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save to camera.json", command=self._save_camera_intrinsics).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Clear Samples", command=self._reset_camera_calib).pack(side=tk.LEFT, padx=4)

        ttk.Label(tab, textvariable=self._cam_calib_status).pack(anchor="w", padx=6, pady=(2, 2))
        ttk.Label(tab, textvariable=self._cam_calib_result_var, font=("Consolas", 9), justify="left").pack(
            anchor="w", padx=6, pady=(2, 6)
        )

    def _get_tcp_position_mm(self):
        if self._execute_app is None or not hasattr(self._execute_app, "get_current_tcp_mm"):
            raise RuntimeError("Robot TCP not available")
        tcp = self._execute_app.get_current_tcp_mm()
        return [
            float(tcp.get("X_mm", 0.0)),
            float(tcp.get("Y_mm", 0.0)),
            float(tcp.get("Z_mm", 0.0)),
        ]

    def _capture_board_point(self, key: str):
        try:
            point = self._get_tcp_position_mm()
            self._board_points[key] = point
            if key in self._board_point_vars:
                self._board_point_vars[key].set(f"{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}")
            self._log(f"Board point {key} captured at {point}.")
        except Exception as exc:
            self._log(f"Capture board point failed: {exc}")

    def _clear_board_points(self):
        self._board_points = {}
        for var in self._board_point_vars.values():
            var.set("not set")
        self._log("Board points cleared.")

    def _compute_base_T_board_from_points(self):
        try:
            p0 = self._board_points.get("p0")
            p1 = self._board_points.get("p1")
            p2 = self._board_points.get("p2")
            if not (p0 and p1 and p2):
                raise RuntimeError("Capture P0, P1, and P2 first.")
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            if cols < 2 or rows < 2:
                raise RuntimeError("Pattern size must be at least 2x2.")
            x_len = (cols - 1) * square
            y_len = (rows - 1) * square

            vx = [p1[i] - p0[i] for i in range(3)]
            vy = [p2[i] - p0[i] for i in range(3)]
            x_axis = normalize_vector(vx)
            y_axis = normalize_vector(vy)
            if x_axis is None or y_axis is None:
                raise RuntimeError("Board points are too close or invalid.")
            z_axis = normalize_vector(cross(x_axis, y_axis))
            if z_axis is None:
                raise RuntimeError("Board points are collinear.")
            y_axis = normalize_vector(cross(z_axis, x_axis))
            if y_axis is None:
                raise RuntimeError("Failed to orthogonalize board axes.")

            R = [
                [x_axis[0], y_axis[0], z_axis[0]],
                [x_axis[1], y_axis[1], z_axis[1]],
                [x_axis[2], y_axis[2], z_axis[2]],
            ]
            base_T_board = make_transform(R, p0)
            self._set_matrix_vars(self._base_T_board_vars, base_T_board)

            pred_p1 = [p0[i] + x_axis[i] * x_len for i in range(3)]
            pred_p2 = [p0[i] + y_axis[i] * y_len for i in range(3)]
            err1 = math.sqrt(sum((pred_p1[i] - p1[i]) ** 2 for i in range(3)))
            err2 = math.sqrt(sum((pred_p2[i] - p2[i]) ** 2 for i in range(3)))
            self._log(f"base_T_board computed. err_x={err1:.2f} mm err_y={err2:.2f} mm")
        except Exception as exc:
            self._log(f"Compute base_T_board failed: {exc}")

    def _reset_camera_calib(self):
        self._cam_calib_objpoints = []
        self._cam_calib_imgpoints = []
        self._cam_calib_image_size = None
        self._cam_calib_pattern = None
        self._cam_calib_square = None
        self._cam_calib_result = None
        self._cam_calib_status.set("No samples")
        self._cam_calib_result_var.set("")
        self._log("Camera calibration samples cleared.")

    def _capture_camera_sample(self):
        try:
            self._ensure_configs()
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            if cols < 2 or rows < 2:
                raise RuntimeError("Pattern size must be at least 2x2.")
            if self._cam_calib_pattern and self._cam_calib_pattern != (cols, rows):
                self._reset_camera_calib()
            if self._cam_calib_square and abs(self._cam_calib_square - square) > 1e-6:
                self._reset_camera_calib()

            cam_cfg = self._configs.get("camera", {})
            image, color_order = self._get_calibration_image(cam_cfg)
            if image is None:
                raise RuntimeError("No camera frame available")

            import cv2
            import numpy as np

            if len(image.shape) == 3:
                if color_order == "rgb":
                    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                else:
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            try:
                flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
                found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
            except Exception:
                found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if not found:
                raise RuntimeError("Chessboard not found")

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objp = np.zeros((rows * cols, 3), np.float32)
            objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
            objp *= square

            self._cam_calib_pattern = (cols, rows)
            self._cam_calib_square = square
            self._cam_calib_objpoints.append(objp)
            self._cam_calib_imgpoints.append(corners2)
            self._cam_calib_image_size = (gray.shape[1], gray.shape[0])
            self._cam_calib_status.set(f"Samples: {len(self._cam_calib_objpoints)}")
            self._log("Camera calibration sample captured.")
        except Exception as exc:
            self._log(f"Capture sample failed: {exc}")

    def _compute_camera_intrinsics(self):
        try:
            if len(self._cam_calib_objpoints) < 5:
                raise RuntimeError("Capture at least 5 samples first.")
            if not self._cam_calib_image_size:
                raise RuntimeError("Missing image size.")

            import cv2

            rms, camera_matrix, dist, _, _ = cv2.calibrateCamera(
                self._cam_calib_objpoints,
                self._cam_calib_imgpoints,
                self._cam_calib_image_size,
                None,
                None,
            )
            fx = float(camera_matrix[0][0])
            fy = float(camera_matrix[1][1])
            cx = float(camera_matrix[0][2])
            cy = float(camera_matrix[1][2])
            dist_list = [float(x) for x in dist.reshape(-1).tolist()]

            self._cam_calib_result = {
                "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                "distortion": dist_list,
                "image_size": list(self._cam_calib_image_size),
                "rms": float(rms),
            }
            self._cam_calib_status.set(f"RMS: {rms:.4f}  Samples: {len(self._cam_calib_objpoints)}")
            self._cam_calib_result_var.set(
                f"fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}\n"
                f"dist={dist_list}"
            )
            self._log("Camera intrinsics computed.")
        except Exception as exc:
            self._log(f"Compute intrinsics failed: {exc}")

    def _save_camera_intrinsics(self):
        try:
            self._ensure_configs()
            if not self._cam_calib_result:
                raise RuntimeError("No intrinsics computed.")
            cam_cfg = self._configs.get("camera", {})
            cam_cfg["intrinsics"] = dict(self._cam_calib_result["intrinsics"])
            cam_cfg["distortion"] = list(self._cam_calib_result["distortion"])
            cam_cfg["image_size"] = list(self._cam_calib_result["image_size"])
            self._write_config("camera", cam_cfg)
            self._log("Camera intrinsics saved. Re-init the pipeline to apply.")
        except Exception as exc:
            self._log(f"Save intrinsics failed: {exc}")

    def _build_marker_tab(self, tab):
        info = ttk.Label(tab, text="Set marker_T_obj (marker to cube center).")
        info.pack(anchor="w", padx=6, pady=(6, 2))

        params = ttk.Frame(tab)
        params.pack(anchor="w", padx=6, pady=4)
        ttk.Label(params, text="Face axis:").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Combobox(params, textvariable=self._marker_axis_var, values=["x", "y", "z"], width=4, state="readonly").grid(
            row=0, column=1, padx=4, pady=2
        )
        ttk.Label(params, text="Sign:").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Combobox(params, textvariable=self._marker_sign_var, values=[-1, 1], width=4, state="readonly").grid(
            row=0, column=3, padx=4, pady=2
        )

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Load marker_T_obj", command=self._load_calibration_into_ui).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Compute marker_T_obj", command=self._compute_marker_T_obj).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save marker_T_obj", command=self._save_marker_T_obj).pack(side=tk.LEFT, padx=4)

        frame, self._marker_T_obj_vars = self._make_matrix_grid(tab, "marker_T_obj", readonly=False)
        frame.pack(anchor="w", padx=6, pady=6)

    def _build_detect_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        info = ttk.Label(tab, text="Run a single detection and show the object pose.")
        info.pack(anchor="w", padx=6, pady=(6, 2))
        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Detect Once", command=self._detect_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Pick up (Test)", command=self._pickup_test).pack(side=tk.LEFT, padx=4)
        ttk.Label(tab, textvariable=self._detect_var, font=("Consolas", 9)).pack(anchor="w", padx=6, pady=4)

        filt = ttk.LabelFrame(tab, text="TNT Filter")
        filt.pack(fill="x", padx=6, pady=6)
        ttk.Label(
            filt,
            text=(
                "H1/H2: Rot-Hue-Bereiche (0-179). "
                "S min/V min: Saettigung/Helligkeit Mindestwerte. "
                "Min area: kleinste Flaeche in Pixeln. "
                "Kernel: Morphologie-Groesse (ungerade Zahl)."
            ),
            wraplength=520,
        ).pack(anchor="w", padx=6, pady=(4, 2))

        def _slider_row(parent, label, var, from_, to_, step=1):
            row = ttk.Frame(parent)
            row.pack(fill="x", padx=6, pady=2)
            ttk.Label(row, text=label, width=14).pack(side=tk.LEFT)
            scale = ttk.Scale(
                row,
                from_=from_,
                to=to_,
                variable=var,
                orient=tk.HORIZONTAL,
                length=200,
                value=var.get(),
                command=lambda _v, v=var, s=step: v.set(int(round(float(_v) / s) * s)),
            )
            scale.pack(side=tk.LEFT, padx=4)
            scale.bind("<ButtonRelease-1>", lambda _e: self._apply_tnt_filters())
            entry = ttk.Entry(row, textvariable=var, width=6, justify="right")
            entry.pack(side=tk.LEFT)
            entry.bind("<Return>", lambda _e: self._apply_tnt_filters())

        _slider_row(filt, "H1 low", self._tnt_h1_low_var, 0, 179)
        _slider_row(filt, "H1 high", self._tnt_h1_high_var, 0, 179)
        _slider_row(filt, "H2 low", self._tnt_h2_low_var, 0, 179)
        _slider_row(filt, "H2 high", self._tnt_h2_high_var, 0, 179)
        _slider_row(filt, "S min", self._tnt_s_min_var, 0, 255)
        _slider_row(filt, "V min", self._tnt_v_min_var, 0, 255)
        _slider_row(filt, "Min area", self._tnt_min_area_var, 0, 5000)
        _slider_row(filt, "Kernel", self._tnt_kernel_var, 3, 15, step=2)

        filt_btns = ttk.Frame(filt)
        filt_btns.pack(anchor="w", padx=6, pady=(4, 6))
        ttk.Button(filt_btns, text="Apply Filters", command=self._apply_tnt_filters).pack(side=tk.LEFT, padx=4)
        ttk.Button(filt_btns, text="Save Filters", command=self._save_tnt_filters).pack(side=tk.LEFT, padx=4)

    def _build_help_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        text = (
            "Setup + Calibration (v6.3) - Step by step\n"
            "\n"
            "A) Basic setup\n"
            "1) Connect the camera and start it in the Vision tab.\n"
            "2) Prepare the chessboard: flat, known square size (mm), board fixed.\n"
            "3) Pattern size (cols/rows) = number of INNER corners.\n"
            "4) In Pick&Place -> Calibration, verify all pattern parameters.\n"
            "\n"
            "B) Camera intrinsics (once per camera)\n"
            "5) Open Calibration -> Camera.\n"
            "6) Capture 10+ samples with varied board poses.\n"
            "7) Click Compute Intrinsics.\n"
            "8) Click Save to camera.json.\n"
            "9) Re-init pipeline (Init Pipeline) or restart app.\n"
            "\n"
            "C) base_T_board (Robot -> Board)\n"
            "10) Open Calibration -> Base-Cam.\n"
            "11) Set pattern cols/rows and square size (mm).\n"
            "12) Move TCP to P0=(0,0) inner corner, Capture P0.\n"
            "13) Move TCP to P1=(cols-1,0) inner corner, Capture P1.\n"
            "14) Move TCP to P2=(0,rows-1) inner corner, Capture P2.\n"
            "15) Compute base_T_board, then Save Board Config.\n"
            "    Note: P1 defines +X, P2 defines +Y. Order must match camera view.\n"
            "\n"
            "D) base_T_cam (Camera -> Base)\n"
            "16) In Vision tab, keep board visible and sharp.\n"
            "17) Click Compute base_T_cam.\n"
            "18) Run Validate base_T_cam (target: <=3 mm and <=1.5 deg).\n"
            "19) Run Simulate calibration (Monte Carlo, keep p95 low).\n"
            "20) Verify matrix and Save base_T_cam (configs/transforms.json).\n"
            "\n"
            "E) marker_T_obj (Marker -> Object center)\n"
            "21) Open Calibration -> Marker-Obj.\n"
            "22) Choose face axis and sign according to marker orientation.\n"
            "23) Compute marker_T_obj and Save marker_T_obj (configs/markers.json).\n"
            "\n"
            "F) Perception test\n"
            "24) Open Calibration -> Perception.\n"
            "25) Run Detect Once and verify position/quaternion/confidence.\n"
            "\n"
            "Troubleshooting\n"
            "- Board not found: check lighting, focus, pattern cols/rows, square size.\n"
            "- Large errors: recalibrate intrinsics.\n"
            "- Rotated pose: verify P0/P1/P2 order matches camera order.\n"
            "- Wrong base_T_cam: board moved or base_T_board is inaccurate.\n"
            "\n"
            "Sketch (board calibration, world coordinates)\n"
            "P0 = (0,0)  P1 = (cols-1,0)  P2 = (0,rows-1)\n"
            "\n"
            "   +Y (rows)\n"
            "   ^\n"
            "   |                R0 (Robot Base)\n"
            "   |                o\n"
            "   |                |\n"
            "   |<---- 25 cm ----|\n"
            "   |\n"
            "   |   P2 o-----------o\n"
            "   |      |           |\n"
            "   |      |   Board   |\n"
            "   |      |           |\n"
            "   |   P0 o-----------o P1  -> +X (cols)\n"
            "   +------------------------------>\n"
            "          World XY plane (Z up)\n"
            "          25 cm distance to side edge (X direction)\n"
        )
        help_text = tk.Text(tab, wrap="word")
        help_text.insert("1.0", text)
        help_text.configure(state="disabled")
        help_text.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

    def _make_matrix_grid(self, parent, title, readonly=False):
        frame = ttk.LabelFrame(parent, text=title)
        vars_grid = []
        for r in range(4):
            row_vars = []
            for c in range(4):
                var = tk.StringVar(value="0.0")
                ent = ttk.Entry(frame, textvariable=var, width=9, justify="right")
                if readonly:
                    ent.configure(state="readonly")
                ent.grid(row=r, column=c, padx=2, pady=2)
                row_vars.append(var)
            vars_grid.append(row_vars)
        return frame, vars_grid

    def _set_matrix_vars(self, vars_grid, matrix):
        if vars_grid is None or matrix is None:
            return
        for r in range(4):
            for c in range(4):
                try:
                    val = float(matrix[r][c])
                except Exception:
                    val = 0.0
                vars_grid[r][c].set(f"{val:.6f}")

    def _read_matrix_vars(self, vars_grid):
        matrix = []
        for r in range(4):
            row = []
            for c in range(4):
                raw = vars_grid[r][c].get().replace(",", ".")
                row.append(float(raw))
            matrix.append(row)
        return matrix

    def _identity_matrix(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _ensure_configs(self):
        if self._configs is None:
            self._configs = config_loader.load_all_configs(self._base_dir)
            self._render_config()

    def _write_config(self, key, data):
        rel_path = config_loader.CONFIG_FILES.get(key)
        if not rel_path:
            raise RuntimeError(f"Unknown config key: {key}")
        path = os.path.join(self._base_dir, rel_path)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self._configs[key] = data
        self._render_config()

    def _load_calibration_into_ui(self):
        try:
            self._ensure_configs()
            calib = self._configs.get("calibration", {})
            pattern = calib.get("board_pattern", {})
            size = pattern.get("pattern_size", [7, 7])
            if len(size) >= 2:
                self._pattern_cols_var.set(int(size[0]))
                self._pattern_rows_var.set(int(size[1]))
            self._square_size_var.set(float(pattern.get("square_size_mm", 30.0)))
            base_T_board = calib.get("base_T_board", self._identity_matrix())
            transforms = self._configs.get("transforms", {})
            base_T_cam = transforms.get("base_T_cam", self._identity_matrix())
            markers = self._configs.get("markers", {})
            marker_T_obj = markers.get("marker_T_obj", self._identity_matrix())
            self._set_matrix_vars(self._base_T_board_vars, base_T_board)
            self._set_matrix_vars(self._base_T_cam_vars, base_T_cam)
            self._set_matrix_vars(self._marker_T_obj_vars, marker_T_obj)
            self._load_tnt_into_ui()
            self._log("Calibration config loaded.")
        except Exception as exc:
            self._log(f"Calibration load failed: {exc}")

    def _save_board_config(self):
        try:
            self._ensure_configs()
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            base_T_board = self._read_matrix_vars(self._base_T_board_vars)
            calib = self._configs.get("calibration", {})
            calib["board_pattern"] = {"pattern_size": [cols, rows], "square_size_mm": square}
            calib["base_T_board"] = base_T_board
            self._write_config("calibration", calib)
            self._log("Calibration board config saved.")
        except Exception as exc:
            self._log(f"Save board config failed: {exc}")

    def _compute_base_T_cam(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board = self._read_matrix_vars(self._base_T_board_vars)
                cam_cfg = self._configs.get("camera", {})
                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)
                objp, corners2 = self._detect_board_points(cols, rows, square, cam_cfg)

                import cv2
                ok, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist)
                if not ok:
                    raise RuntimeError("solvePnP failed")

                proj, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist)
                reproj = proj.reshape(-1, 2) - corners2.reshape(-1, 2)
                reproj_rmse = float((reproj[:, 0] ** 2 + reproj[:, 1] ** 2).mean() ** 0.5)

                R_cam, _ = cv2.Rodrigues(rvec)
                cam_T_board = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
                base_T_cam = matmul(base_T_board, invert_transform(cam_T_board))

                self._set_matrix_vars(self._base_T_cam_vars, base_T_cam)
                self._set_status("base_T_cam computed")
                self._log(f"base_T_cam computed from chessboard. reproj_rmse={reproj_rmse:.3f} px")
            except Exception as exc:
                self._set_status("base_T_cam error")
                self._log(f"Compute base_T_cam failed: {exc}")
        self._start_worker(_task)

    @staticmethod
    def _rot_error_deg(R_a, R_b):
        t = (
            R_a[0][0] * R_b[0][0] + R_a[1][0] * R_b[1][0] + R_a[2][0] * R_b[2][0]
            + R_a[0][1] * R_b[0][1] + R_a[1][1] * R_b[1][1] + R_a[2][1] * R_b[2][1]
            + R_a[0][2] * R_b[0][2] + R_a[1][2] * R_b[1][2] + R_a[2][2] * R_b[2][2]
        )
        cos_theta = max(-1.0, min(1.0, (t - 1.0) * 0.5))
        return math.degrees(math.acos(cos_theta))

    def _camera_model_from_cfg(self, cam_cfg):
        import numpy as np

        intr = cam_cfg.get("intrinsics", {})
        fx = float(intr.get("fx", 800.0))
        fy = float(intr.get("fy", 800.0))
        image_size = cam_cfg.get("image_size", [640, 480])
        w = float(image_size[0]) if isinstance(image_size, list) and len(image_size) >= 2 else 640.0
        h = float(image_size[1]) if isinstance(image_size, list) and len(image_size) >= 2 else 480.0
        cx = float(intr.get("cx", w / 2.0))
        cy = float(intr.get("cy", h / 2.0))
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array(cam_cfg.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float32)
        return camera_matrix, dist

    def _detect_board_points(self, cols, rows, square, cam_cfg):
        image, color_order = self._get_calibration_image(cam_cfg)
        if image is None:
            raise RuntimeError("No camera frame available")

        import cv2
        import numpy as np

        if len(image.shape) == 3:
            if color_order == "rgb":
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            else:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        try:
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
        except Exception:
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            raise RuntimeError("Chessboard not found")

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp *= float(square)
        return objp, corners2

    def _validate_base_T_cam_once(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board_cfg = self._read_matrix_vars(self._base_T_board_vars)
                base_T_cam_cfg = self._read_matrix_vars(self._base_T_cam_vars)
                cam_cfg = self._configs.get("camera", {})
                objp, corners2 = self._detect_board_points(cols, rows, square, cam_cfg)
                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)

                import cv2
                ok, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist)
                if not ok:
                    raise RuntimeError("solvePnP failed")
                R_cam, _ = cv2.Rodrigues(rvec)
                cam_T_board_meas = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
                base_T_board_meas = matmul(base_T_cam_cfg, cam_T_board_meas)

                t_cfg = extract_translation(base_T_board_cfg)
                t_meas = extract_translation(base_T_board_meas)
                pos_err = math.sqrt(sum((t_cfg[i] - t_meas[i]) ** 2 for i in range(3)))

                R_cfg = [row[:3] for row in base_T_board_cfg[:3]]
                R_meas = [row[:3] for row in base_T_board_meas[:3]]
                ang_err = self._rot_error_deg(R_cfg, R_meas)
                status = "OK" if pos_err <= 3.0 and ang_err <= 1.5 else "WARN"
                self._log(
                    f"base_T_cam validation ({status}): board_pos_err={pos_err:.2f} mm "
                    f"board_rot_err={ang_err:.2f} deg"
                )
            except Exception as exc:
                self._log(f"Validate base_T_cam failed: {exc}")

        self._start_worker(_task)

    def _simulate_base_cam_noise(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board = self._read_matrix_vars(self._base_T_board_vars)
                base_T_cam = self._read_matrix_vars(self._base_T_cam_vars)
                cam_cfg = self._configs.get("camera", {})

                import cv2
                import numpy as np

                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)
                cam_T_board = matmul(invert_transform(base_T_cam), base_T_board)
                R = np.array([row[:3] for row in cam_T_board[:3]], dtype=np.float32)
                t = np.array(extract_translation(cam_T_board), dtype=np.float32).reshape(3, 1)
                rvec, _ = cv2.Rodrigues(R)

                objp = np.zeros((rows * cols, 3), np.float32)
                objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
                objp *= float(square)
                ideal, _ = cv2.projectPoints(objp, rvec, t, camera_matrix, dist)
                ideal = ideal.reshape(-1, 2)

                rng = np.random.default_rng(6300)
                sigma_px = 0.35
                trials = 200
                trans_err = []
                rot_err = []
                for _ in range(trials):
                    noisy = ideal + rng.normal(0.0, sigma_px, ideal.shape)
                    noisy = noisy.astype(np.float32).reshape(-1, 1, 2)
                    ok, rvec_n, tvec_n = cv2.solvePnP(objp, noisy, camera_matrix, dist)
                    if not ok:
                        continue
                    R_n, _ = cv2.Rodrigues(rvec_n)
                    cam_T_board_n = make_transform(R_n.tolist(), tvec_n.reshape(-1).tolist())
                    base_T_cam_n = matmul(base_T_board, invert_transform(cam_T_board_n))

                    t_true = extract_translation(base_T_cam)
                    t_est = extract_translation(base_T_cam_n)
                    trans_err.append(math.sqrt(sum((t_true[i] - t_est[i]) ** 2 for i in range(3))))

                    R_true = [row[:3] for row in base_T_cam[:3]]
                    R_est = [row[:3] for row in base_T_cam_n[:3]]
                    rot_err.append(self._rot_error_deg(R_true, R_est))

                if not trans_err:
                    raise RuntimeError("No valid Monte-Carlo trials.")

                trans_arr = np.array(trans_err, dtype=float)
                rot_arr = np.array(rot_err, dtype=float)
                self._log(
                    "Simulation base_T_cam (200x, noise=0.35 px): "
                    f"t_err mean={trans_arr.mean():.2f} mm p95={np.percentile(trans_arr,95):.2f} mm, "
                    f"R_err mean={rot_arr.mean():.2f} deg p95={np.percentile(rot_arr,95):.2f} deg"
                )
            except Exception as exc:
                self._log(f"Simulate calibration failed: {exc}")

        self._start_worker(_task)

    def _save_base_T_cam(self):
        try:
            self._ensure_configs()
            base_T_cam = self._read_matrix_vars(self._base_T_cam_vars)
            transforms = self._configs.get("transforms", {})
            transforms["base_T_cam"] = base_T_cam
            self._write_config("transforms", transforms)
            if self._pipeline is not None:
                self._pipeline.context.perception.base_T_cam = base_T_cam
            self._log("base_T_cam saved.")
        except Exception as exc:
            self._log(f"Save base_T_cam failed: {exc}")

    def _compute_marker_T_obj(self):
        try:
            self._ensure_configs()
            cube_cfg = self._configs.get("cube", {})
            edge = float(cube_cfg.get("edge_length_mm", 50.0))
            axis = (self._marker_axis_var.get() or "z").lower()
            sign = int(self._marker_sign_var.get())
            normal = [0.0, 0.0, 0.0]
            if axis == "x":
                normal[0] = float(sign)
            elif axis == "y":
                normal[1] = float(sign)
            else:
                normal[2] = float(sign)

            t = [-normal[0] * edge / 2.0, -normal[1] * edge / 2.0, -normal[2] * edge / 2.0]
            R = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
            marker_T_obj = make_transform(R, t)
            self._set_matrix_vars(self._marker_T_obj_vars, marker_T_obj)
            self._log("marker_T_obj computed (rotation assumes marker axes align with object axes).")
        except Exception as exc:
            self._log(f"Compute marker_T_obj failed: {exc}")

    def _save_marker_T_obj(self):
        try:
            self._ensure_configs()
            marker_T_obj = self._read_matrix_vars(self._marker_T_obj_vars)
            markers = self._configs.get("markers", {})
            markers["marker_T_obj"] = marker_T_obj
            self._write_config("markers", markers)
            if self._pipeline is not None:
                self._pipeline.context.perception.marker_T_obj = marker_T_obj
            self._log("marker_T_obj saved.")
        except Exception as exc:
            self._log(f"Save marker_T_obj failed: {exc}")

    def _detect_once(self):
        def _task():
            try:
                self._ensure_pipeline()
                self._attach_camera_capture()
                pose = self._pipeline.context.perception.detect_object_pose()
                msg = (
                    f"pos={pose.position_mm} quat={pose.quaternion_xyzw} "
                    f"conf={pose.confidence:.3f}"
                )
                self._detect_var.set(msg)
                self._log(f"Detect ok: {msg}")
            except Exception as exc:
                self._detect_var.set(f"Detect failed: {exc}")
                self._log(f"Detect failed: {exc}")
        self._start_worker(_task)

    def _load_tnt_into_ui(self):
        try:
            tnt_cfg = self._configs.get("tnt", {})
            ranges = tnt_cfg.get("hsv_red_ranges", [])
            if len(ranges) >= 1 and len(ranges[0]) >= 6:
                r1 = ranges[0]
                self._tnt_h1_low_var.set(int(r1[0]))
                self._tnt_s_min_var.set(int(r1[1]))
                self._tnt_v_min_var.set(int(r1[2]))
                self._tnt_h1_high_var.set(int(r1[3]))
            if len(ranges) >= 2 and len(ranges[1]) >= 6:
                r2 = ranges[1]
                self._tnt_h2_low_var.set(int(r2[0]))
                self._tnt_h2_high_var.set(int(r2[3]))
            self._tnt_min_area_var.set(int(tnt_cfg.get("min_area_px", 500)))
            self._tnt_kernel_var.set(int(tnt_cfg.get("morph_kernel", 3)))
        except Exception as exc:
            self._log(f"TNT load failed: {exc}")

    def _build_tnt_config(self):
        h1_low = int(self._tnt_h1_low_var.get())
        h1_high = int(self._tnt_h1_high_var.get())
        h2_low = int(self._tnt_h2_low_var.get())
        h2_high = int(self._tnt_h2_high_var.get())
        s_min = int(self._tnt_s_min_var.get())
        v_min = int(self._tnt_v_min_var.get())
        min_area = int(self._tnt_min_area_var.get())
        kernel = int(self._tnt_kernel_var.get())
        if kernel % 2 == 0:
            kernel += 1
        tnt_cfg = dict(self._configs.get("tnt", {}))
        tnt_cfg["hsv_red_ranges"] = [
            [h1_low, s_min, v_min, h1_high, 255, 255],
            [h2_low, s_min, v_min, h2_high, 255, 255],
        ]
        tnt_cfg["min_area_px"] = min_area
        tnt_cfg["morph_kernel"] = kernel
        return tnt_cfg

    def _apply_tnt_filters(self):
        try:
            self._ensure_configs()
            tnt_cfg = self._build_tnt_config()
            self._configs["tnt"] = tnt_cfg
            if self._pipeline is not None:
                self._pipeline.context.perception.tnt_cfg = tnt_cfg
            if self._execute_app is not None and hasattr(self._execute_app, "board_detector"):
                try:
                    self._execute_app.board_detector.set_tnt_cfg(tnt_cfg)
                except Exception:
                    pass
            self._log("TNT filters applied.")
        except Exception as exc:
            self._log(f"Apply TNT filters failed: {exc}")

    def _save_tnt_filters(self):
        try:
            self._ensure_configs()
            tnt_cfg = self._build_tnt_config()
            self._write_config("tnt", tnt_cfg)
            if self._pipeline is not None:
                self._pipeline.context.perception.tnt_cfg = tnt_cfg
            if self._execute_app is not None and hasattr(self._execute_app, "board_detector"):
                try:
                    self._execute_app.board_detector.set_tnt_cfg(tnt_cfg)
                except Exception:
                    pass
            self._log("TNT filters saved.")
        except Exception as exc:
            self._log(f"Save TNT filters failed: {exc}")

    def _rpy_from_R(self, R):
        sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
        singular = sy < 1e-6
        if not singular:
            roll = math.degrees(math.atan2(R[2][1], R[2][2]))
            pitch = math.degrees(math.atan2(-R[2][0], sy))
            yaw = math.degrees(math.atan2(R[1][0], R[0][0]))
        else:
            roll = math.degrees(math.atan2(-R[1][2], R[1][1]))
            pitch = math.degrees(math.atan2(-R[2][0], sy))
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _wrap_angle_deg(angle):
        return ((angle + 180.0) % 360.0) - 180.0

    def _rpy_to_R(self, roll_deg, pitch_deg, yaw_deg):
        yaw_r = math.radians(yaw_deg)
        pitch_r = math.radians(pitch_deg)
        roll_r = math.radians(roll_deg)
        cy, sy = math.cos(yaw_r), math.sin(yaw_r)
        cp, sp = math.cos(pitch_r), math.sin(pitch_r)
        cr, sr = math.cos(roll_r), math.sin(roll_r)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]

    def _current_rpy(self):
        tcp = self._execute_app.get_current_tcp_mm()
        roll = float(tcp.get("Roll_deg", 0.0))
        pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
        yaw = float(tcp.get("Yaw_deg", 0.0))
        R = self._rpy_to_R(roll, pitch, yaw)
        # If tool Z points up, flip orientation to point down.
        if R[2][2] > 0.0:
            roll = self._wrap_angle_deg(roll + 180.0)
        return roll, pitch, yaw

    def _get_speed_slider_feed(self, fallback: float) -> float:
        if self._execute_app is None:
            return fallback
        try:
            speed_val = getattr(self._execute_app, "speed_val", None)
            max_feed = getattr(self._execute_app, "max_feed", None)
            if speed_val is None or max_feed is None:
                return fallback
            val = float(speed_val.get())
            max_f = float(max_feed.get())
            feed = (val / 1000.0) * max_f
            msf = getattr(self._execute_app, "manual_speed_factor", None)
            if msf is not None:
                try:
                    feed *= float(msf.get())
                except Exception:
                    pass
            if feed <= 0.0:
                return fallback
            return feed
        except Exception:
            return fallback

    def _wait_for_idle(self, timeout_s=30.0):
        if self._execute_app is None or not hasattr(self._execute_app, "client"):
            return True
        start = time.time()
        while time.time() - start < timeout_s:
            try:
                st = getattr(self._execute_app.client, "last_status", None) or {}
                state = str(st.get("state", "")).lower()
                if "idle" in state or state == "":
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        return False

    def _pickup_test(self):
        def _task():
            try:
                if self._execute_app is None or not hasattr(self._execute_app, "kinematics_tabs"):
                    raise RuntimeError("Kinematics UI not available")
                self._ensure_pipeline()
                self._attach_camera_capture()

                pose = self._pipeline.context.perception.detect_object_pose()
                cube_cfg = self._configs.get("cube", {})
                obj_pos = extract_translation(pose.matrix)
                grasp_offset = float(cube_cfg.get("grasp_offset_mm", 0.0))
                approach_dist = float(cube_cfg.get("approach_distance_mm", 0.0))
                lift_dist = float(cube_cfg.get("lift_distance_mm", 0.0))
                normal = [0.0, 0.0, 1.0]  # always approach vertically from above
                grasp_pos = [
                    obj_pos[0] + normal[0] * grasp_offset,
                    obj_pos[1] + normal[1] * grasp_offset,
                    obj_pos[2] + normal[2] * grasp_offset,
                ]
                approach_pos = [
                    grasp_pos[0] + normal[0] * approach_dist,
                    grasp_pos[1] + normal[1] * approach_dist,
                    grasp_pos[2] + normal[2] * approach_dist,
                ]
                lift_pos = [
                    grasp_pos[0],
                    grasp_pos[1],
                    grasp_pos[2] + lift_dist,
                ]

                robot_cfg = self._configs.get("robot", {})
                base_feed = self._get_speed_slider_feed(float(robot_cfg.get("approach_speed", 60.0)))
                approach_feed = base_feed
                grasp_feed = base_feed
                lift_feed = base_feed

                def _move_pos_split(pos, fast_feed, slow_feed):
                    roll, pitch, yaw = self._current_rpy()
                    tcp = self._execute_app.get_current_tcp_mm()
                    start_pos = [
                        float(tcp.get("X_mm", 0.0)),
                        float(tcp.get("Y_mm", 0.0)),
                        float(tcp.get("Z_mm", 0.0)),
                    ]
                    mid = [
                        start_pos[0] + 0.9 * (pos[0] - start_pos[0]),
                        start_pos[1] + 0.9 * (pos[1] - start_pos[1]),
                        start_pos[2] + 0.9 * (pos[2] - start_pos[2]),
                    ]
                    ok = self._execute_app.kinematics_tabs.move_tcp_pose(
                        mid[0],
                        mid[1],
                        mid[2],
                        roll,
                        pitch,
                        yaw,
                        feed=fast_feed,
                    )
                    if not ok:
                        return False
                    self._wait_for_idle()
                    return self._execute_app.kinematics_tabs.move_tcp_pose(
                        pos[0],
                        pos[1],
                        pos[2],
                        roll,
                        pitch,
                        yaw,
                        feed=slow_feed,
                    )

                self._execute_app.send_now("M3 S0")
                slow_feed = 300.0
                if not _move_pos_split(approach_pos, approach_feed, slow_feed):
                    raise RuntimeError("Approach move failed")
                self._wait_for_idle()
                if not _move_pos_split(grasp_pos, grasp_feed, slow_feed):
                    raise RuntimeError("Grasp move failed")
                self._execute_app.send_now("M3 S1000")
                self._wait_for_idle()
                if not _move_pos_split(lift_pos, lift_feed, slow_feed):
                    raise RuntimeError("Lift move failed")
                self._log("Pick up test complete.")
            except Exception as exc:
                self._log(f"Pick up test failed: {exc}")
        self._start_worker(_task)

    def _ui(self, fn):
        self.after(0, fn)

    def _log(self, message: str):
        if self._logger:
            try:
                self._logger(message)
            except Exception:
                pass
        def _write():
            self._log_text.insert(tk.END, message + "\n")
            self._log_text.see(tk.END)
        self._ui(_write)

    def _set_status(self, text: str):
        self._ui(lambda: self._status_var.set(text))

    def _refresh_stack_index(self):
        idx = "n/a"
        try:
            if self._pipeline is not None:
                idx = str(int(getattr(self._pipeline.context, "stack_index", 0)))
        except Exception:
            idx = "n/a"
        self._ui(lambda: self._stack_index_var.set(f"Stack index: {idx}"))

    def _reset_stack_index(self):
        try:
            self._ensure_pipeline()
            self._pipeline.context.stack_index = 0
            self._refresh_stack_index()
            self._log("Stack index reset to 0.")
        except Exception as exc:
            self._log(f"Stack index reset failed: {exc}")

    def _init_pipeline(self):
        try:
            self._pipeline, self._configs = build_pipeline(self._base_dir)
            self._attach_camera_capture()
            lr = self._ensure_learner()
            lr.sync_baseline_from_configs(self._configs)
            self._refresh_learning_from_manager()
            sim_cfg = self._configs.get("simulation", {})
            cycles = int(sim_cfg.get("cycles", 1000))
            self._sim_cycles.set(cycles)
            self._render_config()
            self._load_calibration_into_ui()
            self._refresh_stack_index()
            self._set_status("Pipeline ready")
            self._log("Pipeline initialized.")
        except Exception as exc:
            self._set_status("Init failed")
            self._log(f"Init failed: {exc}")

    def _ensure_pipeline(self):
        if self._pipeline is None:
            self._init_pipeline()
        if self._pipeline is None:
            raise RuntimeError("Pipeline not initialized")
        self._attach_camera_capture()

    def _attach_camera_capture(self):
        if self._pipeline is None:
            return
        if self._camera_capture is None:
            return
        try:
            self._pipeline.context.perception.camera = _CameraCaptureAdapter(self._camera_capture)
        except Exception:
            pass

    def _ensure_learner(self):
        if self._learner is None:
            self._learner = TntSelfLearningManager(self._base_dir, logger=self._log)
        return self._learner

    def _refresh_learning_from_manager(self):
        lr = self._ensure_learner()
        st = lr.get_status()
        self._learning_enabled_var.set(bool(st.get("enabled", False)))
        self._learning_mode_var.set(str(st.get("mode", "shadow")))
        self._learning_status_var.set(
            f"Learning: {st.get('mode', 'shadow')} "
            f"eps={st.get('epsilon', 0.0):.3f} "
            f"succ={st.get('successes', 0)}/{st.get('attempts', 0)}"
        )

    def _on_learning_toggle(self):
        lr = self._ensure_learner()
        lr.set_enabled(bool(self._learning_enabled_var.get()))
        lr.save_settings()
        self._refresh_learning_from_manager()
        self._log(f"Learning enabled set to {bool(self._learning_enabled_var.get())}.")

    def _on_learning_mode_change(self):
        lr = self._ensure_learner()
        lr.set_mode(self._learning_mode_var.get())
        lr.save_settings()
        self._refresh_learning_from_manager()
        self._log(f"Learning mode set to {self._learning_mode_var.get()}.")

    def _show_learning_status(self):
        lr = self._ensure_learner()
        st = lr.get_status()
        self._log(
            "Learning status: "
            f"mode={st['mode']} enabled={st['enabled']} epsilon={st['epsilon']:.3f} "
            f"success_rate={st['success_rate']:.2%} attempts={st['attempts']} "
            f"failure_streak={st.get('failure_streak', 0)} last_reward={st.get('last_reward', 0.0):+.2f} "
            f"context={st.get('active_context', 'global')}"
        )

    def _reset_learning_policy(self):
        try:
            self._ensure_configs()
            lr = self._ensure_learner()
            lr.reset_policy_from_configs(self._configs)
            self._refresh_learning_from_manager()
            self._log("Learning policy reset from current config.")
        except Exception as exc:
            self._log(f"Learning reset failed: {exc}")

    def _learning_before_cycle(self, simulated: bool):
        if self._configs is None:
            return
        lr = self._ensure_learner()
        info = lr.before_cycle(self._configs, simulated=simulated, context_hint=self._last_cycle_ctx)
        self._learning_event_count += 1
        should_log = (not simulated) or (self._learning_event_count % 50 == 0)
        if info.get("applied"):
            if should_log:
                self._log(
                f"Learning candidate applied (mode={info.get('mode')}, explore={info.get('explore')})."
                )
        elif info.get("enabled"):
            if should_log:
                self._log(
                f"Learning candidate evaluated in shadow mode (explore={info.get('explore')})."
                )

    def _make_cycle_context(self, duration_s: float, err):
        ctx = {"duration_s": float(duration_s)}
        try:
            obj_pose = getattr(self._pipeline.context, "object_pose", None)
            if obj_pose is not None and hasattr(obj_pose, "confidence"):
                ctx["confidence"] = float(obj_pose.confidence)
        except Exception:
            pass
        if err is not None:
            ctx["error_type"] = type(err).__name__
        return ctx

    def _learning_after_cycle(self, success: bool, err, cycle_ctx=None):
        lr = self._ensure_learner()
        msg = "" if err is None else str(err)
        ctx = cycle_ctx or {}
        out = lr.after_cycle(bool(success), msg, cycle_ctx=ctx)
        self._last_cycle_ctx = dict(ctx)
        if out.get("updated"):
            st = lr.get_status()
            suffix = ""
            if out.get("rolled_back"):
                suffix += " rollback"
            if out.get("mode_changed"):
                suffix += f" mode={st.get('mode')}"
            self._learning_status_var.set(
                f"Learning: {st.get('mode')} "
                f"eps={out.get('epsilon', 0.0):.3f} "
                f"reward={out.get('reward', 0.0):+.2f}"
                f" fs={st.get('failure_streak', 0)}"
                f" ctx={st.get('active_context', 'global')}{suffix}"
            )

    def _start_worker(self, target):
        if self._worker and self._worker.is_alive():
            self._log("Worker already running.")
            return
        self._stop_event.clear()
        self._worker = threading.Thread(target=target, daemon=True)
        self._worker.start()

    def _run_one_cycle(self):
        def _task():
            try:
                self._ensure_pipeline()
                self._learning_before_cycle(simulated=False)
                t0 = time.time()
                ok = self._pipeline.run_cycle()
                dt = time.time() - t0
                err = self._pipeline.state_machine.last_error
                self._learning_after_cycle(ok, err, cycle_ctx=self._make_cycle_context(dt, err))
                self._refresh_stack_index()
                if ok:
                    self._set_status("Cycle ok")
                    self._log("Cycle success.")
                else:
                    self._set_status("Cycle failed")
                    self._log(f"Cycle failed: {err}")
            except Exception as exc:
                self._set_status("Cycle error")
                self._log(f"Cycle error: {exc}")
        self._start_worker(_task)

    def _run_n_cycles(self):
        def _task():
            try:
                self._ensure_pipeline()
                total = max(1, int(self._run_count.get()))
                successes = 0
                for i in range(total):
                    if self._stop_event.is_set():
                        self._log("Run stopped by user.")
                        break
                    self._learning_before_cycle(simulated=False)
                    t0 = time.time()
                    ok = self._pipeline.run_cycle()
                    dt = time.time() - t0
                    err = self._pipeline.state_machine.last_error
                    self._learning_after_cycle(ok, err, cycle_ctx=self._make_cycle_context(dt, err))
                    self._refresh_stack_index()
                    if not ok:
                        self._set_status("Cycle failed")
                        self._log(f"Cycle {i + 1} failed: {err}")
                        break
                    successes += 1
                    self._set_status(f"Success {successes}/{total}")
                self._log(f"Run complete: {successes}/{total} successes.")
            except Exception as exc:
                self._set_status("Run error")
                self._log(f"Run error: {exc}")
        self._start_worker(_task)

    def _run_simulation(self):
        def _task():
            try:
                self._ensure_pipeline()
                cycles = max(1, int(self._sim_cycles.get()))
                runner = SimulationRunner(self._pipeline, self._configs.get("perception", {}))
                def _before(_i):
                    self._learning_before_cycle(simulated=True)

                def _cb(_i, ok, err, meta):
                    self._learning_after_cycle(ok, err, cycle_ctx=meta or {})

                successes = runner.run(cycles, per_cycle=_cb, before_cycle=_before)
                self._refresh_stack_index()
                self._set_status(f"Simulation {successes}/{cycles}")
                self._log(f"Simulation complete: {successes}/{cycles} successes.")
            except Exception as exc:
                self._set_status("Simulation error")
                self._log(f"Simulation error: {exc}")
        self._start_worker(_task)

    def _stop(self):
        self._stop_event.set()
        if self._pipeline is not None:
            try:
                self._pipeline.context.safe_stop()
            except Exception:
                pass
        self._set_status("Stopped")
        self._log("Stop requested.")

    def _render_config(self):
        if self._configs is None:
            return
        payload = json.dumps(self._configs, indent=2, sort_keys=True)
        self._cfg_text.delete("1.0", tk.END)
        self._cfg_text.insert(tk.END, payload)

    def _get_calibration_image(self, cam_cfg):
        if self._camera_capture is not None:
            image = self._camera_capture.get_latest_frame()
            if image is not None:
                return image, "rgb"
        camera = None
        if self._pipeline is not None:
            camera = self._pipeline.context.perception.camera
        if camera is None:
            from perception.camera import build_camera
            camera = build_camera(cam_cfg)
        frame = camera.get_frame()
        return frame.image, "bgr"
