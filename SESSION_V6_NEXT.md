# Robotrol v6.1 - Session Snapshot

Status summary:
- Vision: TNT cube detection working; overlay shown in Vision tab. Debug images saved when enabled.
- Pick&Place: "Pick up (Test)" uses vertical approach (+Z), current TCP orientation (keeps gripper parallel to table), and speed slider feed.
- Motion: Each leg runs 90% at slider speed, last 10% at F300.
- UI: "Vision rechts" toggle shows the Vision overlay image to the right of the Speed column.
- UI: TNT filter sliders (H1/H2, S min, V min, Min area, Kernel) apply on slider release or Enter; Save writes configs/tnt.json.
- UI: TCP Pose buttons renamed to "Pose to Queue" and "Sequence to Queue".
- Help: Calibration help expanded with ASCII sketch and robot base offset.

How to start:
- Run: `python robocontrol_v6_0/Robotrol_FluidNC_v6_1.py`

Key UI locations:
- Vision tab: camera + chessboard overlay; toggle "Vision rechts" to mirror overlay in right panel.
- Pick&Place -> Calibration -> Perception: TNT filter sliders + Apply/Save, Detect Once, Pick up (Test).
- Pick&Place -> Help: full setup/calibration guide + board sketch.

Config notes:
- `configs/tnt.json`: widened HSV ranges, min_area reduced, morph kernel set to 3, debug_save true.
- Debug images: `data/tnt_input.png`, `data/tnt_mask.png`, `data/tnt_contours.png`.

Files touched (v6):
- `Robotrol_FluidNC_v6_1.py`: Vision rechts toggle + view panel, TCP Pose button labels.
- `board_pose_v1.py`: stores last overlay image, TNT contour overlay, set_tnt_cfg.
- `pickplace_ui.py`: calibration help, TNT filters UI, pick up test logic, speed split, current TCP orientation.
- `perception/tnt_detector.py`: contour helper + debug capture.
- `configs/tnt.json`: HSV/min_area tuning + debug paths.
- `TEST_PROCEDURE_V5_7.md`: calibration procedure added.

Open checks / next steps:
- Verify pick-up target position is centered on the cube (tune TNT filters if outline partial).
- Confirm base_T_board orientation matches camera corner order (P0/P1/P2).
- If Vision rechts does not update, confirm camera is running and board detector is active.

---

Session note (2026-02-02):
- Added EB300 DH profile (EB300.json) with DH: d1=95mm, a2=348mm, a3=318mm, d4=122mm, d5=99mm, d6=57mm; alpha1=-90deg, alpha4=-90deg, alpha5=+90deg (rad in file). Axis order A/X/Y/B/Z/C, mirror_x=true.
- config_profiles DEFAULT_PROFILE set to EB300; profile list includes EB300.
- Robotrol header and window title updated to v6.1.
- PyInstaller onefile target: dist\Robotrol_FluidNC_v6_1.exe (when rebuilt).
- Bundle target: dist\Robotrol_FluidNC_v6_1_bundle (exe + profiles + model\ + robosim_visualizer_v90.exe + _internal).
