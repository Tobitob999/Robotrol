# RoboControl v5.7 - Test Procedure

Scope:
Manual validation of profile-based configuration, endstops, and kinematics preview.

Preconditions:
- Start app from `robocontrol_v5_7` with `RoboControl_FluidNC_v5_7.py`.
- Profile selector shows `Moveo` and `Red15`.

Procedure:
1) Startup + profile selector
   - Launch app and confirm header shows profile dropdown next to Theme.
   - Ensure default is `Moveo`.

2) Endstops UI + highlight
   - Open Endstops tab and confirm min/max values match Moveo.json.
   - In Kinematics tab, generate preview with a value within limits -> expect green.
   - Set a value outside limits -> expect red background for that axis value.
   - Test negative values within limits -> should stay green.

3) Endstops apply/save
   - In Endstops tab, change a min value and press Enter.
   - Confirm sliders update their ranges.
   - Generate preview; check highlight uses new limits.
   - Reopen Endstops tab and confirm values persist in Moveo.json.

4) DH save into profile
   - In Kinematics tab, adjust a DH parameter and click Save DH.
   - Confirm Moveo.json `dh_model` updated (no change to model/dh.json expected).
   - Verify TCP Pose values still update without error.

5) Gamepad config save into profile
   - Open Gamepad tab, change a value, click Save.
   - Confirm Moveo.json `gamepad` updated.

6) cam_to_base save into profile
   - In Calibration tab, save cam_to_base.
   - Confirm Moveo.json `cam_to_base` updated.

7) Profile switch (Moveo -> Red15)
   - Select Red15 in profile dropdown.
   - Expected: DH table updates to Red15 values and manual slider ranges refresh.

Calibration procedure (v6):
Preconditions:
- Start app from `robocontrol_v6_0` with `RoboControl_FluidNC_v6_0.py`.
- Chessboard with known square size (mm), board fixed on the table.
- Camera running in Vision tab and robot TCP available.

Steps:
1) Calibrate camera intrinsics (once per camera)
   - Open Pick&Place -> Calibration -> Camera.
   - Capture 10+ samples with different board poses (tilt and position).
   - Click Compute Intrinsics, then Save to camera.json.
   - Re-init pipeline or restart the app to reload camera config.

2) Calibrate base_T_board (robot -> board)
   - Open Pick&Place -> Calibration -> Base-Cam.
   - Set pattern cols/rows and square size (mm).
   - Move TCP to inner corner P0=(0,0), P1=(cols-1,0), P2=(0,rows-1).
   - Capture P0/P1/P2 in that order, then Compute base_T_board.
   - Save Board Config.
   - Note: P1 defines +X and P2 defines +Y; keep the same corner order as the camera view.

3) Calibrate base_T_cam (camera -> base)
   - Ensure the chessboard is visible in the Vision tab.
   - Click Compute base_T_cam and verify the matrix updates.
   - Save base_T_cam (writes configs/transforms.json).

4) Calibrate marker_T_obj (marker -> object center)
   - Open Pick&Place -> Calibration -> Marker-Obj.
   - Choose the cube face axis and sign where the marker is attached.
   - Compute marker_T_obj and Save (writes configs/markers.json).

5) Validate perception
   - Open Pick&Place -> Calibration -> Perception.
   - Click Detect Once and check position/quaternion/confidence.
   - If unstable, re-check intrinsics, lighting, and base_T_cam.

Pass criteria:
- All steps execute without crashes.
- Kinematics preview red/green matches endstop ranges, including negatives.
- Moveo.json updates for endstops, DH, gamepad, cam_to_base.
