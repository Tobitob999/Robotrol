# RoboControl v5.7 - Working Status (Waypoint)

Purpose:
This file captures the current state so the next session can resume here.

Project layout:
- Base folder: d:\VSCode\robocontrol_latest\robocontrol_clean\robocontrol_v5_7
- Entry script: RoboControl_FluidNC_v5_7.py
- Kinematics: tcp_world_kinematics_frame.py
- TCP pose panel: tcp_pose_module_v3.py
- Gamepad: gamepad_block_v3.py

Current behavior summary:
- Profiles (Moveo/Red15) store endstops, manual_limits, DH model, gamepad, cam_to_base.
- Manual sliders use profile-based manual_limits; endstops remain independent.
- Fixed TCP shows live target G-code with endstop warning and auto execute-on-release.
- TCP Point orientation aligns tool Z to target while keeping a stable X axis from current TCP.
- Kinematics preview highlights out-of-range values using profile endstops.

Known issues / focus areas:
- Validate Fixed TCP point-mode orientation on hardware.
- Confirm Red15 manual_limits/endstops once robot ranges are known.
- Verify endstop highlighting with negative ranges on real controller.

Recent changes (latest to older):
1) RoboControl_FluidNC_v5_7.py
   - Manual slider limits are profile-based (manual_limits).
   - Fixed TCP: live target G-code + endstop status; execute-on-release auto.
   - TCP Point orientation stabilized to use tool direction + current X axis.
   - TCP Pose area: Send to Queue + Gripper Open/Close queue buttons.
   - Homing/Control buttons flattened; TCP Pose title hidden.
2) tcp_world_kinematics_frame.py
   - DH table refresh on profile change.
   - Added preview_tcp_gcode for Fixed TCP preview.
   - Limit checks skip HW limits when endstops allow negative ranges.
3) Red15.json / Moveo.json
   - Red15 profile created with DH parameters from image.
   - Moveo profile gained manual_limits section.

How to start v5.7:
- Start in background: Start-Process -FilePath python -ArgumentList 'RoboControl_FluidNC_v5_7.py'
  - Working dir: d:\VSCode\robocontrol_latest\robocontrol_clean\robocontrol_v5_7

Session notes (next start):
- If Red15 still shows Moveo DH, verify profile dropdown persists on Red15 and DH table refresh runs.
- Update Red15 manual_limits/endstops once real travel limits are known.
