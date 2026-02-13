# RoboControl v5.7 - Next Session Status

Purpose:
This note captures the handoff state before testing.

Project layout:
- Base folder: d:\VSCode\robocontrol_latest\robocontrol_clean\robocontrol_v5_7
- Entry script: RoboControl_FluidNC_v5_7.py
- Profile config: Moveo.json (active) + Red15.json (configured)

Current behavior summary:
- Configs are consolidated into profile files (endstops, dh_model, gamepad, cam_to_base).
- Profile selector in the header next to Theme (Moveo/Red15).
- Endstops tab writes into profile; kinematics preview uses endstop ranges for red highlighting.
- DH save writes into profile; TCP pose reads geometry from active profile.
- Gamepad config and cam_to_base save into profile.

Known issues / focus areas:
- Verify kinematics limit highlighting aligns with profile endstops for negative ranges.
- Validate profile switching updates endstops, DH, and manual slider ranges cleanly.
- Confirm cam_to_base saves into Moveo.json (no legacy file writes).
- Fill Red15 manual_limits/endstops once actual ranges are known.

How to start:
- Start in background: Start-Process -FilePath python -ArgumentList 'RoboControl_FluidNC_v5_7.py'
  - Working dir: d:\VSCode\robocontrol_latest\robocontrol_clean\robocontrol_v5_7
