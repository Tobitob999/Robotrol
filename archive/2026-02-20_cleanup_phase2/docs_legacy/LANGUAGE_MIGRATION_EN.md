# English Language Migration (Low Priority)

Project policy:
- `configs/project_flags.json`
  - `language = "en"`
  - `enforce_english_text = true`
  - `language_migration_policy = "low_priority_incremental"`

## Scope
Convert all user-facing text to English over time, while keeping functionality unchanged.

## Priority Buckets
1. High
- Runtime warnings/errors shown during operation.
- Safety-relevant prompts and status lines.

2. Medium
- Tab labels, button captions, help text, calibration flows.

3. Low
- Legacy comments, internal debug strings, old modules not in active workflow.

## Current Progress
- English calibration docs added:
  - `CALIBRATION_INSTRUCTIONS_v6_3.md` (concise)
  - `CALIBRATION_GUIDE_v6_3.md`
  - `ANLEITUNG_KALIBRIERUNG_v6_3.md` kept as compatibility redirect file
- Main app title switched to v6.3 and core zeroing/status messages translated in:
  - `Robotrol_FluidNC_v6_2.py`
  - `gamepad_block_v3.py`
- Pick&Place calibration help text translated to English:
  - `pickplace_ui.py`
- Board pose UI/calibration user-facing strings translated:
  - `board_pose_v1.py`
- OTA updater dialogs/status strings translated:
  - `fluidnc_updater_v2.py`
- Camera selector label translated:
  - `camera_capturev_v1_1.py`
- Visualizer runtime status strings translated:
  - `robosim_visualizer_v90.py`
- Additional comment/docstring cleanup translated in:
  - `Robotrol_FluidNC_v6_2.py`
  - `camera_capturev_v1_1.py`
  - `gamepad_block_v3.py`
  - `fluidnc_updater_v2.py`
  - `ik_rotosim.py`
  - `tcp_pose_module_v3.py`
  - `tcp_world_kinematics_frame.py`
  - `board_pose_v1.py`

## Remaining Known Areas
- Legacy Markdown/docs and external notes may still contain German text.
- Continue opportunistic cleanup when touching additional modules/files.

## Working Rule
Whenever a file is touched for functional changes, translate nearby user-facing strings in the same patch when safe.

## Automation
- Language check: `python tools/check_english_text.py --strict`
- Smoke checks: `python tools/smoke_checks.py`
- CI workflow runs compile + smoke checks on push/PR.
