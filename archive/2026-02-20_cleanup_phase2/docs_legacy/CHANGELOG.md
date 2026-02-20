# Changelog

## [6.3] - 2026-02-19

### Added
- New launcher entrypoint `Robotrol_FluidNC_v6_3.py`.
- New build spec `Robotrol_FluidNC_v6_3.spec`.
- New calibration documentation `CALIBRATION_GUIDE_v6_3.md`.
- New project language flag config `configs/project_flags.json`.
- New language migration tracker `LANGUAGE_MIGRATION_EN.md`.
- New language check helper `tools/check_english_text.py`.
- New concise calibration manual `CALIBRATION_INSTRUCTIONS_v6_3.md`.
- New smoke-check helper `tools/smoke_checks.py` (language strict + mock simulation).
- New CI workflow `.github/workflows/ci.yml`.
- New TNT self-learning module `learning/tnt_self_learning.py`.
- New self-learning concept document `SELF_LEARNING_TNT_CONCEPT.md`.
- New learning configs:
  - `configs/tnt_learning.json`
  - `configs/tnt_learning_policy.json`
- Base-Cam calibration tools in `pickplace_ui.py`:
  - `Validate base_T_cam` (single-shot consistency check),
  - `Simulate calibration` (Monte Carlo sensitivity check with pixel noise).

### Changed
- App window title updated to `Robotrol V6.3`.
- `Pick&Place -> Calibration -> Base-Cam -> Compute base_T_cam` now logs reprojection RMSE.
- Integrated help text updated from v6 to v6.3 workflow with quality thresholds.
- README updated to v6.3 launcher and calibration guide reference.
- First pass translation of key runtime/user strings to English in main app/gamepad/calibration help.
- Second pass translation of remaining runtime strings in serial/connect logs, visualizer start messages, updater dialogs/status, and camera selector label.
- Third pass translation sweep for Python sources:
  - translated remaining German comments/docstrings in main modules
  - translated remaining gamepad/camera UI labels
  - kept behavior unchanged (text-only updates)
- `ANLEITUNG_KALIBRIERUNG_v6_3.md` converted to compatibility redirect; active concise doc is `CALIBRATION_INSTRUCTIONS_v6_3.md`.
- README now documents strict language-check mode for CI (`--strict`).
- `pickplace_ui.py` now integrates a v1 self-learning control panel (enable/mode/reset/status) and per-cycle learning hooks.
- `simulation/simulation_loop.py` now supports optional `before_cycle` and `per_cycle` callbacks.
- `control/config.py` now loads `tnt_learning` config.
- Self-learning upgraded to v2:
  - context-aware reward (confidence + duration + error penalty)
  - automatic rollback to stable parameters after failure streak
  - optional mode fallback (default to shadow on rollback)
- `tools/smoke_checks.py` now validates learner v2 rollback behavior.
- Self-learning v3 step:
  - context buckets by confidence band (`low/mid/high/unknown`) and phase (`sim/real`)
  - per-context best-parameter memory and reward tracking
  - UI status now shows active learning context

### Notes
- Existing core file remains `Robotrol_FluidNC_v6_2.py` for compatibility.
- v6.3 is exposed via launcher/spec/title and calibration workflow updates.
