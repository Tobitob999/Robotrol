# Deprecated Filename

This file is kept for compatibility only.

Use `CALIBRATION_INSTRUCTIONS_v6_3.md` (concise) and `CALIBRATION_GUIDE_v6_3.md` (full guide).
6. Click `Save Board Config`

Important:
- P1 defines board +X.
- P2 defines board +Y.
- Wrong point order causes rotated/flipped board coordinates.

## 4) Compute and validate `base_T_cam` (camera -> robot base)
Path: `Pick&Place -> Calibration -> Base-Cam`

1. Ensure board is visible and sharp.
2. Click `Compute base_T_cam`.
3. Check logged `reproj_rmse`:
   - target `< 0.5 px`
   - acceptable `< 0.8 px`
4. Click `Validate base_T_cam`:
   - expected `board_pos_err <= 3.0 mm`
   - expected `board_rot_err <= 1.5 deg`
5. Click `Simulate calibration` (Monte Carlo):
   - p95 translation/rotation errors should stay low.
6. Click `Save base_T_cam`.

## 5) Optional: `marker_T_obj`
Path: `Pick&Place -> Calibration -> Marker-Obj`

1. Set marker face axis/sign according to physical mounting.
2. Click `Compute marker_T_obj`.
3. Click `Save marker_T_obj`.

## 6) Final validation before live grasping
Path: `Pick&Place -> Calibration -> Perception`

1. Run `Detect Once` multiple times (5-10).
2. Verify stable pose and confidence.
3. Run dry motion above the board (safe Z).
4. Start real grasp/place only after dry-run passes.

## 7) Typical failure patterns
- Board not detected reliably:
  - fix lighting/focus, verify pattern size and square size.
- Systematic XY offset:
  - wrong square size, wrong P0/P1/P2, stale config files.
- Calibration looks fine but grasp is off:
  - verify TCP/tool offset and `marker_T_obj`.

## 8) Relevant files
- `configs/camera.json`
- `configs/calibration.json`
- `configs/transforms.json`
- `configs/markers.json`
- `CALIBRATION_GUIDE_v6_3.md` (extended guide)
