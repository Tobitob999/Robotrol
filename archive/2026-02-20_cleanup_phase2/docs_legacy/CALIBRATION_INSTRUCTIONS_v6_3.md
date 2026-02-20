# Calibration Instructions v6.3

This is the concise English calibration workflow for reliable chess piece pick/place.

## 1) Preparation
1. Fix the chessboard rigidly to the table.
2. Keep lighting stable and avoid reflections.
3. Set camera focus so all inner corners are sharp.
4. In `Pick&Place -> Calibration`:
   - `pattern_size` = number of inner corners (cols, rows),
   - `square_size_mm` = real square edge length.

## 2) Camera Intrinsics (once per camera/lens setup)
Path: `Pick&Place -> Calibration -> Camera`

1. Capture 10-20 samples with diverse viewpoints.
2. Click `Compute Intrinsics`.
3. Accept quality only if RMS reprojection error is low:
   - good: `< 0.5 px`
   - acceptable: `0.5..0.8 px`
   - recalibrate if `> 0.8 px`
4. Save via `Save to camera.json`.
5. Re-init pipeline or restart app.

## 3) Compute `base_T_board` (robot base -> board)
Path: `Pick&Place -> Calibration -> Base-Cam`

Capture three board points with TCP:
- `P0 = (0,0)` inner corner
- `P1 = (cols-1,0)` inner corner
- `P2 = (0,rows-1)` inner corner

Steps:
1. Move TCP to P0 -> `Capture P0`
2. Move TCP to P1 -> `Capture P1`
3. Move TCP to P2 -> `Capture P2`
4. Click `Compute base_T_board`
5. Check residuals (`err_x`, `err_y`):
   - target `< 1.5 mm`
   - acceptable `< 3.0 mm`
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
