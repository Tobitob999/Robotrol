# Calibration Guide v6.3

This procedure establishes the minimum prerequisites for reliable chess piece pick/place:
- stable camera intrinsics,
- stable `base_T_board`,
- stable `base_T_cam`,
- validated marker/object transform.

All values are in mm / deg unless noted.

## 1. Preconditions
1. Fix the chessboard rigidly on the table (no movement between calibration and operation).
2. Ensure constant lighting and sharp focus over the full board.
3. Verify pattern settings:
   - `pattern_size = number of inner corners` (cols, rows)
   - `square_size_mm = measured square edge length`
4. Robot must report plausible TCP values (no drifting offsets).

## 2. Camera Intrinsics (one-time per camera/lens)
Use `Pick&Place -> Calibration -> Camera`.

1. Capture at least 10 samples with strong pose diversity:
   - center, corners, near/far, tilt left/right, tilt up/down.
2. Click `Compute Intrinsics`.
3. Accept only if RMS reprojection error is low and stable:
   - good: `< 0.5 px`
   - acceptable: `0.5..0.8 px`
   - repeat calibration if `> 0.8 px`
4. Save with `Save to camera.json`.
5. Re-init pipeline or restart app.

## 3. base_T_board (robot to board frame)
Use `Pick&Place -> Calibration -> Base-Cam`.

Capture 3 physical board points with TCP:
- `P0 = (0,0)` inner corner
- `P1 = (cols-1, 0)` inner corner
- `P2 = (0, rows-1)` inner corner

Then:
1. `Capture P0`, `Capture P1`, `Capture P2`
2. `Compute base_T_board`
3. Check logged residuals `err_x`, `err_y`:
   - target `< 1.5 mm`, acceptable `< 3.0 mm`
4. `Save Board Config`

Important:
- P1 defines +X of board.
- P2 defines +Y of board.
- Wrong point order rotates/inverts board frame.

## 4. base_T_cam (camera to robot base)
Use `Pick&Place -> Calibration -> Base-Cam`.

1. Keep board fully visible.
2. Click `Compute base_T_cam`.
3. Check logged `reproj_rmse` (PnP):
   - target `< 0.5 px`, acceptable `< 0.8 px`
4. Click `Validate base_T_cam`:
   - expected: `board_pos_err <= 3.0 mm`
   - expected: `board_rot_err <= 1.5 deg`
5. Click `Simulate calibration`:
   - Monte Carlo (pixel noise) should show small p95 errors.
   - if p95 translation/rotation is too high, improve image quality and repeat.
6. Save with `Save base_T_cam`.

## 5. marker_T_obj (if marker-based object pose is used)
Use `Pick&Place -> Calibration -> Marker-Obj`.

1. Set marker face axis/sign according to physical mounting.
2. `Compute marker_T_obj`
3. `Save marker_T_obj`

## 6. End Validation (must pass before gripping)
Use `Pick&Place -> Calibration -> Perception`:

1. `Detect Once` repeatedly (5-10 times) without moving setup.
2. Position jitter should be low and confidence consistently high.
3. Execute dry-run pick/place above board (safe Z) before live gripping.

## 7. Failure Patterns and Fixes
- Chessboard intermittently not found:
  - reduce glare, increase contrast, refocus, verify pattern size.
- Good intrinsics but bad base pose:
  - board moved after calibration,
  - wrong P0/P1/P2 ordering,
  - unstable TCP reference.
- Systematic XY offset:
  - wrong square size,
  - old calibration files still loaded.
- Good calibration, poor grasping:
  - check gripper TCP offset and marker/object transform.
