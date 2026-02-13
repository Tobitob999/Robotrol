# Kinematics Contract (MOVEo 6DOF)

## Coordinate and units
- Units: meters and radians in API inputs/outputs.
- Pose: position (x,y,z) in meters and orientation as rotation matrix or RPY in radians.

## Required functions
- fk(q) -> pose
  - q: list/array of 6 joint angles in the order A, X, Y, Z, B, C (rad)
  - pose: {p: [x,y,z], R: 3x3} or equivalent
- jacobian(q) -> 6x6
  - consistent with fk(q)
- ik(target_pose, seed, opts) -> solution or list of solutions
  - seed-biased selection
  - joint limits enforced
  - damping near singularities (DLS/LM)

## Error metrics
- Position error: norm of delta position (m)
- Orientation error: angle of rotation error (rad) via log-map or quaternion angle

## Legacy compatibility
- The GUI mirrors X after FK for legacy display coordinates.
  This is not part of the DH chain and should be documented in tests.
