# Acceptance Tests

## Unit
- Optimizer returns theta within clamps
- Store writes trials and reads best theta

## Integration (no robot motion)
- Edge starts and detects ArUco board
- Edge can call Agent `/v1/learn/next` with PSK

## Hardware
- Dry-run safe-Z move without GRBL alarm
- Pick/place on one square shows expected occupancy delta
