# Hardware Validation Checklist (TCP/Sequence)

## Preconditions
- Robot homed or zeroed consistently.
- Correct profile loaded (`Moveo` / `EB15_red` / `EB300`).
- `Fixed TCP` mode configured as intended.

## Test A: No Pre-Move Before Sequence
1. Set a stable start pose.
2. Trigger sequence via gamepad buttons `6/7`.
3. Verify first commanded move does not drift away from expected start.
4. Repeat 10 times.
5. Pass criterion: no visible unsolicited pre-move in any run.

## Test B: Direction Consistency
1. Jog in +X/+Y/+Z via UI/gamepad.
2. Confirm real robot direction matches TCP panel direction.
3. Repeat in at least two orientations.
4. Pass criterion: no sign flips or 90-degree axis swaps.

## Test C: Sequence Reliability
1. Execute full sequence repeatedly (>=10 cycles).
2. Monitor controller for alarms/fault state.
3. Pass criterion: no controller error state during/after cycle.

## Test D: Restart Stability
1. Start `Robotrol_FluidNC_v6_3.py`.
2. Run one short sequence.
3. Stop app, restart, repeat.
4. Pass criterion: behavior remains identical across restarts.

## Logging
- Record profile, timestamp, and result for each cycle.
- If failure occurs, capture:
  - active profile,
  - current TCP pose,
  - queued first command,
  - controller alarm/error text.
