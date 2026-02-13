# Validation Plan (MOVEo 6DOF)

## FK consistency
- Random q within limits
- Compare DH FK vs ground truth chain
- Tolerance: position <= 1e-4 m, orientation <= 1e-4 rad

## IK round-trip
- Random q_true within limits
- T = fk(q_true)
- q_sol = ik(T, seed=q_true)
- Assert fk(q_sol) ~= T within tolerance
- Optionally check joint closeness modulo 2pi

## Edge cases
- Near singular wrist configurations
- Near joint limits
- Multiple-solution targets (wrist flip)

## Traceability checks
- Each DH row has non-empty source_note
- DH convention documented
- Offsets listed for every joint

## Failure class coverage
- DH vs MDH mismatch
- Missing +/- 90 deg offsets
- a vs d swapped
- wrong axis sign or joint order
- TCP applied in wrong frame
- inconsistent joint zero
