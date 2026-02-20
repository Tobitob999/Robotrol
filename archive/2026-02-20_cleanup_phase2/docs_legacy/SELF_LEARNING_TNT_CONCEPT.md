# Self-Learning TNT Pick&Place Concept (v1)

## Objective
Improve Minecraft TNT cube pick-and-place success over time by safely adapting perception, grasp, and motion parameters.

## Scope
- Keep existing pipeline (`perception -> planning -> execution`).
- Learn bounded parameter updates, not full policy replacement.
- Support two modes:
  - `shadow`: evaluate candidates, do not apply.
  - `active`: apply bounded candidates before each cycle.

## Learnable Parameters
- `cube`: `grasp_offset_mm`, `approach_distance_mm`, `lift_distance_mm`
- `robot`: `approach_speed`, `grasp_speed`, `gripper.close_force`
- `perception`: `min_confidence`
- `tnt`: `min_area_px`

## Safety
- Hard parameter bounds from config.
- Exploration decay and minimum epsilon.
- Default startup in `shadow` mode.
- No online DH/kinematics modification.

## Learning Loop
1. Build candidate params from current policy.
2. Explore with probability `epsilon` (Gaussian perturbation within bounds).
3. Optionally apply candidate (`active` mode).
4. Execute cycle and compute reward from context:
   - success/failure
   - confidence
   - cycle duration
   - error presence
5. Persist stats, epsilon, and logs.
6. Accept new best policy when reward improves in `active` mode.

## Reward (v2)
- Success: positive reward
- Failure: negative reward
- Simulation reward scale: reduced weight
- Confidence bonus: rewards stable high-confidence detections
- Time penalty: penalizes cycles above a target duration
- Error penalty: penalizes runtime errors

## Rollback Safety (v2)
- Track consecutive failures (`failure_streak`).
- If failure streak crosses threshold, automatically rollback to last stable parameters.
- Optional automatic mode fallback (default: `active` -> `shadow`).

## Context Buckets (v3)
- The learner keeps per-context parameter memory:
  - `sim/real` phase
  - confidence band (`low`, `mid`, `high`, `unknown`)
- Candidate generation starts from the active context bucket.
- Best reward tracking is maintained per context and globally.

## Persistence
- Settings: `configs/tnt_learning.json`
- Policy: `configs/tnt_learning_policy.json`
- Cycle log: `data/learning_tnt_log.jsonl`

## Rollout
1. Run simulation in `shadow`, inspect logs.
2. Run real robot in `shadow`, verify stability.
3. Enable `active` with conservative bounds.
4. Monitor success rate and rollback if regressions appear.
