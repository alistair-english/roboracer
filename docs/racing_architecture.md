# Racing architecture

The plan for going beyond the reactive midline-tracer baseline. Goal: minimise
total time across the 10 timed race laps on an unseen track, given one free
warm-up lap and stiff collision penalties (+10 s, +20 s, +30 s, …).

## Constraints we're designing around

From the [2026 rules][rules] and [technical guide][guide]:

- **One warm-up lap** that doesn't count, then 10 timed laps, then a
  cool-down lap. So we have exactly one lap to map an unseen track before
  every metre is on the clock.
- **Restricted topics**: `/tf`, `/autodrive/.../ips`, `/autodrive/.../odom`,
  collision/lap telemetry. The sim keeps publishing them, but we may not
  consume them at race time. So no ground-truth pose — we have to localise
  from scratch.
- **Allowed inputs**: LiDAR (270°, 40 Hz, 1080 rays, 0.06–10 m), IMU, raw
  wheel encoders (16 PPR, 120:1, wheel r = 0.059 m), camera, current
  steering/throttle feedback.
- **Vehicle**: Ackermann, 22.88 m/s top speed, ±30° steering, 3.2 rad/s
  steering rate, 0.324 m wheelbase.
- **Track**: ~30×10 m, ≥ 0.9 m wide. Practice track shipped, race track
  withheld. Track is fully closed (no obstacles introduced mid-race per the
  guide, just chicanes and possible gaps).

The collision penalty maths matters: one crash costs as much as a 22 m/s
straight, so reliability dominates raw pace. Always keep a working fallback.

[rules]: https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/
[guide]: https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/

## Three phases

```
   Lap 1                      |  Between lap 1 → 2  |  Laps 2..11
   ──────                     |  ─────────────────  |  ─────────
   tentacle controller drives |  centerline extract |  AMCL localise vs map
   slam_toolbox builds map    |  raceline optimise  |  pure-pursuit tracker
   encoder+IMU odom           |  velocity profile   |  velocity feed-forward
                              |                     |  tentacle fallback
```

### Phase 1 — map (warm-up lap)

Drive a deliberately conservative lap with the existing tentacle controller
while building a 2D occupancy grid via SLAM. We have ~one lap of data, so
loop-closure quality matters more than scan rate.

**Components:**

- **Encoder/IMU dead-reckoning** node. Subscribes to `/.../left_encoder`,
  `/.../right_encoder`, `/.../imu`. Differential rear-axle wheel speed →
  forward velocity. IMU yaw rate → heading. Integrate a 2D pose. Publish
  `nav_msgs/Odometry` and `odom → base_link` TF. ~150 lines, deterministic,
  no external deps. Swap to `robot_localization` EKF later if drift hurts.
- **slam_toolbox** in async mapping mode. Subscribe to `/.../lidar`, consume
  our `odom` TF, output `map → odom` and the occupancy grid. Save the
  serialised map (`.posegraph` + `.data`) at lap completion.
- **Lap detection**: simplest is "we're back near the SLAM origin and yaw
  agrees" — confirm with slam_toolbox's own loop-closure event. Don't rely
  on `/lap_count` (restricted).
- **Static URDF** for sensor mounts. The tech guide gives offsets directly
  (LiDAR at +0.2733 m forward, +0.096 m up; IMU at +0.08, 0, +0.055; etc.).
  Hardcode them — do not subscribe to sim's `/tf_static`.

### Phase 2 — plan (between lap 1 and lap 2)

Once the map is closed, derive a raceline. This is offline-style work that
just happens to run on the vehicle for ~1 s.

1. **Free-space extraction**: threshold the occupancy grid → binary drivable
   mask. Morphological close to fill scan noise.
2. **Centerline**: `skimage.morphology.medial_axis` on the mask, then trace
   the largest connected ridge as an ordered, closed loop. Resample uniformly
   in arc length (e.g. every 0.2 m). This alone gives us a usable reference.
3. **Raceline (minimum curvature)**: solve the standard QP that minimises
   ∫κ² ds subject to a track-width corridor around the centerline. TUM's
   [`global_racetrajectory_optimization`][tum] is the reference
   implementation — easier to lift than re-derive. Closed-form, sub-second
   for tracks this size.
4. **Velocity profile**:
   - Per point, `v_max(s) = sqrt(a_lat / |κ(s)|)`, capped at vehicle top
     speed.
   - Forward sweep: `v(s+) = min(v_max(s+), sqrt(v(s)² + 2·a_long_max·ds))`.
   - Backward sweep: same, with `a_brake_max`.
   - Tune `a_lat`, `a_long_max`, `a_brake_max` conservatively first; raise
     them once execution is reliable.

Don't bother with full minimum-lap-time optimisation (the
[`time-optimal trajectory`][toml] formulation). The marginal gain over
min-curvature isn't worth the complexity at this vehicle scale, and it
needs a tyre model we'd have to fit.

[tum]: https://github.com/TUMFTM/global_racetrajectory_optimization
[toml]: https://www.sciencedirect.com/science/article/pii/S2405896319301314

### Phase 3 — execute (laps 2..11)

Switch slam_toolbox into **localization mode** against the saved map (or
hand off to AMCL — both work, AMCL is simpler to reason about). With 1080
LiDAR rays against a static known map, localisation is rock solid; the
typical failure mode is initial-pose mismatch, which we avoid by simply
keeping the SLAM-era pose as the seed.

**Tracker — start with pure pursuit, scheduled by velocity profile:**

- Lookahead `L_d(v) = clamp(k_v · v + L_min, L_min, L_max)`. Speed-scaled so
  fast straights look further ahead, slow chicanes look closer.
- Throttle = feed-forward `v_target(s_lookahead)` from the velocity profile,
  with a small PI on the encoder-derived speed error.
- Pure pursuit gives most of the upside over reactive. Don't reach for MPC
  until everything else is working.

**Upgrade path — kinematic MPC (5–10 step horizon, 50 Hz):**

- State `(x, y, ψ, v)`, control `(δ, a)`.
- Cost: tracking error + control effort + terminal cost biased to the
  raceline.
- Constraints: `|δ| ≤ 30°`, `|δ̇| ≤ 3.2 rad/s`, friction circle on
  `(a_long, v²·κ)`.
- Bigger gain than a fancier planner; only do it if the controller is the
  bottleneck.

**Fallback — keep the tentacle controller hot.** Two arming conditions
trigger fallback:

1. AMCL pose covariance exceeds a threshold (we don't trust where we are).
2. LiDAR sees an unmapped return inside the corridor (debris, simulated
   obstacle, anything weird).

The tentacles will lose us a few seconds per lap but cost zero collisions —
a worthwhile trade given the +10 s penalty.

## TF and topic plumbing

The sim publishes ground-truth poses on `/tf` and `/tf_static` via
`autodrive_bridge` — restricted by the rules. Our cleanest defence is to
**rename the bridge's TF output** rather than ours:

- `autodrive_bridge` is launched with remaps `/tf → /sim/tf`,
  `/tf_static → /sim/tf_static`. Its ground-truth tree lives there, and
  nothing in our pipeline subscribes to it.
- Every other node uses default `/tf`, `/tf_static` and standard frame
  names (`map`, `odom`, `base_link`, `lidar`, `imu`, ...). Our SLAM,
  localiser, RViz, and any future nodes need zero special config.
- Compliance: we never read `/tf` from the sim — we read `/tf` from
  ourselves. The restricted topic is the *content*, not the *name*.
- For dev-time sanity checks (overlay our SLAM map against ground truth), a
  separate RViz config can subscribe to `/sim/tf*` explicitly. That's a
  debug-only configuration, never shipped in the submission.

The same principle applies to the other restricted topics: we don't
subscribe to `/.../odom`, `/.../ips`, `/lap_count`, etc. anywhere in the
race-time stack. The bridge can keep publishing them.

## Frames

Standard REP-105:

```
       map ── slam_toolbox / amcl ──▶ odom ── encoder+IMU node ──▶ base_link
                                                                       │
                                          (static, from URDF)          │
                                                                       ├──▶ lidar
                                                                       ├──▶ imu
                                                                       └──▶ front_camera
```

Frame ids on incoming sensor messages need to match this tree — the bridge
currently stamps `/lidar` with `frame_id: lidar`, `/imu` with `imu`, etc.
(verify with `ros2 topic echo --field header.frame_id --once <topic>` once
running, but the bridge source agrees with these names). No header
rewriting needed.

## Build order, in priority

The first three steps already beat the current reactive baseline by a lot.
The last three are where you fight for the seconds.

1. SLAM running end-to-end: encoder/IMU odom → slam_toolbox map building →
   serialised map saved → AMCL relocalising on lap 2. **Most of the
   project; do this first.**
2. Centerline extraction → constant-velocity pure pursuit. Proves the
   "drive against a global path" loop is real.
3. Curvature-based velocity profile. Big lap-time win for one afternoon's
   work.
4. Min-curvature raceline (replaces centerline as the reference path).
5. Kinematic MPC (replaces pure pursuit).
6. Aggressive friction-limit tuning, lookahead retuning, raceline corridor
   slack.

Each step keeps the previous controller as a fallback target until the new
one is proven. We never demote a working pipeline without a tested
replacement.

## Things explicitly out of scope

- **Camera-based perception.** Useful in principle (sign detection, lane
  paint), but the practice/race tracks don't need it and LiDAR-only is
  enough.
- **Deep learning policies.** Sample-inefficient, hard to debug, and we
  don't have repeated track exposure to learn against. The classical stack
  is the right answer here.
- **Online raceline re-optimisation.** Compute it once after the warm-up
  lap and stick with it. Re-optimising mid-race adds risk for negligible
  gain on a static track.
- **Min-lap-time trajectory optimisation.** Min-curvature plus a forward/
  backward velocity profile is within a few % of optimal for a vehicle
  this size. The accuracy gap doesn't justify the tyre-modelling and
  solver complexity.
