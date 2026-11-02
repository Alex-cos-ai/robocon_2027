# Recovery from an incorrect ICP initial pose

Previously, the node repeatedly ran local GICP from the same wheel-propagated
guess. The 1 m initial correction gate was not a search radius and could not
recover a distant starting position or a large yaw error.

Added a bounded XYZ/yaw grid search in a background worker after repeated local
failures, distinct-candidate GICP refinement, full-scan quality and ambiguity
checks, wheel-motion compensation, and validation on fresh consecutive scans.
Tracking also checks full-scan overlap and returns to recovery after repeated
failures. Added `/relocalize` and the RViz `/initialpose` subscriber. RViz's 2D
chassis XY/yaw preserves the current height/tilt estimate before conversion to
the radar pose, including when map Z points downward. An accepted local alignment
or newer manual reset invalidates an older asynchronous search result.

The current default map (`mid360_sim_2.pcd`), launch defaults, FAST-LIO parameters
and Gazebo spawn were retained. Both user maps were copied before testing and
their SHA-256 hashes remained unchanged afterwards. `changes.patch` and source
backups are stored here; `COLCON_IGNORE` prevents backups being built as packages.

Build passed. The compiled node passed all cases in `check_global_recovery.py`:

- Wait for three consistent scans before accepting local initialization, then
  reject a 0.50 m tracking jump before the loss threshold is reached.
- Reject insufficient overlap even when the matching subset has low error.
- Recover a synthetic initial guess with XYZ error over 4 m and yaw error 1.17
  rad; then recover a simulated unobserved 1.3 m displacement while wheel odometry
  stays unchanged.
- Reject two indistinguishable locations in a repeated synthetic map; accept a
  subsequent RViz-style 2D initial pose and validate it. The `/relocalize` service
  also triggered a new search request.

Reproduce with an unused ROS domain after sourcing `scripts/env.sh`:

```bash
ROS_DOMAIN_ID=95 python3 reports/icp_global_recovery_20261101_170944/check_global_recovery.py
```

The actual field simulation was started with deliberately incorrect map pose
`initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=1.5708`. It loaded the
59,071-point `mid360_sim_2.pcd` and used the 12,856-point downsampled map. Ten
rejections triggered search; 139,968 grid poses and 31 GICP refinements produced
the candidate in approximately 1.3 s. Three fresh scans validated the lidar pose:

```
map XYZ: [-2.7197, -2.5663, 0.6141] m
map RPY: [-0.002789, 0.003037, -0.049340] rad
fitness: 0.00151 m^2; overlap within 0.20 m: 100%
```

Later stationary scan-to-map nearest-neighbor RMSE was 0.0394 m, with 100% of
1,860 scan points within 0.20 m and matching scan/pose timestamps. See
`simulation_wrong_xy_yaw.log` and `simulation_overlap.json`. This demonstrates
local map agreement after recovery, not independent global map accuracy.

The search assumes an approximately correct gravity/mount roll and pitch,
searches within map bounds plus a 1 m margin, and caps grid size/work. Sparse,
repetitive or distorted maps can remain ambiguous; a manually supplied location
or better map is still necessary in those cases. It does not use Gazebo truth.
