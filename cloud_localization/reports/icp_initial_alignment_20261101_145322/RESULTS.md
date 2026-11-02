# ICP initialization check

The current source localization launch selects `mid360_sim_2.pcd`. Its existing
initial XY guess (-2.63, -2.63), Gazebo spawn, IMU settings and map selection were
retained. Both `mid360_sim_1.pcd` and `mid360_sim_2.pcd` were copied before testing;
SHA-256 checks confirmed neither source map changed during the checks.

The first candidate required map-axis translation [-0.095, +0.071, +0.615] m.
The relative rotation was approximately [-0.17, +0.19, -2.31] degrees. The 0.626 m
correction norm had exceeded the original 0.35 m gate, mainly due to Z.

After three consistent scans, the accepted initial lidar pose was:

```
map XYZ (metres): -2.7256 -2.5592 0.6144
map RPY (radians): -0.002941 0.003293 -0.041321
fitness: 0.00153 m^2
scan overlap within 0.20 m: 100%
```

These are measured estimates for this run, not universal calibration defaults.
Subsequent tracking corrections were on the millimetre scale. The saved
`simulation_localization.log` contains the candidates and stage transition.

A scan-to-map nearest-neighbor check measured RMSE 0.0394 m while stationary,
and 0.0406 m after a short 0.10 m/s, 3 s forward command and settling. Both checks
had 100% of points within 0.20 m. Cloud/pose timestamps differed by about 0.10 s;
these measurements were taken after stopping and are not a moving-scan latency
benchmark. Details are in `simulation_overlap.json`.

Build and launch argument parsing passed. The actual compiled node also passed
three synthetic input checks: apply a valid >0.35 m initial correction only after
three scans; reject a further 0.50 m jump once tracking; reject initialization
outside its configured limit or with insufficient scan overlap. To reproduce:

```bash
cd /home/alice/rc_2027/cloud_localization
source scripts/env.sh
ROS_DOMAIN_ID=94 python3 reports/icp_initial_alignment_20261101_145322/check_initial_alignment.py
```

Use an unused ROS domain when running this test. It generates temporary PCD
targets and does not start Gazebo. `changes.patch` shows the source changes
against the backups in this directory.
