# r2_bringup

Top-level composition for the R2 mecanum base, plus the wheel-odometry/IMU fusion
configuration.

**Start here.**

## Run

```bash
ros2 launch r2_bringup sim.launch.xml
```

Brings up: Gazebo world → sensor bridge → robot → controllers → localisation → RViz.

Drive it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/key_vel
```

`stamped:=true` is required — see `r2_control/README.md`.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `r2_gazebo/worlds/empty.sdf` | Must keep the Sensors and Imu system plugins |
| `gui` | `true` | Gazebo GUI |
| `rviz` | `true` | RViz with `r2_gazebo/config/r2_sim.rviz` |
| `paused` | `false` | Start paused |
| `use_sim_time` | `true` | |
| `contact_model` | `friction` | `friction` or `rollers` — see `r2_gazebo/README.md` |
| `publish_roller_states` | `false` | Complete the TF tree with the passive rollers. Needs `contact_model:=rollers`. |
| `mecanum_mu_slip` | `0.05` | Friction across `fdir1` (`friction` model only) |
| `mecanum_mu_grip` | `1.0` | Friction along `fdir1` (`friction` model only) |
| `use_ground_truth` | `true` | Publish `/ground_truth/odom` for scoring |
| `localization` | `true` | Run the EKF; `false` to inspect raw wheel odometry alone |
| `x`, `y`, `z`, `yaw` | `0, 0, 0.15, 0` | Spawn pose |
| `controllers_file` | `r2_control/config/r2_controllers.yaml` | |
| `ekf_config` | `config/ekf.yaml` | |

Headless: `ros2 launch r2_bringup sim.launch.xml gui:=false rviz:=false`

## Localisation

`robot_localization`'s `ekf_node`, configured in `config/ekf.yaml`.

### Why an EKF rather than a complementary filter

Mecanum wheel odometry's worst-estimated quantity is **yaw**. Roller slip on all four wheels
compounds into heading with no redundancy to cancel it, while a MEMS gyro is excellent at
short-term yaw rate. So the fusion has one specific job:

> **wheel odometry supplies `vx`/`vy`; the IMU supplies `vyaw`.**

A hand-rolled complementary filter would mean custom code to write and test, would emit no
covariance (nav2 needs it), and would not generalise when a second pose source arrives.
`robot_localization` is the proven, entirely config-driven option, and `two_d_mode: true`
makes it cheap. Measured cost: one node at 50 Hz.

### What is fused

| Source | Fused | Deliberately not fused |
|---|---|---|
| `/mecanum_drive_controller/odometry` | `vx`, `vy` | pose (it is just the integral of those — fusing both double-counts and makes the filter overconfident); `vyaw` (left to the gyro) |
| `/imu/data` | `vyaw`, `ax`, `ay` | absolute orientation — see below |

**Absolute IMU orientation is deliberately excluded.** Gazebo derives IMU orientation from
ground truth, so fusing it would give the simulated robot a drift-free heading that no real
gyro-without-magnetometer can deliver, hiding exactly the error this filter exists to
manage. Only yaw *rate* is used, which is what real hardware gives. This is a deliberate
choice to make the simulation harder, not easier.

`two_d_mode: true` pins z, roll and pitch instead of letting them random-walk on integrated
accelerometer noise. Valid because the base is rigid and planar.

## API

| Topic | Direction | Type |
|---|---|---|
| `/odometry/filtered` | published | `nav_msgs/Odometry` (50 Hz) |
| `/mecanum_drive_controller/odometry` | subscribed | `nav_msgs/Odometry` |
| `/imu/data` | subscribed | `sensor_msgs/Imu` |
| `/tf` | published | `odom → base_footprint`, and nothing else |

The EKF is the **only** publisher of `odom → base_footprint`. The controller's
`enable_odom_tf` is `false` for that reason.

## Measured accuracy

Closed 2 m square, translation only, `contact_model:=rollers`:

| metric | run 1 | run 2 | run 3 | |
|---|---|---|---|---|
| loop closure (ground truth) | 0.100 m | 0.109 m | 0.066 m | 0.8–1.4% of the 8 m path — drive/contact quality |
| estimate error (EKF vs truth) | 0.058 m | 0.189 m | 0.237 m | 0.7–3.0% of path — odometry quality |
| estimate yaw error | −0.20° | −0.25° | −0.05° | over the whole square |

Loop closure and yaw are stable run to run; **the EKF translation error is not**, ranging
over roughly 4×. The driver is the IMU accelerometer bias, drawn afresh each run from
`bias_mean: 0.1` m/s² in `r2_description/urdf/r2.gazebo.xacro` and then integrated. If you
want a tighter and more repeatable estimate, that bias and the `ax`/`ay` fusion in
`ekf.yaml` are the first two things to look at — quoting a single run's number here would
have been misleading.

Reproduce with `ros2 run r2_gazebo odometry_check.py` while this stack is running.

## Tuning

Covariances live in `config/ekf.yaml`, never in a launch file. The two knobs that matter:

- `process_noise_covariance` — the yaw entry (index 5) is deliberately large relative to
  x/y. That is what makes the filter lean on the gyro instead of on integrated wheel
  velocities.
- `pose_covariance_diagonal` / `twist_covariance_diagonal` in
  `r2_control/config/r2_controllers.yaml` — the controller's own confidence, with yaw set
  far looser than x/y for the same reason.

Treat this file as the package default: copy it and point `ekf_config` at your copy rather
than editing it in place.

## Why localisation lives here

It is currently one YAML plus a short launch include for an off-the-shelf node. A separate
`r2_localization` package for that would be over-engineering. It should graduate to its own
package when map-frame localisation (AMCL, map_server) arrives with nav2.

## Not included, by design

No nav2, no SLAM, no map server, no hardware drivers. The stack is deliberately stopped at
a good `odom → base_footprint` so nav2 can be added on top as a separate step.
