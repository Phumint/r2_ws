# r2_gazebo

Gazebo Harmonic simulation for the R2 mecanum base: worlds, the ROS↔Gazebo topic bridge,
the launch files that put a simulated robot in a world, and the tools that measure whether
it drives correctly.

Simulation-only. Nothing here is needed on real hardware.

## Run

Minimal isolated rig — empty world, robot, controllers. **No localisation, no RViz**, so
that "is the mecanum drive itself working?" can be answered without an EKF in the loop that
might mask or invent behaviour:

```bash
ros2 launch r2_gazebo empty_world.launch.xml
```

Then drive it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/key_vel
```

`stamped:=true` is required — see `r2_control/README.md`.

For the full stack (localisation + RViz) use `r2_bringup/launch/sim.launch.xml`.

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo GUI; `false` for headless/CI |
| `world` | `worlds/empty.sdf` | World SDF path |
| `contact_model` | `friction` | `friction` or `rollers` — see below |
| `mecanum_mu_slip` | `0.05` | Friction **across** `fdir1`, the roller free-run direction. `friction` model only. |
| `mecanum_mu_grip` | `1.0` | Friction **along** `fdir1`, the roller axle direction. `friction` model only. |
| `publish_roller_states` | `false` | Export the passive roller joints on `/joint_states`. Requires `contact_model:=rollers`. |
| `mecanum_roller_count` | `8` | Rollers per wheel (`rollers` model only) |
| `mecanum_roller_damping` | `0.0` | Roller bearing viscous damping (`rollers` model only) |
| `mecanum_roller_friction` | `0.0` | Roller bearing Coulomb friction (`rollers` model only) |

`mecanum_mu_slip` lands in SDF `<mu2>` and `mecanum_mu_grip` in `<mu>`. Swapping them
rotates the anisotropy by 90°, which still drives forward — see `r2_description/README.md`.

## Launch files

| File | Role |
|---|---|
| `world.launch.xml` | Gazebo + the topic bridge. No robot. Takes `physics_engine` (blank = the world's own choice). |
| `spawn_robot.launch.xml` | `robot_state_publisher` + `ros_gz_sim create` |
| `empty_world.launch.xml` | The two above + controllers. The isolated test rig. |

## Worlds

| World | Contents | RTF (headless, no robot) |
|---|---|---|
| `worlds/empty.sdf` | ground plane + sun | 1.00 |
| `robocon2027_field/worlds/robocon2027_field_only.sdf` | the 11 x 11 m game field | 1.00 |
| `robocon2027_field/worlds/robocon2027.sdf` | field + 53 game objects | **0.014** |

The field worlds live in the `robocon2027_field` package and are generated from the team's
Fusion CAD by its `tools/make_field.py`. Launch them with
`ros2 launch r2_bringup field_sim.launch.xml` — see that package's README for the frame,
the start poses and the real-time-factor trade-off.

**Every world this robot is spawned into must declare `gz-sim-sensors-system` and
`gz-sim-imu-system`.** The field world as generated from CAD declared no system plugins at
all, which costs you every sensor with no error message; the plugin block is now emitted by
`make_field.py`, and `r2_bringup`'s
`test_worlds_load_the_plugins_the_sensors_need` fails if any world loses them.

**Do not switch physics engine to buy back the full world's real-time factor.** `bullet` and
`bullet-featherstone` run it at RTF 1.00 but neither supports the anisotropic `fdir1`
friction a mecanum base needs — measured strafe efficiency drops to 0.00 (`friction` model)
and −0.02 (`rollers`), i.e. the robot cannot strafe at all. dartsim is the only engine this
robot drives correctly on. The `physics_engine` argument on `world.launch.xml` exists to
reproduce that comparison.

## Bridged topics

Configured in `config/ros_gz_bridge.yaml`. All are Gazebo → ROS.

| ROS topic | Type | Rate | frame_id |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | — | — |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `lidar_link` |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | `imu_link` |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 15 Hz req / **13.1 measured** | `camera_optical_link` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `camera_optical_link` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 15 Hz req / **13.4 measured** | `camera_optical_link` |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 15 Hz req / **13.3 measured** | `camera_optical_link` |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 50 Hz | `ground_truth_odom` |

Sensor streams are bridged with `qos: SENSOR_DATA`. Without it the bridge publishes
RELIABLE, which mismatches RViz displays and anything using `rclcpp::SensorDataQoS` — the
topics connect and no messages ever arrive.

**`/ground_truth/odom` is a measurement instrument, not a robot sensor.** Nothing in the
robot's own control or localisation stack may subscribe to it, or the simulation stops
proving anything.

Its `ground_truth_odom` frame is the **Gazebo world frame** — the plugin reports the world
pose of `base_footprint`, so spawning at `x:=2.0 y:=1.0` makes it read `(2.0, 1.0)` while
the EKF reads `~0`. No link in the description carries that name, so `sim.launch.xml`
anchors it with a static `odom → ground_truth_odom` transform equal to the inverse of the
spawn pose (identity for the default origin spawn). Without that anchor the frame is an
orphan, and any tf2 `MessageFilter` consumer — RViz's "Odometry (ground truth)" display
above all — queues every message waiting for a transform that never arrives and then
reports:

```
Message Filter dropping message: frame 'ground_truth_odom'
for reason 'discarding message because the queue is full'
```

The anchor is only launched when `localization:=true`, because `odom` does not exist
otherwise: `mecanum_drive_controller` has `enable_odom_tf: false`, so the EKF is the only
publisher of that frame. `empty_world.launch.xml` runs no EKF and no RViz, so it neither
has nor needs it.

## Validation tools

### `holonomic_check.py`

Commands a constant body velocity and reports what the robot actually did, measured against
ground truth. This is the test that decides whether the contact model is good enough.

```bash
ros2 run r2_gazebo holonomic_check.py
ros2 run r2_gazebo holonomic_check.py --duration 4.0 --speed 0.4
```

### `odometry_check.py`

Drives a closed 2 m square using translation only, then scores loop closure (drive quality)
and EKF error (odometry quality). Needs the full stack from `r2_bringup`.

```bash
ros2 run r2_gazebo odometry_check.py --side 2.0 --speed 0.3
```

### `stop_check.py`

Measures coast distance and failure-to-stop over N trials, under three command profiles:
`continuous`, `ramp`, and `burst` (which emulates teleop's one-message-per-keypress
behaviour). See "Stopping behaviour" below.

```bash
ros2 run r2_gazebo stop_check.py --profile burst --trials 10
```

## Measured results

Recorded on this workspace, headless, `contact_model:=rollers`, 3 s per case at 0.3 m/s and
0.5 rad/s. Re-run these after any change to wheel geometry, mass or friction.

| case | commanded | measured | efficiency | cross-axis |
|---|---|---|---|---|
| forward | vx 0.30 | dx +0.845 | 0.94 | dy +0.001, dyaw −0.02° |
| strafe | vy 0.30 | dy +0.838 | 0.93 | dx −0.016, dyaw +0.08° |
| rotate | wz 0.50 | dyaw +80.05° | 0.93 | dx, dy ≈ 0 |
| diagonal | vx, vy 0.30 | dx +0.797, dy +0.781 | 0.89 / 0.87 | dyaw +0.45° |

The 6–13% shortfall is acceleration ramp-up inside the measurement window plus genuine
roller slip. Closed 2 m square, translation only:

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

Real-time factor with all sensors and the roller model: **≈1.00**.

The RGB-D sensor does not quite reach its configured 15 Hz: measured 13.1-13.4 Hz on all
three streams over a 16 s window, about 88% of target. Physics is not starved (RTF stays
1.00), so this is render throughput. It is stated here rather than papered over - if you
need a true 15 Hz, lower the resolution before lowering the rate.

## Stopping behaviour

Everything in this section was measured on `contact_model:=rollers`. The default `friction`
model does not have this problem — 0 failures in 10, coast 0.095–0.107 m. See "The contact
model is the cause" below.

Commanding zero velocity holds the wheel joints at exactly 0.000 rad/s within ~0.2 s. The
chassis then coasts, and **intermittently fails to stop at all**, creeping on at
0.05-0.22 m/s. Among successful stops the coast is 0.10-0.33 m (typically ~0.14 m), implying
~0.57 m/s2 of deceleration.

Braking here is limited by the wheel/ground contact, not by torque: stopping uses ~0.28 N.m
per wheel against a 40 N.m limit, an effective braking friction of only ~0.06 despite the
rollers being set to `mu = 1.0`.

### Measurement methodology matters more than any parameter

Failure rate depends heavily on how the batch was run, which confounds naive comparisons:

| grouping | failures |
|---|---|
| fresh simulator, short batch (<=6 trials) | 1 / 19 |
| long or shared simulator session | 10 / 35 |

Something accumulates over a simulator session. **Restart Gazebo between configurations, run
at least 10 trials, and always collect a same-session baseline.** An early conclusion of this
investigation was wrong because it compared batches with different structures.

### What survives that confound

| comparison | result | verdict |
|---|---|---|
| continuous vs burst command profile, head-to-head, 10 trials each | 3/10 vs 3/10 | no effect |
| `publish_roller_states:=true` vs default, both fresh-sim | 5/11 vs 1/19 | real effect |

`teleop_twist_keyboard` publishes one message per keypress with no repeat timer, so teleop
produces bursts rather than a stream and `reference_timeout: 0.5` fills the gaps. That is a
genuinely different regime, but measured head-to-head it makes no difference to stopping.

Ruled out by measurement: insufficient motor torque (~140x headroom), a missing deceleration
limiter (ramping does not help), and roller bearing losses or roller count.

Still unexplained: why `publish_roller_states:=true` degrades stopping. It is not control-loop
overrun - the gz_ros2_control logs are identical between runs.

Reproduce with `ros2 run r2_gazebo stop_check.py --profile burst --trials 10`.

### The contact model is the cause

Run head to head, same session, fresh simulator each, burst profile, 10 trials each:

| contact model | failed to stop | coast min / max / mean | implied decel |
|---|---|---|---|
| `rollers` | **4 / 10** | 0.113 / 0.643 / 0.293 m | 0.27 m/s² |
| `friction`, fixed | **0 / 10** | 0.095 / 0.107 / 0.101 m | 0.79 m/s² |

Every `friction` trial ended at exactly 0.0000 m/s, and the coast spread collapsed from 5.7×
to 1.1×. `rollers` also fails to hold its commanded cruise speed (`v_release` 0.398–0.424
against a commanded 0.400) where `friction` hits 0.400 every trial.

This settles the open question of whether the rollers roll or slide at the moment of failure.
Removing the roller *links* — while keeping the same wheel mass, the same 45° anisotropy and
the same `mu = 1.0` — removes the failure completely. The energy was going into free-spinning
roller bearings, so the mechanism is the constraint geometry, not contact friction. It is
consistent with the rest of the evidence: the effective braking coefficient of ~0.06 against
rollers set to `mu = 1.0` is what a nearly free rolling contact looks like, and it explains
why bearing damping, roller count and command ramping all failed to move the number — none of
them stops a roller that is free to turn.

It also very likely explains the `publish_roller_states:=true` regression, which only exists
in a model that has roller joints to publish.

## Contact model: `friction` vs `rollers`

**Both are holonomic.** `friction` was documented here as not being holonomic, and the
measurement behind that was real, but the cause was three bugs in `mecanum_wheel.xacro`
rather than a limitation of dartsim. They are described in full in `r2_description/README.md`;
in short: `fdir1` was resolved in the wheel's own rotating frame, `mu`/`mu2` were swapped
relative to the roller axle, and the collision was a cylinder instead of a sphere.

Measured on this workspace, headless, same session, 3 s per case at 0.3 m/s and 0.5 rad/s:

| contact model | forward | strafe | rotate | diagonal | SDF links |
|---|---|---|---|---|---|
| `friction`, before the fix | works (1.04 m) | **0.000** | **0.000** | **~0.000** | 5 |
| `friction`, fixed | 0.92 | 0.92 | 0.93 | 0.92 | 5 |
| `rollers` | 0.94 | 0.93 | 0.93 | 0.89 / 0.87 | 37 |

`friction` is now the default. It matches `rollers` on every motion axis, costs 5 links
instead of 37, and unlike `rollers` it stops when told to. `rollers` is kept as the more
physically faithful model — the rollers really do roll, and it is the right thing to measure
a contact approximation against — but it should not be used to drive.

## Gotchas

- **World system plugins are not boilerplate.** `gz-sim-sensors-system` and
  `gz-sim-imu-system` are required for the camera/LIDAR and the IMU respectively. The stock
  `empty.sdf` shipped with gz-sim has **neither**, so a robot spawned into it produces no
  sensor data at all — silently, with no error. Any replacement world must keep both.
- **`colcon build` can silently skip reinstalling data files.** Launch and config edits then
  appear to have no effect, and you debug a stale model. Build this workspace with
  `colcon build --symlink-install` so `src` edits take effect immediately.
- **`gz topic -l` needs `GZ_CONFIG_PATH` and `LD_LIBRARY_PATH`** pointing at the vendored
  gz libraries, or it silently reports zero topics.
- The ground-truth plugin's `<odom_topic>` needs a **leading slash**. Without it, gz also
  advertises a model-scoped `/model/r2/ground_truth/odom` that carries no data, which is an
  easy one to bridge by mistake.
- The `<plane>` ground collision is finite at its `<size>`. A robot that slides far enough
  falls off the edge.
- **Resting contacts, not shape count, are what cost real time.** The game field's 87 static
  collision primitives run at RTF 1.00; adding the 53 game objects — which start in 84 pairs
  of exact zero-gap contact — drops it to 0.014. If a world crawls, look for bodies resting
  on each other before you look at how many shapes it has.
- **Measure holonomic motion away from walls and platform edges.** `holonomic_check.py`
  drives ~0.5 m per case from wherever the robot is; started in a Start Zone it strafes
  straight into the field boundary and reports a low efficiency that is really a collision.
  Separately, within ~5 cm of the Level 1 platform face, diagonal efficiency genuinely
  collapses to 0.27 with ~5.5° of yaw error while every other case stays normal.
