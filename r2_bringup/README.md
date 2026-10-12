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

## Run on the Robocon 2027 game field

```bash
ros2 launch r2_bringup field_sim.launch.xml objects:=false   # recommended for driving work
ros2 launch r2_bringup field_sim.launch.xml                  # full field, all 53 game objects
```

Same stack as `sim.launch.xml`, on `robocon2027_field`'s world, spawned inside a
rulebook-legal Start Zone instead of at the world origin.

**Field frame**: metres, origin = **south-west field corner on the floor**, x east,
y north, z up. Red half x < 5.5, Blue half x > 5.5. Floor z 0, Level 1 z 0.6,
Level 2 z 0.9, Central Pillar top z 1.7.

| Argument | Default | Meaning |
|---|---|---|
| `objects` | `true` | Spawn the 40 Earth Blocks, 12 Sky Blocks and Mustika. **Read the cost below.** |
| `team` | `red` | `red` or `blue`; picks the default start `x` |
| `world` | derived from `objects` | Overrides `objects` if given explicitly |
| `x`, `y`, `z`, `yaw` | `1.75, 0.85, 0.15, 1.5708` | Red Start Zone 2 (BR), facing north |
| `physics_engine` | `""` | Blank honours the world's choice. **Do not set this — see below.** |

Everything else is forwarded to `sim.launch.xml` unchanged.

### Start pose

The 2027 rulebook names two robots: **TR** (Transporter Robot) and **BR** (Builder Robot).
**R2 is the BR.** Rule 3.2.2 permits BR in the Transfer Area, Level 1 (L1) and Level 2 (L2),
and lets it touch the Ground Area during retries or while navigating onto the ramp/stairs.

> The current build carries no manipulator — that is a placeholder for movement and
> navigation testing, not a statement that BR needs none. Nothing about the start pose or
> launch changes when the real end-effector is added; only the URDF does.

Rule 4.1.2 lets the team choose which of the two 700 × 700 mm Start Zone boxes takes TR and
which takes BR, and requires the robot to be *completely* inside it. At 600 × 600 mm the
base has 50 mm of clearance on every side at the box centre.

| | zone 1 (TR) | zone 2 (BR, default) |
|---|---|---|
| Red | (0.85, 0.85) | (1.75, 0.85) |
| Blue | (9.25, 0.85) | (10.15, 0.85) |

Zone 2 is BR's default because its x lane lines up with the ramp and the L1 stairs
(both x 1.5–2.5) — exactly what BR needs to reach L1/L2. `yaw 1.5708` faces north, into the
field. Use `x:=0.85` (or `team:=blue` with the corresponding x) for TR's Ground-only box
instead.

### Game objects cost 70x real time

Measured on this workspace, headless, dartsim, no robot:

| world | RTF |
|---|---|
| `robocon2027_field_only.sdf` (field only) | **1.00** |
| `robocon2027.sdf` (field + 53 objects) | **0.014** |

The objects start in 84 pairs of exact zero-gap contact — the Earth Blocks are ten touching
two-high stacks per side, sitting flush on the floor — and the simulator spends its entire
budget resolving those resting contacts at the 1 ms step. That is roughly 70 s of wall time
per second of sim: usable for inspecting the field or a short scripted pick, not for
driving, odometry runs or anything timed.

So use `objects:=false` for navigation, odometry and controller work, which is everything
this stack currently does, and switch the objects on when the blocks actually matter.

### Do not "fix" that with a different physics engine

`bullet` and `bullet-featherstone` both run the full world at RTF 1.00, which is tempting.
Neither reproduces the anisotropic `fdir1` friction a mecanum base needs. Measured on the
empty world, 2 s per case at 0.3 m/s:

| engine | contact model | forward | strafe | rotate | diagonal |
|---|---|---|---|---|---|
| dartsim | `friction` | 0.88 | 0.88 | 0.90 | 0.86 |
| bullet-featherstone | `friction` | 0.92 | **0.00** | **0.00** | **0.16** |
| bullet-featherstone | `rollers` | 0.94 | **−0.02** | 0.79 | **0.12** |

Under bullet the robot cannot strafe at all — it is no longer a holonomic base.
**dartsim is the only engine this robot drives correctly on.** The `physics_engine`
argument exists to reproduce that comparison, not to be switched on.

### Measured drive quality on the field

Field-only world, open ground at (1.0, 1.2), 2 s per case at 0.3 m/s, against the same
measurement on `empty.sdf`:

| case | empty.sdf | field | |
|---|---|---|---|
| forward | 0.88 | 0.88 | |
| strafe | 0.88 | 0.88 | |
| rotate | 0.90 | 0.89 | |
| diagonal | 0.86 | 0.86 | |

The field floor's `mu` 0.6 (against empty.sdf's 1.0) costs nothing measurable. **Driving on
the field is the same as driving on the empty world.**

One exception, worth knowing before you debug it: within ~5 cm of the **Level 1 platform's
vertical face**, diagonal efficiency collapses to 0.27/0.40 and picks up ~5.5° of yaw error,
while forward, strafe and rotate stay normal. Bisected to the single collision shape
`c010_level_1_area__red`; the mirrored blue platform 3 m further away has no effect, so it
is proximity-dependent, not a property of the field as a whole. Raising `mecanum_mu_grip`
to 2.0 removes the yaw error but not the shortfall. Keep clearance from the platform edge
when measuring, or expect that number.

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
