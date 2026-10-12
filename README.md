# R2 — holonomic (mecanum) mobile base

Simulation-first ROS 2 stack for an ABU Robocon R2 robot: a 60 × 60 × 50 cm four-wheel
mecanum base with an RGB-D camera, 2D LIDAR, IMU and wheel encoders, localised by fusing
wheel odometry with the IMU.

**ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) · Ubuntu 24.04**

No nav2, no SLAM, no hardware drivers yet — deliberately. The stack stops at a good
`odom → base_footprint` so navigation can be added on top as a separate step.

## Quick start

```bash
# One-time: the ros2_control metapackages are not pulled in by rosdep here
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli

cd ~/r2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install      # see the note below
source install/setup.bash

ros2 launch r2_bringup sim.launch.xml
```

On the Robocon 2027 game field instead of the empty world:

```bash
ros2 launch r2_bringup field_sim.launch.xml objects:=false   # recommended for driving work
ros2 launch r2_bringup field_sim.launch.xml                  # full field, all 53 game objects
```

R2 is the Builder Robot (BR): it spawns in Red Start Zone 2 at (1.75, 0.85) facing north,
the box that lines up with the ramp and L1 stairs BR needs to reach L1/L2. `objects:=true`
is faithful but runs at RTF 0.014 — see [`r2_bringup/README.md`](r2_bringup/README.md) for
why, for the field frame, and for switching to TR's box (`x:=0.85`).

Drive it, in a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/key_vel
```

`stamped:=true` is **required** — `mecanum_drive_controller` takes `TwistStamped`, not
`Twist`. See [`r2_control/README.md`](r2_control/README.md).

> **Build with `--symlink-install`.** Plain `colcon build` can silently skip reinstalling
> launch and config files, so edits appear to have no effect and you end up debugging a
> stale model. These are data-only packages; symlinks make `src` edits take effect
> immediately.

## Packages

| Package | Role | Needed on real hardware? |
|---|---|---|
| [`r2_description`](r2_description/README.md) | URDF/xacro: chassis, wheels, sensors, frames | yes |
| [`r2_control`](r2_control/README.md) | `ros2_control` controllers, velocity multiplexer | yes |
| [`r2_gazebo`](r2_gazebo/README.md) | Worlds, ROS↔Gazebo bridge, validation tools | no |
| [`r2_bringup`](r2_bringup/README.md) | Top-level composition + EKF localisation | yes |
| [`robocon2027_field`](robocon2027_field/README.md) | ABU Robocon 2027 game field: world, URDF, game objects | no |

## Architecture

```
teleop / joystick / (later) nav2
        │  TwistStamped
        ▼
   twist_mux ──► /mecanum_drive_controller/reference
                          │
                          ▼
        mecanum_drive_controller   (mecanum IK + forward-kinematic odometry)
                          │  velocity command interfaces
                          ▼
        ┌─────────────────────────────────────┐
        │  gz_ros2_control/GazeboSimSystem    │ ◄── swap this for a real
        │           (simulation)              │     SystemInterface plugin
        └─────────────────────────────────────┘
                          │
                          ▼
                    Gazebo physics
                          │
    /joint_states, /mecanum_drive_controller/odometry, /imu/data, /scan, /camera/*
                          │
                          ▼
              robot_localization EKF  ──►  odom → base_footprint, /odometry/filtered
```

### Why `ros2_control` rather than Gazebo's `MecanumDrive` plugin

Both were investigated. Gazebo's `MecanumDrive` system only writes the `JointVelocityCmd`
component — it commands wheel joint velocities and lets physics do the rest, *exactly* like
`ros2_control` does. So the choice buys nothing in physics fidelity and comes down to
portability: swapping `gz_ros2_control/GazeboSimSystem` for a real
`hardware_interface::SystemInterface` changes one `<plugin>` line, and leaves controller
config, topic names, odometry maths, the TF tree and the EKF untouched. Odometry is computed
from joint state — which is what real encoders give — so simulation and hardware run the
same code path.

### Why an EKF rather than a complementary filter

Mecanum wheel odometry's worst quantity is yaw: roller slip on four wheels compounds into
heading with nothing to cancel it, while a gyro is excellent at short-term yaw rate. So
**wheel odometry supplies `vx`/`vy` and the IMU supplies `vyaw`**. `robot_localization` does
this from a config file, emits the covariance nav2 will need, and needs no custom code.
Details in [`r2_bringup/README.md`](r2_bringup/README.md).

## Measured behaviour

Headless, 3 s per case at 0.3 m/s / 0.5 rad/s:

| case | `friction` (default) | `rollers` |
|---|---|---|
| forward | 0.92 | 0.94 |
| strafe | 0.92 | 0.93 |
| rotate | 0.93 | 0.93 |
| diagonal | 0.92 | 0.89 / 0.87 |

Cross-axis error is ≈ 0 for `friction` and under 0.016 m / 0.45° for `rollers`. Stopping from
0.4 m/s, burst profile, 10 trials each, same session: `friction` **0 failures**, coast
0.095–0.107 m; `rollers` **4 failures**, coast 0.113–0.643 m.

Closed 2 m square, translation only, over three runs: loop closure **0.066–0.109 m**
(0.8–1.4% of the 8 m path), EKF error vs ground truth **0.058–0.237 m** (0.7–3.0%), yaw
error **−0.05° to −0.25°**. Real-time factor with all sensors: **≈1.00**.

Loop closure and yaw are stable; the EKF translation error is not, and varies about 4×
between runs because the simulated IMU draws a fresh accelerometer bias each time. See
[`r2_bringup/README.md`](r2_bringup/README.md) for what to tune.

Reproduce:

```bash
ros2 launch r2_gazebo empty_world.launch.xml gui:=false   # then, in another terminal:
ros2 run r2_gazebo holonomic_check.py

ros2 launch r2_bringup sim.launch.xml gui:=false rviz:=false
ros2 run r2_gazebo odometry_check.py
```

## Things worth knowing

- **The anisotropic-friction mecanum trick does work in Gazebo Harmonic — but only with
  `fdir1` pinned to the chassis.** This repo previously concluded the opposite, and the
  measurement behind that (drives forward, cannot strafe or rotate at any friction value)
  was real. The cause was three bugs in `mecanum_wheel.xacro`, not a limitation of dartsim:
  `fdir1` was resolved in the wheel's own rotating frame, `mu`/`mu2` were swapped relative to
  the roller axle, and the collision was a cylinder rather than a sphere. Fixed, the model is
  holonomic at 0.92 on every axis, costs 5 links instead of 37, and stops reliably where the
  roller model does not — so `contact_model:=friction` is now the default. Full reasoning in
  [`r2_description/README.md`](r2_description/README.md).
- **`gz:expressed_in` needs `xmlns:gz` on every xacro file that uses it.** xacro parses each
  include as its own XML document, so a declaration in the top-level file does not reach an
  included one. Missing it fails the build with `unbound prefix`.
- **`sum_of_robot_center_projection_on_X_Y_axis` (0.490) must equal `lx + ly` from the
  URDF.** A mismatch silently scales odometry and strafe response. Guarded by the test
  suite in `r2_bringup/test/`.
- **Exactly one node publishes `odom → base_footprint`** — the EKF. The controller's
  `enable_odom_tf` is `false`. Also guarded by tests.
- **A world must load `gz-sim-sensors-system` and `gz-sim-imu-system`.** The stock gz-sim
  `empty.sdf` has neither, and a robot spawned into it produces no sensor data at all,
  silently.
- **Under `contact_model:=rollers`, the roller links are absent from TF by default.** Only
  the four wheel joints are in `<ros2_control>`, so `/joint_states` carries only those and
  `robot_state_publisher` cannot place the 32 roller links.
  `contact_model:=rollers publish_roller_states:=true` completes the tree, but makes stopping
  unreliable - see [`r2_gazebo/README.md`](r2_gazebo/README.md). The flag is a no-op under
  the default `friction` model, which has no roller joints at all.
- **Commanding zero stops the robot in ~0.10 m** from 0.4 m/s on the default `friction`
  model (10/10 trials, coast 0.095–0.107 m). That full stop is intended: four locked mecanum
  wheels constrain the chassis. `rollers` does *not* stop reliably — 4 failures in 10.
- **`--` is illegal inside XML comments.** A pasted `--ros-args` or an em-dash makes a launch
  file fail to parse with a confusing error. Guarded by tests.

## Validate before going further

1. **Roller-model fidelity under load.** The numbers above are for an unloaded robot on a
   flat, high-friction floor. Re-measure with the real mass and a Robocon-like surface.
2. **Depth camera cost.** The RGB-D sensor is the most expensive thing in the world and
   already runs ~12% under its configured rate (13.1–13.4 Hz measured against 15 Hz)
   even with RTF at 1.00, so it is render-throughput limited. On a busier world or a
   weaker GPU it will degrade first. Drop the `/camera/depth/points` bridge entry before
   anything else — the depth image alone is far cheaper.
3. **No acceleration limiting.** `mecanum_drive_controller` has no `SpeedLimiter`, so a
   velocity step becomes a wheel velocity step. Add a rate limiter upstream of `twist_mux`
   if that causes slip.
4. **Height envelope.** The LIDAR tops out at 0.461 m against a 0.50 m budget. Confirm the
   competition measures height with sensors fitted.
5. **Motor realism.** Wheel joints are ideal velocity sources — no motor dynamics, gearbox
   backlash or current limit. Real Robocon motors will not track a velocity step this well.

## Testing

```bash
colcon test && colcon test-result --all
```

18 tests: XML well-formedness across every package, plus integration checks on the seams
between the description, controller config and bridge config — the cross-package constants
that fail silently at runtime rather than loudly at build time.
