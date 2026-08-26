# r2_control

`ros2_control` controller configuration and velocity multiplexing for the R2 mecanum base.

Deliberately contains nothing simulation-specific. The same controllers, the same config
and the same topic names run against real hardware — that is the point of putting the drive
stack behind `ros2_control` rather than using a Gazebo drive plugin.

## Run

Normally included by `r2_gazebo/launch/empty_world.launch.xml` or
`r2_bringup/launch/sim.launch.xml`. Standalone, against an already-running
`controller_manager`:

```bash
ros2 launch r2_control control.launch.xml
```

| Argument | Default | Meaning |
|---|---|---|
| `controllers_file` | `config/r2_controllers.yaml` | Controller parameters |
| `twist_mux_file` | `config/twist_mux.yaml` | Multiplexer inputs and priorities |
| `controller_manager_timeout` | `60` | Seconds to wait for the controller manager |
| `start_twist_mux` | `true` | Run the velocity multiplexer |

## Controllers

| Controller | Type | Purpose |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes `/joint_states` from the four wheels' position + velocity state interfaces |
| `mecanum_drive_controller` | `mecanum_drive_controller/MecanumDriveController` | Mecanum inverse kinematics and odometry |

## API

### Subscribed

| Topic | Type | Note |
|---|---|---|
| `/mecanum_drive_controller/reference` | `geometry_msgs/TwistStamped` | **Stamped, and not `/cmd_vel`.** See below. |
| `/joy_vel`, `/key_vel`, `/nav_vel` | `geometry_msgs/TwistStamped` | `twist_mux` inputs, priority 100 / 90 / 10 |

### Published

| Topic | Type | Note |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | |
| `/mecanum_drive_controller/odometry` | `nav_msgs/Odometry` | True mecanum forward kinematics, **not** a diff-drive approximation |
| `/mecanum_drive_controller/controller_state` | `control_msgs/MecanumDriveControllerState` | |
| `/mecanum_drive_controller/tf_odometry` | `tf2_msgs/TFMessage` | Inactive: `enable_odom_tf` is `false` |

`enable_odom_tf` is **false** so that the EKF in `r2_bringup` is the only publisher of
`odom → base_footprint`.

## The TwistStamped wart

`mecanum_drive_controller` is a *chainable* controller. Its reference input is
`geometry_msgs/TwistStamped` on `~/reference`, not the plain `geometry_msgs/Twist` on
`/cmd_vel` that most ROS teleop and navigation tooling emits. `twist_mux` with
`use_stamped: true` bridges the gap and adds priority arbitration:

```
teleop_twist_keyboard (stamped) → /key_vel  ┐
joystick                        → /joy_vel  ├→ twist_mux → /mecanum_drive_controller/reference
nav2 (later)                    → /nav_vel  ┘
```

### Driving it

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/key_vel
```

`stamped:=true` is **required**. Without it the node publishes plain `Twist` and nothing
moves.

> **Publishing references from your own node?** Set `use_sim_time` on it. The controller
> enforces `reference_timeout` by comparing `header.stamp` against *its* clock, which is
> sim time. A node stamping with wall-clock time produces references that look hours stale,
> so every one is silently discarded and the robot sits still while looking perfectly
> healthy. This cost real debugging time.

## Key parameters

| Parameter | Value | Why |
|---|---|---|
| `update_rate` | 100 Hz | Above the 50 Hz EKF and 10 Hz LIDAR; cheap for four joints |
| `reference_timeout` | 0.5 s | Zeroes the wheels if teleop dies. The only safety interlock right now. |
| `kinematics.wheels_radius` | 0.076 | |
| `kinematics.sum_of_robot_center_projection_on_X_Y_axis` | **0.490** | `lx + ly`. **Must match `r2_description`** — a mismatch silently scales odometry. |
| `enable_odom_tf` | `false` | The EKF owns `odom → base_footprint` |
| `*_covariance_diagonal` | yaw ≫ x/y | Encodes that mecanum wheel-derived heading is the least trustworthy quantity, so the EKF leans on the gyro |

## Known gaps

- **No acceleration or jerk limiting.** Unlike `diff_drive_controller`, this controller has
  no `SpeedLimiter`. A velocity step command becomes a wheel velocity step. If that causes
  slip, add a rate limiter upstream of `twist_mux`.
- `mecanum_drive_controller` is newer and less battle-tested than `diff_drive_controller`.

## Moving to real hardware

Nothing in this package changes. Write a `hardware_interface::SystemInterface` plugin that
reads four encoders and writes four wheel velocity setpoints, then set `use_sim:=false` so
`r2_description/urdf/r2.ros2_control.xacro` loads it instead of
`gz_ros2_control/GazeboSimSystem`.
