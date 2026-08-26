# r2_description

URDF/xacro description of the **R2** holonomic (mecanum) mobile base: chassis, four
mecanum wheels, RGB-D camera, 2D LIDAR and IMU.

This package describes *what the robot is*. It contains no simulator-specific logic beyond
an optional Gazebo block that is switched off by `use_sim:=false`, so the same description
drives a real robot.

## Run

Visualise the description on its own — no Gazebo, no controllers, no physics:

```bash
ros2 launch r2_description display.launch.xml
```

Sliders for the four wheel joints appear in `joint_state_publisher_gui`. This is the
fastest way to check link geometry, joint axes, sensor mounting and the TF tree with
nothing else running that could explain away a mistake.

| Argument | Default | Meaning |
|---|---|---|
| `rviz` | `true` | Start RViz with `config/r2_view.rviz` |
| `jsp_gui` | `true` | Joint sliders; `false` publishes zeros |
| `contact_model` | `friction` | Set `rollers` to eyeball the explicit roller geometry |

## File layout

| File | Role |
|---|---|
| `urdf/r2.urdf.xacro` | Top level. **Include this, never the parts.** |
| `urdf/common_properties.xacro` | Every dimension, mass and material. Single source of truth. |
| `urdf/inertial_macros.xacro` | Box/cylinder/sphere inertia tensors |
| `urdf/base.xacro` | `base_footprint`, `base_link`, chassis, sensor mast |
| `urdf/mecanum_wheel.xacro` | One wheel macro, instantiated 4× |
| `urdf/sensors.xacro` | Sensor links and frames |
| `urdf/r2.ros2_control.xacro` | `<ros2_control>` block — the hardware swap point |
| `urdf/r2.gazebo.xacro` | Gazebo `<sensor>` elements and sim-only plugins |

## Xacro arguments

| Argument | Default | Meaning |
|---|---|---|
| `use_sim` | `true` | `true` → `gz_ros2_control` + Gazebo sensors; `false` → real hardware plugin |
| `use_ros2_control` | `true` | Emit the `<ros2_control>` block |
| `controllers_file` | `""` | Controller YAML path, read by the plugin inside the Gazebo process |
| `contact_model` | `friction` | `friction` or `rollers` — see below |
| `use_ground_truth` | `true` | Emit the Gazebo ground-truth odometry plugin |
| `publish_roller_states` | `false` | Export the 32 passive roller joints on `/joint_states` so TF is complete. Requires `contact_model:=rollers`. |
| `mecanum_roller_count` | `8` | Rollers per wheel |
| `mecanum_roller_damping` | `0.0` | Roller bearing viscous damping |
| `mecanum_roller_friction` | `0.0` | Roller bearing Coulomb friction |
| `mecanum_mu_slip` | `0.05` | Friction **across** `fdir1`, i.e. the roller free-run direction. `friction` model only. |
| `mecanum_mu_grip` | `1.0` | Friction **along** `fdir1`, i.e. the roller axle direction. `friction` model only. |

## Geometry

Everything derives from `common_properties.xacro`, so the 60 × 60 cm footprint and 50 cm
height limit stay satisfied when a value is retuned.

| Quantity | Value | Note |
|---|---|---|
| Footprint | 0.600 × 0.600 m | Chassis 0.600 × 0.480, plus 0.060 wheel width each side |
| Chassis | 0.600 × 0.480 × 0.200 m | 20.0 kg |
| Wheel radius / width | 0.076 / 0.060 m | 152 mm diameter, 1.2 kg each |
| `lx` (half wheelbase) | 0.220 m | |
| `ly` (half track) | 0.270 m | Derived: `(chassis_y + wheel_width) / 2` |
| **`lx + ly`** | **0.490 m** | **Must match the controller YAML — see below** |
| Total height | 0.461 m | LIDAR top; under the 0.50 m limit |
| Total mass | 25.58 kg | `friction` model; 27.18 kg with `rollers` (32 × 0.05 kg) |

### The one cross-file constant

`kinematics.sum_of_robot_center_projection_on_X_Y_axis` in
`r2_control/config/r2_controllers.yaml` **must equal `lx + ly` = 0.490**. There is no way
to share a value across the xacro/YAML boundary. A mismatch does not error — it silently
scales odometry and strafe response.

## Frames (REP-103 / REP-105)

```
map                          (not published yet; arrives with nav2)
 └─ odom                     published by the EKF in r2_bringup, and by nothing else
     ├─ ground_truth_odom    simulation only; static, = inverse of the spawn pose
     └─ base_footprint       ground-projected origin
         └─ base_link        rigid-body origin, at wheel axle height (z = 0.076)
             ├─ front_left_wheel_link … rear_right_wheel_link   (continuous, axis 0 1 0)
             ├─ imu_link                   (0, 0, 0.050 from base_link)
             ├─ lidar_link                 (0, 0, 0.358 from base_link)
             ├─ mast_link                  (0, 0, 0.265 from base_link)
             └─ chassis_link               (0, 0, 0.100 from base_link)
                 ├─ front_marker_link      visual only; marks "forward"
                 └─ camera_link            (front face, forward-looking)
                     └─ camera_optical_link  REP-103 optical: z fwd, x right, y down
```

13 links in the `friction` model, 45 with `rollers`.

Image, `camera_info` and point-cloud data are stamped with **`camera_optical_link`**, never
`camera_link`.

`ground_truth_odom` is the Gazebo world frame, carried on `/ground_truth/odom`. It is
anchored to `odom` by a static transform in `sim.launch.xml` — without it the frame is an
orphan and every tf2 `MessageFilter` consumer (RViz's Odometry display) queues its messages
and then drops them. It is deliberately a **leaf**: parenting the other way would need no
arithmetic, but it would make `odom` a child of `ground_truth_odom`, and `map → odom` from
nav2 would then give `odom` two parents.

Exactly one node publishes `odom → base_footprint`: the EKF. The controller's
`enable_odom_tf` is `false` for that reason. Two publishers on that transform is the most
common failure in this stack, and shows up as `TF_REPEATED_DATA` warnings.

## Mecanum contact model

Neither candidate drive plugin makes the robot holonomic by itself — Gazebo's `MecanumDrive`
system and `ros2_control` both just command wheel joint velocities. **Holonomic motion has
to come out of the wheel/ground contact**, and there are two ways to get it.

### `contact_model="friction"` (default) — holonomic, and the cheap one

One **sphere** collision per wheel with anisotropic friction: high friction along `fdir1`
(the contacting roller's axle) and low friction across it, with `fdir1` pinned to the
chassis. Cheap — 5 links, 4 joints.

Measured after the fix described below: forward 0.92, strafe 0.92, rotate 0.93, diagonal
0.92, cross-axis error ≈ 0. It also **stops reliably**, which the roller model does not —
see "Stopping behaviour" in `r2_gazebo/README.md`.

> This model was previously documented here as *not* holonomic — "it drives forward but
> strafe, rotate and diagonal are all exactly zero" — and that measurement was real. It was
> caused by three bugs in this repo, not by a limitation of dartsim:
>
> 1. **`fdir1` resolved in the collision-fixed frame**, which rotates with the wheel, so the
>    low-friction direction swept instead of holding a fixed 45°. Averaged over a revolution
>    it lands on ±Y and the wheel grips fore-aft: forward-only motion. Fixed with
>    `<fdir1 gz:expressed_in="base_footprint">`, which dartsim **does** support (verified in
>    gz-physics7 7.6.0). The old claim that it "refuses any other reference frame" was wrong.
> 2. **`mu` and `mu2` were swapped.** A roller grips along its own axle and free-runs across
>    it, so the coefficient along `fdir1` must be the *grip*. It was the slip. That rotates
>    the anisotropy by 90°, which is a mirrored wheel — it still drives forward, so the error
>    survived any test that did not check strafe *direction*.
> 3. **The collision was a cylinder.** On a plane that gives a line contact whose point count
>    flips between one and two, while a friction direction is only meaningful at a single
>    point. It is now a sphere of `wheel_radius`.

Two mechanical requirements, both of which fail loudly rather than silently:

- `xmlns:gz="http://gazebosim.org/schema"` must be declared on the `<robot>` element of
  **every xacro file that uses the prefix**. xacro parses each include as its own XML
  document, so a declaration in `r2.urdf.xacro` does not reach `mecanum_wheel.xacro`;
  without the local one the build fails with `unbound prefix`.
- The friction block must use the nested `<collision><surface><friction><ode>` form. The
  flat `<mu1>`/`<mu2>`/`<fdir1>` shorthand accepted by `<gazebo reference="…">` carries no
  XML attributes, so `gz:expressed_in` cannot be attached to it.

> `mecanum_mu_slip` may now be `0`; measured 0.92/0.92/0.93/0.92, indistinguishable from the
> 0.05 default. The old warning that 0 makes the robot slide frictionlessly applied when slip
> sat in the `mu` slot — DART collapses a contact's friction entirely at `mu = 0`. Slip now
> sits in `mu2`, where 0 is well defined.

### `contact_model="rollers"` — the faithful reference model

Eight free-spinning roller links per wheel at 45°, replacing the friction trick. Physically
correct — the rollers really do roll. Costs 37 links and 36 joints instead of 5 and 4.

**Measured**: forward/strafe/rotate/diagonal all work at 0.87–0.94 efficiency with
negligible cross-axis error, and real-time factor stays ≈1.00 despite the extra links. The
feared cost did not materialise. Numbers and method in `r2_gazebo/README.md`.

The roller rake polarity (`roller_dir` in `r2.urdf.xacro`) was determined by measurement.
With it mirrored the robot drives forward correctly but strafes and yaws in the **wrong
direction**: `vx` is the common mode of all four wheels and is insensitive to rake, while
`vy` and `wz` are differential modes whose sign flips with it. Re-run the holonomic check
if you ever rebuild the roller geometry.

### Roller links and TF

Only the four wheel joints are declared in `<ros2_control>`, so `/joint_states` carries only
those. The 32 roller joints are movable joints with no state source, which means
`robot_state_publisher` cannot compute the `*_wheel_link -> *_roller_*_link` transforms and
**the roller links are absent from the TF tree**. RViz reports them as missing on the
RobotModel display.

This is deliberate. Those transforms have no consumer: no sensor is mounted on a roller and
nothing plans against one, while publishing them adds 32 rapidly-changing frames to `/tf`.

If you want a complete TF tree (or just want to watch the rollers turn in RViz):

```bash
ros2 launch r2_bringup sim.launch.xml contact_model:=rollers publish_roller_states:=true
```

`contact_model:=rollers` is **required**. The default `friction` model has no roller joints
at all, so `publish_roller_states:=true` on its own is a silent no-op — the xacro gates it on
`publish_roller_states and contact_model == 'rollers'`.

That exports each roller as a **state-only** interface (no command interface - they are
driven by ground contact, never by an actuator), so `joint_state_broadcaster` publishes all
36 joints and the tree completes. Verified: 32 roller transforms appear on `/tf`.

> **It has a cost.** With `publish_roller_states:=true` the robot intermittently fails to
> stop when commanded to zero, creeping on instead (5 failures in 11 trials, versus 0 in 15
> with it off). The effect is reproducible; the mechanism is not established. See
> "Stopping behaviour" in `r2_gazebo/README.md`. Use it to inspect the model, not to drive.

## Assumptions

- Rigid planar base, no suspension.
- Wheel joints are ideal velocity sources: no motor dynamics, gearbox backlash or current
  limit. The `command_interface` min/max is a crude stand-in.
- RGB-D is modelled on a RealSense D435-class device but is not device-exact: no IR stereo
  simulation and no per-device distortion model.
- Visual rollers in `friction` mode are decorative only. They have no joints and no
  collisions; they exist so the wheel reads as a mecanum wheel.

## Gotchas

- **Fixed-joint lumping.** sdformat merges every fixed-joint child into `base_footprint`,
  so a `friction` model yields 5 SDF links from 13 URDF ones (37 SDF from 45 with rollers). The sensor *poses* survive correctly and the TF
  frames still come from `robot_state_publisher`, so this is harmless — but it is why
  `lidar_link` does not appear in `gz model` output.
- **`--` in XML comments.** Not legal. A comment containing `--ros-args` or an em-dash makes
  the file fail to parse with a confusing "not well-formed (invalid token)".
- **`<noise>` schema differs by sensor.** `camera.sdf` and `lidar.sdf` take `<type>` as a
  child element; the generic `noise.sdf` used by the IMU takes it as an *attribute*. Using
  the wrong one is silently ignored, leaving a noise-free sensor.
