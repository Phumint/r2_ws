# robocon2027_field — ABU Robocon 2027 game field for Gazebo

Generated from the team's Fusion field CAD (`Robocon 2027 v4`) by `tools/make_field.py`.
Every surface, zone, barrier, stair, ramp and pillar is at its CAD position. Only primitives
(boxes, cylinders, spheres) are used, so there are no mesh paths to break.

## What's inside

> **Vendored into `r2_ws` and modified.** Two changes from the CAD drop, both made in
> `tools/make_field.py` so regenerating keeps them — see "Changes for r2" at the end.

| File | What it is |
|---|---|
| `urdf/robocon2027_field.urdf` | The field: one static link, 87 collision + 109 visual primitives |
| `urdf/objects/*.urdf` | Earth Block (red/blue), Sky Block (red-up/blue-up), Mustika — dynamic, real mass + inertia |
| `worlds/robocon2027.sdf` | Complete world: field + 40 Earth Blocks + 12 Sky Blocks + Mustika at their start positions |
| `worlds/robocon2027_field_only.sdf` | Same field and plugins, no game objects. **70x faster** — see below |
| `launch/field.launch.py` | ROS 2: opens `worlds/robocon2027.sdf` in gz sim |
| `launch/field_urdf.launch.py` | ROS 2: empty gz world, spawns the field URDF + each object URDF; publishes `/field_description` |
| `config/objects.json` | Start pose of every game object (used by the launch file) |
| `docs/field_check_mujoco.png` | Render of the URDF with the physics checks below |

## Run it

No ROS needed (Gazebo Harmonic / Garden / Fortress):

```bash
gz sim worlds/robocon2027.sdf          # Fortress: ign gazebo worlds/robocon2027.sdf
```

Gazebo Classic 11:

```bash
gazebo worlds/robocon2027.sdf
```

ROS 2 (Humble + ros_gz, or Jazzy):

```bash
cd ~/ros2_ws/src && cp -r /path/to/robocon2027_field .
cd ~/ros2_ws && colcon build --packages-select robocon2027_field && source install/setup.bash
ros2 launch robocon2027_field field.launch.py                     # whole world
ros2 launch robocon2027_field field_urdf.launch.py                # URDF version
ros2 launch robocon2027_field field_urdf.launch.py objects:=false # field only
```

## Frame and units

- metres; origin = **south-west corner of the field on the floor surface**; x east, y north, z up
- Red half is x < 5.5, Blue half x > 5.5
- Floor z = 0, Level 1 z = 0.6, Level 2 z = 0.9, Central Pillar top z = 1.7

## Key positions (metres)

| Item | Red | Blue |
|---|---|---|
| Start zones | (0.5–1.2, 0.5–1.2), (1.4–2.1, 0.5–1.2) | (8.9–9.6, 0.5–1.2), (9.8–10.5, 0.5–1.2) |
| Transfer Area (z 0.6) | x 1.5–2.5, y 4.5–5.5 | x 8.5–9.5, y 4.5–5.5 |
| Ramp (foot y 9.0 → top y 5.5) | x 1.5–2.5 | x 8.5–9.5 |
| L1 stairs (4 × 150 mm, foot y 3.3) | x 1.5–2.5 | x 8.5–9.5 |
| L2 stairs (2 × 150 mm) | x 3.4–4.0, y 5.0–6.0 | x 7.0–7.6, y 5.0–6.0 |
| Storage Area | x 1.2–3.2, y 9.6–10.6 | x 7.8–9.8, y 9.6–10.6 |
| Mustika Pillar | centre (5.5, 9.3), h 0.5 | shared |
| Central Pillar (on L2) | centre (5.5, 5.5), top z 1.7 | shared |

## How it was checked

The URDF was loaded in MuJoCo and test objects were dropped on it:

| Test | Result |
|---|---|
| Mustika (Ø210) dropped on the Mustika Pillar | rests in the cup, centre z **0.554** (CAD: 0.554) |
| Mustika dropped on the Central Pillar | rests in the cup, centre z **1.754** (= 1.7 + 0.054) |
| Box dropped on the ramp | sits on the 9.73° surface and stays there |
| Box dropped on L2 | rests at z 0.9 |

All XML files parse, and every model, visual and collision name is unique. The launch files
have not been run here (no ROS on the machine that generated them).

## What is simplified

- **Painted zones** (start, storage, shared, retry, building spots) are visual only, 0.5 mm above
  the floor, so robots don't bump over 1 mm steps that the real field doesn't have.
- **Pillar cups** (Ø180 × 100 deep) are a ring of 24 boxes; the Mustika sits on their inner
  24-gon edge, within 0.4 mm of a true circle.
- **Ramps** are one tilted box whose top face is the ramp surface; the part of it that would
  stick out ends up under the floor or inside the Transfer Area block.
- **Friction** is assumed: field 0.6, cardboard blocks 0.5, ball 0.6. Measure yours and edit
  `MU_FIELD` / `MU_CARD` in `tools/make_field.py`.
- **Earth Blocks** are placed exactly as in the CAD: 10 two-high stacks per side, touching.
- Block masses are the rulebook nominal values (Earth 0.60 kg, Sky 0.22 kg, Mustika 0.27 kg).

## Regenerating

```bash
python tools/make_field.py    # reads tools/field_dump.json, rewrites urdf/, worlds/, config/
```

`tools/field_dump.json` is the body list exported from the Fusion field document.


## Changes for r2

This copy differs from the CAD drop in two ways. Both are emitted by `tools/make_field.py`,
so `python tools/make_field.py` keeps them; neither touches the field geometry, and the
regenerated `robocon2027.sdf` is byte-identical to the original apart from these.

### 1. World system plugins (required)

The generated world declared **no Gazebo system plugins at all**. gz sim does not error on
that — it falls back to a built-in default set of Physics, UserCommands and SceneBroadcaster,
which has neither `Sensors` nor `Imu`. A robot spawned into such a world produces no LIDAR,
no camera and no IMU data, silently. `make_field.py` now emits the full block
(`WORLD_PLUGINS`), matching `r2_gazebo/worlds/empty.sdf`. The SDF version was also raised
from 1.6 to 1.10 for Gazebo Harmonic parity.

`r2_bringup`'s `test_worlds_load_the_plugins_the_sensors_need` fails if any world loses them.

### 2. A field-only world (performance)

Measured on this workspace, headless, dartsim, no robot:

| world | RTF |
|---|---|
| `robocon2027_field_only.sdf` | **1.00** |
| `robocon2027.sdf` | **0.014** |

The 53 game objects start in 84 pairs of exact zero-gap contact — the Earth Blocks are ten
touching two-high stacks per side, flush with the floor, exactly as the CAD places them —
and the simulator spends its whole budget resolving those resting contacts at the 1 ms step.
Roughly 70 s of wall time per second of sim.

Nothing is wrong with the field: the 87 static field primitives alone run at RTF 1.00. If
you need the objects to settle faster, giving each block a millimetre of clearance in
`field_dump.json` is the place to start, not the field geometry.

**Do not switch physics engine for this.** `bullet` and `bullet-featherstone` both run the
full world at RTF 1.00, but neither reproduces the anisotropic `fdir1` friction a mecanum
base needs: measured strafe efficiency on the empty world drops to 0.00 with the `friction`
contact model and −0.02 with `rollers`, i.e. the robot cannot strafe at all. dartsim is the
only engine r2 drives correctly on.

### Using it with r2

```bash
ros2 launch r2_bringup field_sim.launch.xml objects:=false
```

See `r2_bringup/README.md` for the start poses, the rulebook reasoning behind them, and the
measured drive quality on this field.
