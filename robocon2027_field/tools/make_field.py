"""Generate the ABU Robocon 2027 game field for Gazebo from the team's Fusion field CAD dump.

Input : field_dump.json (every body of the Fusion doc 'Robocon 2027 v4': name, bounding box, colour)
Output: urdf/robocon2027_field.urdf       static field (one link, primitives only)
        urdf/objects/*.urdf                earth / sky block and Mustika, dynamic
        worlds/robocon2027.sdf            complete world: field + all 52 blocks + Mustika

Frame: metres, origin = SW corner of the field on the floor surface (same as the CAD),
x east (red half x < 5.5), y north, z up.
"""
import json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
D = json.load(open(os.path.join(HERE, 'field_dump.json'), encoding='utf-8'))
ROWS = [r for r in D['rows'] if r['vis']]

MU_FIELD = 0.6      # painted plywood (assumed; measure on your field)
MU_CARD = 0.5       # cardboard with stickers (assumed)
SLAB_LIFT = 0.0005  # painted zones are drawn 0.5 mm above their surface, visual only

# Gazebo system plugins emitted into worlds/robocon2027.sdf.
#
# NOT boilerplate, and not optional. A world that declares no plugins at all makes
# gz sim fall back to its built-in default set - Physics, UserCommands and
# SceneBroadcaster - which has NEITHER Sensors NOR Imu. A robot spawned into such a
# world produces no LIDAR, no camera and no IMU data, silently and with no error
# message. That is the single most expensive way to lose an afternoon here.
#
# Physics          : steps the simulation
# UserCommands     : owns the /world/<name>/create service that `ros_gz_sim create`
#                    calls to spawn a robot. Without it the spawner just times out.
# SceneBroadcaster : scene state for the GUI and for gz topic consumers
# Contact          : contact reporting
# Sensors          : REQUIRED by gpu_lidar and rgbd_camera
# Imu              : REQUIRED by the imu sensor
#
# Keep this list in step with r2_gazebo/worlds/empty.sdf, which carries the same
# block and the same reasoning.
WORLD_PLUGINS = [
    '    <plugin filename="gz-sim-physics-system"',
    '            name="gz::sim::systems::Physics"/>',
    '    <plugin filename="gz-sim-user-commands-system"',
    '            name="gz::sim::systems::UserCommands"/>',
    '    <plugin filename="gz-sim-scene-broadcaster-system"',
    '            name="gz::sim::systems::SceneBroadcaster"/>',
    '    <plugin filename="gz-sim-contact-system"',
    '            name="gz::sim::systems::Contact"/>',
    '',
    '    <!-- Required by the gpu_lidar and rgbd_camera sensors. -->',
    '    <plugin filename="gz-sim-sensors-system"',
    '            name="gz::sim::systems::Sensors">',
    '      <render_engine>ogre2</render_engine>',
    '    </plugin>',
    '',
    '    <!-- Required by the imu sensor. -->',
    '    <plugin filename="gz-sim-imu-system"',
    '            name="gz::sim::systems::Imu"/>',
]


def m(v):
    return v / 1000.0


def rgba(rgb, a=1.0):
    return '%.3f %.3f %.3f %.1f' % (rgb[0] / 255, rgb[1] / 255, rgb[2] / 255, a)


def box_from_bb(bb):
    x0, y0, z0, x1, y1, z1 = [m(v) for v in bb]
    return ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0)


class Shape:
    """one primitive: kind box|cylinder|sphere, pose (xyz, rpy), size, colour, visual/collision flags"""
    def __init__(self, name, kind, xyz, size, rgb, rpy=(0, 0, 0), vis=True, col=True):
        self.name, self.kind, self.xyz, self.size, self.rgb, self.rpy = name, kind, xyz, size, rgb, rpy
        self.vis, self.col = vis, col


def cup_pillar(name, bb, rgb, cup_d=0.180, cup_depth=0.100, n=24):
    """pillar with a concave receptacle: solid cylinder below the cup + a ring of n boxes whose
    flat inner faces form a regular n-gon of apothem cup_d/2 (the Mustika rests on its rim)"""
    (cx, cy, _), (dx, _, h) = box_from_bb(bb)
    z0 = m(bb[2])
    R = dx / 2
    out = [Shape(name + ' body', 'cylinder', (cx, cy, z0 + (h - cup_depth) / 2), (R, h - cup_depth), rgb)]
    ri = cup_d / 2
    t = (R - ri)
    w = 2 * R * math.tan(math.pi / n)
    for k in range(n):
        a = 2 * math.pi * k / n
        rc = ri + t / 2
        out.append(Shape('%s rim %02d' % (name, k), 'box',
                         (cx + rc * math.cos(a), cy + rc * math.sin(a), z0 + h - cup_depth / 2),
                         (t, w, cup_depth), rgb, rpy=(0, 0, a)))
    return out


def ramp(name, bb, rgb):
    """wedge rising from the foot (larger y, z 0) to the Transfer Area edge (smaller y, z top):
    one box whose top face is the ramp surface; it is thick enough to fill the wedge (the part
    that ends up under the floor or inside the Transfer Area block is hidden)"""
    x0, y0, z0, x1, y1, z1 = [m(v) for v in bb]
    H, Ly = z1 - z0, y1 - y0
    th = math.atan2(H, Ly)
    L = math.hypot(H, Ly)
    T = H * math.cos(th)
    top_mid = ((x0 + x1) / 2, (y0 + y1) / 2, z0 + H / 2)
    nrm = (0.0, math.sin(th), math.cos(th))       # surface normal of a surface falling towards +y
    c = (top_mid[0], top_mid[1] - T / 2 * nrm[1], top_mid[2] - T / 2 * nrm[2])
    return [Shape(name, 'box', c, (x1 - x0, L, T), rgb, rpy=(-th, 0, 0))]


def field_shapes():
    S = []
    for r in ROWS:
        nm = r['name']
        if r['occ'].startswith('07 Game Objects'):
            continue
        rgb = r['rgb'] or [180, 180, 180]
        if r['nf'] == 6 and r['fill'] > 0.99:
            xyz, size = box_from_bb(r['bb'])
            if '[SLAB]' in nm:          # painted zone: visual only, sits just above its surface
                S.append(Shape(nm, 'box', (xyz[0], xyz[1], m(r['bb'][2]) + SLAB_LIFT), (size[0], size[1], 0.0006), rgb, col=False))
            else:
                S.append(Shape(nm, 'box', xyz, size, rgb))
        elif 'Ramp' in nm:
            S += ramp(nm, r['bb'], rgb)
        elif 'Pillar' in nm:
            S += cup_pillar(nm, r['bb'], rgb)
        else:
            raise SystemExit('unhandled body shape: %s %s' % (nm, r))
    return S


def objects():
    """game objects at their CAD start positions"""
    out = []
    for r in ROWS:
        if not r['occ'].startswith('07 Game Objects'):
            continue
        xyz, size = box_from_bb(r['bb'])
        nm = r['name']
        if nm.startswith('Earth Block'):
            out.append(('earth_block_' + ('red' if 'Red' in nm else 'blue'), nm, xyz))
        elif nm.startswith('Sky Block'):
            out.append(('sky_block_' + ('red_up' if '(R up)' in nm else 'blue_up'), nm, xyz))
        elif nm == 'Mustika':
            out.append(('mustika', nm, xyz))
    return out


RED, BLUE, CARD, BALL = [223, 34, 34], [50, 0, 255], [205, 170, 125], [235, 215, 60]
OBJ = {
    # name: (kind, size, mass kg, colour parts)
    'earth_block_red': dict(size=0.350, kg=0.600, top=RED, bottom=RED, side=RED),
    'earth_block_blue': dict(size=0.350, kg=0.600, top=BLUE, bottom=BLUE, side=BLUE),
    'sky_block_red_up': dict(size=0.200, kg=0.220, top=RED, bottom=BLUE, side=CARD),
    'sky_block_blue_up': dict(size=0.200, kg=0.220, top=BLUE, bottom=RED, side=CARD),
    'mustika': dict(radius=0.105, kg=0.270, colour=BALL),
}


def safe(s):
    return ''.join(c if c.isalnum() else '_' for c in s).strip('_').lower()


# ---------------------------------------------------------------------------------- URDF
def geom_urdf(s):
    if s.kind == 'box':
        return '<box size="%.5f %.5f %.5f"/>' % s.size
    if s.kind == 'cylinder':
        return '<cylinder radius="%.5f" length="%.5f"/>' % s.size
    return '<sphere radius="%.5f"/>' % s.size[0]


def field_urdf(S):
    cols = {}
    for s in S:
        cols.setdefault(tuple(s.rgb), 'c_%d_%d_%d' % tuple(s.rgb))
    L = ['<?xml version="1.0"?>',
         '<!-- ABU Robocon 2027 game field. Generated by tools/make_field.py from the Fusion field CAD. -->',
         '<!-- Frame: metres, origin = SW field corner on the floor surface, x east, y north, z up. -->',
         '<robot name="robocon2027_field">']
    for rgb, n in cols.items():
        L.append('  <material name="%s"><color rgba="%s"/></material>' % (n, rgba(rgb)))
    L.append('  <link name="field">')
    L.append('    <inertial><origin xyz="5.5 5.5 0.2"/><mass value="1000"/>'
             '<inertia ixx="1e4" ixy="0" ixz="0" iyy="1e4" iyz="0" izz="1e4"/></inertial>')
    for s in S:
        org = '<origin xyz="%.5f %.5f %.5f" rpy="%.6f %.6f %.6f"/>' % (s.xyz + s.rpy)
        if s.vis:
            L.append('    <visual name="v_%s">%s<geometry>%s</geometry><material name="%s"/></visual>'
                     % (safe(s.name), org, geom_urdf(s), cols[tuple(s.rgb)]))
        if s.col:
            L.append('    <collision name="c_%s">%s<geometry>%s</geometry></collision>' % (safe(s.name), org, geom_urdf(s)))
    L.append('  </link>')
    L.append('  <gazebo><static>true</static></gazebo>')
    L.append('  <gazebo reference="field"><mu1>%.2f</mu1><mu2>%.2f</mu2></gazebo>' % (MU_FIELD, MU_FIELD))
    L.append('</robot>')
    return '\n'.join(L) + '\n'


def obj_urdf(name):
    o = OBJ[name]
    L = ['<?xml version="1.0"?>', '<robot name="%s">' % name]
    if name == 'mustika':
        r, kg = o['radius'], o['kg']
        I = 2.0 / 3.0 * kg * r * r         # thin-walled ball
        L.append('  <material name="ball"><color rgba="%s"/></material>' % rgba(o['colour']))
        L.append('  <link name="%s">' % name)
        L.append('    <inertial><mass value="%.3f"/><inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/></inertial>' % (kg, I, I, I))
        L.append('    <visual><geometry><sphere radius="%.4f"/></geometry><material name="ball"/></visual>' % r)
        L.append('    <collision><geometry><sphere radius="%.4f"/></geometry></collision>' % r)
        L.append('  </link>')
        L.append('  <gazebo reference="%s"><mu1>0.6</mu1><mu2>0.6</mu2></gazebo>' % name)
    else:
        a, kg = o['size'], o['kg']
        I = kg * a * a / 6.0
        L.append('  <material name="top"><color rgba="%s"/></material>' % rgba(o['top']))
        L.append('  <material name="bottom"><color rgba="%s"/></material>' % rgba(o['bottom']))
        L.append('  <material name="side"><color rgba="%s"/></material>' % rgba(o['side']))
        L.append('  <link name="%s">' % name)
        L.append('    <inertial><mass value="%.3f"/><inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/></inertial>' % (kg, I, I, I))
        L.append('    <visual name="body"><geometry><box size="%.4f %.4f %.4f"/></geometry><material name="side"/></visual>' % (a, a, a - 0.002))
        L.append('    <visual name="top"><origin xyz="0 0 %.4f"/><geometry><box size="%.4f %.4f 0.001"/></geometry><material name="top"/></visual>' % (a / 2 - 0.0005, a, a))
        L.append('    <visual name="bottom"><origin xyz="0 0 %.4f"/><geometry><box size="%.4f %.4f 0.001"/></geometry><material name="bottom"/></visual>' % (-a / 2 + 0.0005, a, a))
        L.append('    <collision><geometry><box size="%.4f %.4f %.4f"/></geometry></collision>' % (a, a, a))
        L.append('  </link>')
        L.append('  <gazebo reference="%s"><mu1>%.2f</mu1><mu2>%.2f</mu2></gazebo>' % (name, MU_CARD, MU_CARD))
    L.append('</robot>')
    return '\n'.join(L) + '\n'


# ---------------------------------------------------------------------------------- SDF
def mat_sdf(rgb):
    c = rgba(rgb)
    return '<material><ambient>%s</ambient><diffuse>%s</diffuse><specular>0.1 0.1 0.1 1</specular></material>' % (c, c)


def geom_sdf(s):
    if s.kind == 'box':
        return '<box><size>%.5f %.5f %.5f</size></box>' % s.size
    if s.kind == 'cylinder':
        return '<cylinder><radius>%.5f</radius><length>%.5f</length></cylinder>' % s.size
    return '<sphere><radius>%.5f</radius></sphere>' % s.size[0]


def surface_sdf(mu):
    return ('<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode><bullet><friction>%.2f</friction>'
            '<friction2>%.2f</friction2></bullet></friction></surface>' % (mu, mu, mu, mu))


def obj_model_sdf(kind, inst, xyz):
    o = OBJ[kind]
    L = ['    <model name="%s">' % inst, '      <pose>%.5f %.5f %.5f 0 0 0</pose>' % xyz, '      <link name="link">']
    if kind == 'mustika':
        r, kg = o['radius'], o['kg']
        I = 2.0 / 3.0 * kg * r * r
        L.append('        <inertial><mass>%.3f</mass><inertia><ixx>%.6g</ixx><iyy>%.6g</iyy><izz>%.6g</izz>'
                 '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>' % (kg, I, I, I))
        L.append('        <collision name="c"><geometry><sphere><radius>%.4f</radius></sphere></geometry>%s</collision>' % (r, surface_sdf(0.6)))
        L.append('        <visual name="v"><geometry><sphere><radius>%.4f</radius></sphere></geometry>%s</visual>' % (r, mat_sdf(o['colour'])))
    else:
        a, kg = o['size'], o['kg']
        I = kg * a * a / 6.0
        L.append('        <inertial><mass>%.3f</mass><inertia><ixx>%.6g</ixx><iyy>%.6g</iyy><izz>%.6g</izz>'
                 '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>' % (kg, I, I, I))
        L.append('        <collision name="c"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry>%s</collision>' % (a, a, a, surface_sdf(MU_CARD)))
        L.append('        <visual name="body"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry>%s</visual>' % (a, a, a - 0.002, mat_sdf(o['side'])))
        L.append('        <visual name="top"><pose>0 0 %.4f 0 0 0</pose><geometry><box><size>%.4f %.4f 0.001</size></box></geometry>%s</visual>' % (a / 2 - 0.0005, a, a, mat_sdf(o['top'])))
        L.append('        <visual name="bottom"><pose>0 0 %.4f 0 0 0</pose><geometry><box><size>%.4f %.4f 0.001</size></box></geometry>%s</visual>' % (-a / 2 + 0.0005, a, a, mat_sdf(o['bottom'])))
    L += ['      </link>', '    </model>']
    return '\n'.join(L)


def world_sdf(S, objs):
    """Complete world: static field plus every game object at its start pose.

    Pass objs=[] for the field-only variant. That is not a cosmetic difference:
    the 53 game objects start in 84 pairs of exact zero-gap contact (the Earth
    Blocks are 10 touching two-high stacks per side, flush with the floor), and
    resolving those resting contacts every 1 ms step is what the simulator spends
    its time on. Measured on this workspace, headless, no robot:

        field only              RTF 1.00
        field + 53 objects      RTF 0.014        (about 70x slower)

    Switching physics engine does NOT fix this. bullet and bullet-featherstone
    both run the full world at RTF 1.00, but neither reproduces the anisotropic
    fdir1 friction a mecanum base needs: measured strafe efficiency drops to 0.00
    under bullet-featherstone with the friction contact model, and to -0.02 with
    the rollers model, i.e. the robot cannot strafe at all. dartsim is the only
    engine this robot drives correctly on, so the objects are the lever, not the
    engine.
    """
    L = ['<?xml version="1.0"?>',
         '<!-- ABU Robocon 2027 game field%s. Generated by tools/make_field.py from the Fusion field CAD. -->'
         % (' + game objects' if objs else ' (field only, no game objects)'),
         '<sdf version="1.10">', '  <world name="robocon2027">',
         '    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>',
         '    <gravity>0 0 -9.81</gravity>',
         '    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.8 0.85 1</background><shadows>true</shadows></scene>',
         '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>5.5 5.5 20 0 0 0</pose>'
         '<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.4 -1</direction></light>',
         '']
    L += WORLD_PLUGINS
    L += ['',
          '    <model name="robocon2027_field">', '      <static>true</static>', '      <link name="field">']
    for i, s in enumerate(S):
        pose = '<pose>%.5f %.5f %.5f %.6f %.6f %.6f</pose>' % (s.xyz + s.rpy)
        nm = '%03d_%s' % (i, safe(s.name))
        if s.col:
            L.append('        <collision name="c%s">%s<geometry>%s</geometry>%s</collision>' % (nm, pose, geom_sdf(s), surface_sdf(MU_FIELD)))
        if s.vis:
            L.append('        <visual name="v%s">%s<geometry>%s</geometry>%s</visual>' % (nm, pose, geom_sdf(s), mat_sdf(s.rgb)))
    L += ['      </link>', '    </model>']
    for kind, nm, xyz in objs:
        L.append(obj_model_sdf(kind, safe(nm), xyz))
    L += ['  </world>', '</sdf>']
    return '\n'.join(L) + '\n'


def main():
    S = field_shapes()
    objs = objects()
    os.makedirs(os.path.join(PKG, 'urdf', 'objects'), exist_ok=True)
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    open(os.path.join(PKG, 'urdf', 'robocon2027_field.urdf'), 'w', newline='\n').write(field_urdf(S))
    for k in OBJ:
        open(os.path.join(PKG, 'urdf', 'objects', k + '.urdf'), 'w', newline='\n').write(obj_urdf(k))
    open(os.path.join(PKG, 'worlds', 'robocon2027.sdf'), 'w', newline='\n').write(world_sdf(S, objs))
    # Field-only variant. Same field, same plugins, none of the 53 game objects.
    # See world_sdf's docstring for why this exists and what skipping it costs.
    open(os.path.join(PKG, 'worlds', 'robocon2027_field_only.sdf'), 'w', newline='\n').write(world_sdf(S, []))
    # spawn list for the ROS 2 launch file (instance name, urdf, x, y, z)
    spawn = [dict(name=safe(nm), urdf=kind + '.urdf', xyz=[round(v, 5) for v in xyz]) for kind, nm, xyz in objs]
    json.dump(spawn, open(os.path.join(PKG, 'config', 'objects.json'), 'w'), indent=1)
    print('field primitives: %d (%d collision), objects: %d' % (len(S), sum(s.col for s in S), len(objs)))
    print('worlds: robocon2027.sdf (full), robocon2027_field_only.sdf (no game objects)')


if __name__ == '__main__':
    main()
