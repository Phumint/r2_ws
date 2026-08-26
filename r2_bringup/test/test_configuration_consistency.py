"""Integration checks across r2_description, r2_control and r2_gazebo.

These packages carry no application logic to unit-test -- they are xacro, YAML and
launch files. What can go wrong is the seams between them, and every assertion
here corresponds to a mistake that is silent at runtime: nothing errors, the robot
just drives or estimates wrongly.
"""

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

DESCRIPTION = Path(get_package_share_directory('r2_description'))
CONTROL = Path(get_package_share_directory('r2_control'))
GAZEBO = Path(get_package_share_directory('r2_gazebo'))
BRINGUP = Path(get_package_share_directory('r2_bringup'))

TOP_LEVEL_XACRO = DESCRIPTION / 'urdf' / 'r2.urdf.xacro'


def expand(contact_model='friction', **args):
    """Run xacro and return the parsed URDF root."""
    xacro = shutil.which('xacro')
    assert xacro, 'xacro not on PATH'
    cmd = [xacro, str(TOP_LEVEL_XACRO), f'contact_model:={contact_model}']
    cmd += [f'{k}:={v}' for k, v in args.items()]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, f'xacro failed:\n{out.stderr}'
    return ET.fromstring(out.stdout)


def joint_origin(root, name):
    for j in root.iter('joint'):
        if j.get('name') == name:
            xyz = j.find('origin').get('xyz').split()
            return tuple(float(v) for v in xyz)
    raise AssertionError(f'joint {name} not found')


@pytest.fixture(scope='module')
def urdf():
    return expand()


@pytest.mark.parametrize('contact_model', ['rollers', 'friction'])
def test_xacro_expands_and_urdf_is_well_formed(contact_model):
    root = expand(contact_model)
    links = {l.get('name') for l in root.iter('link')}
    for required in ('base_footprint', 'base_link', 'imu_link', 'lidar_link',
                     'camera_link', 'camera_optical_link'):
        assert required in links, f'{required} missing from {contact_model} model'


def test_all_four_wheels_present(urdf):
    joints = {j.get('name') for j in urdf.iter('joint')}
    for corner in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        assert f'{corner}_wheel_joint' in joints


def test_kinematics_constant_matches_controller_config(urdf):
    """lx + ly in the URDF must equal the controller's mecanum IK constant.

    There is no way to share a value across the xacro/YAML boundary. A mismatch
    does not raise anything at runtime -- it silently scales both odometry and
    strafe response, which is very hard to spot by eye.
    """
    fl = joint_origin(urdf, 'front_left_wheel_joint')
    rr = joint_origin(urdf, 'rear_right_wheel_joint')
    lx, ly = abs(fl[0]), abs(fl[1])

    # Wheels must be symmetric about base_link, or lx/ly are meaningless.
    assert math.isclose(lx, abs(rr[0]), abs_tol=1e-9)
    assert math.isclose(ly, abs(rr[1]), abs_tol=1e-9)

    cfg = yaml.safe_load((CONTROL / 'config' / 'r2_controllers.yaml').read_text())
    kin = cfg['mecanum_drive_controller']['ros__parameters']['kinematics']

    assert math.isclose(kin['sum_of_robot_center_projection_on_X_Y_axis'],
                        lx + ly, abs_tol=1e-6), (
        f'controller says {kin["sum_of_robot_center_projection_on_X_Y_axis"]}, '
        f'URDF geometry gives lx+ly={lx + ly}')


def effective_rolling_radius(root, contact_model):
    """Distance from the wheel axle to the outermost point that touches ground.

    This is NOT simply the hub collision radius. Under contact_model="rollers"
    the hub is deliberately inset so the rollers carry the load, so its collision
    radius (0.044) is much smaller than the radius the robot actually rolls on
    (0.076). Getting this wrong scales odometry by ~40%.
    """
    if contact_model == 'friction':
        # A sphere, not a cylinder: the anisotropic friction model needs a single
        # contact point that keeps its character as the wheel turns. See the
        # collision block in mecanum_wheel.xacro.
        radius = None
        for link in root.iter('link'):
            if link.get('name') != 'front_left_wheel_link':
                continue
            for coll in link.findall('collision'):
                sph = coll.find('geometry/sphere')
                if sph is not None:
                    radius = max(radius or 0.0, float(sph.get('radius')))
        # Assert rather than fall through to 0.0. A silent zero here reports
        # itself as an odometry mismatch, which sends you to the wrong file.
        assert radius is not None, (
            'front_left_wheel_link has no sphere collision; the friction '
            'contact model geometry changed and this helper was not updated')
        return radius

    # rollers: axle -> roller centre, plus the roller's own radius.
    best = 0.0
    roller_radius = {}
    for link in root.iter('link'):
        name = link.get('name') or ''
        if not name.startswith('front_left_roller_'):
            continue
        cyl = link.find('collision/geometry/cylinder')
        assert cyl is not None, f'{name} has no cylinder collision'
        roller_radius[name.replace('_link', '')] = float(cyl.get('radius'))

    assert roller_radius, 'no rollers found on front_left wheel'
    for joint in root.iter('joint'):
        name = joint.get('name') or ''
        if not name.startswith('front_left_roller_'):
            continue
        xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
        offset = math.hypot(xyz[0], xyz[2])  # radial, in the wheel's XZ plane
        best = max(best, offset + roller_radius[name.replace('_joint', '')])
    return best


@pytest.mark.parametrize('contact_model', ['rollers', 'friction'])
def test_effective_rolling_radius_matches_controller_config(contact_model):
    """Controller wheel radius must match the radius the robot actually rolls on."""
    root = expand(contact_model)
    radius = effective_rolling_radius(root, contact_model)

    cfg = yaml.safe_load((CONTROL / 'config' / 'r2_controllers.yaml').read_text())
    kin = cfg['mecanum_drive_controller']['ros__parameters']['kinematics']
    assert math.isclose(kin['wheels_radius'], radius, abs_tol=1e-6), (
        f'controller says wheels_radius={kin["wheels_radius"]}, but the '
        f'{contact_model} geometry rolls on {radius}')


def test_only_the_ekf_publishes_odom_tf():
    """Two publishers on odom -> base_footprint is the classic failure here."""
    cfg = yaml.safe_load((CONTROL / 'config' / 'r2_controllers.yaml').read_text())
    params = cfg['mecanum_drive_controller']['ros__parameters']
    assert params['enable_odom_tf'] is False, (
        'mecanum_drive_controller must not publish odom TF; the EKF owns it')

    ekf = yaml.safe_load((BRINGUP / 'config' / 'ekf.yaml').read_text())
    ekf_params = ekf['ekf_filter_node']['ros__parameters']
    assert ekf_params['publish_tf'] is True
    assert ekf_params['odom_frame'] == params['odom_frame_id']
    assert ekf_params['base_link_frame'] == params['base_frame_id']


def test_ekf_consumes_the_controller_odometry_topic():
    ekf = yaml.safe_load((BRINGUP / 'config' / 'ekf.yaml').read_text())
    params = ekf['ekf_filter_node']['ros__parameters']
    assert params['odom0'] == '/mecanum_drive_controller/odometry'
    assert params['imu0'] == '/imu/data'
    # vx, vy from the wheels; vyaw from the gyro. See r2_bringup/README.md.
    assert params['odom0_config'][6:8] == [True, True], 'wheel vx/vy must be fused'
    assert params['odom0_config'][11] is False, 'wheel vyaw must NOT be fused'
    assert params['imu0_config'][11] is True, 'gyro vyaw must be fused'
    assert params['imu0_config'][3:6] == [False, False, False], (
        'absolute IMU orientation must not be fused; Gazebo derives it from '
        'ground truth and it would hide real drift')


def test_bridge_frame_ids_exist_in_the_description(urdf):
    """A bridged frame_id that is not a real link breaks TF for that sensor."""
    links = {l.get('name') for l in urdf.iter('link')}
    entries = yaml.safe_load((GAZEBO / 'config' / 'ros_gz_bridge.yaml').read_text())
    for entry in entries:
        frame = entry.get('frame_id')
        # ground_truth_odom is the Gazebo world frame, not a robot link. It is
        # anchored into the tree by a static transform in sim.launch.xml.
        if not frame or frame == 'ground_truth_odom':
            continue
        assert frame in links, (
            f'{entry["ros_topic_name"]} is bridged with frame_id "{frame}", '
            f'which is not a link in the robot description')


def test_non_link_bridged_frames_are_anchored_by_a_static_transform():
    """Every bridged frame_id that is not a robot link must be anchored to the tree.

    test_bridge_frame_ids_exist_in_the_description exempts ground_truth_odom
    because it is a world frame rather than a link. Nothing used to check that
    the exemption was safe, and it was not: the frame was an orphan, so RViz's
    Odometry display queued every ground-truth message waiting for a transform
    that never arrived and then dropped them. Assert the anchor exists.
    """
    links = {l.get('name') for l in expand().iter('link')}
    entries = yaml.safe_load((GAZEBO / 'config' / 'ros_gz_bridge.yaml').read_text())
    non_link_frames = {
        e['frame_id'] for e in entries
        if e.get('frame_id') and e['frame_id'] not in links
    }

    launch = (BRINGUP / 'launch' / 'sim.launch.xml').read_text()
    root = ET.fromstring(launch)
    anchored = set()
    for node in root.iter('node'):
        if node.get('exec') != 'static_transform_publisher':
            continue
        args = (node.get('args') or '').split()
        if '--child-frame-id' in args:
            anchored.add(args[args.index('--child-frame-id') + 1])

    missing = non_link_frames - anchored
    assert not missing, (
        f'bridged frame(s) {sorted(missing)} are not robot links and are not '
        f'anchored by a static_transform_publisher in sim.launch.xml, so any '
        f'tf2 MessageFilter consumer will queue and drop their messages')


@pytest.mark.parametrize('pkg', [DESCRIPTION, CONTROL, GAZEBO, BRINGUP])
def test_xml_files_are_well_formed(pkg):
    """Catches the '--' in XML comments trap.

    A double hyphen inside an XML comment (an em-dash, or a pasted '--ros-args')
    is illegal and makes the file fail to parse with a confusing
    "not well-formed (invalid token)" error at launch time rather than build time.
    """
    for path in sorted(list(pkg.rglob('*.xml')) + list(pkg.rglob('*.xacro'))
                       + list(pkg.rglob('*.sdf'))):
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            pytest.fail(f'{path} is not well-formed XML: {exc}')
