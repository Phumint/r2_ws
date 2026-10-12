"""Sanity check: load the field URDF in MuJoCo, drop test objects, render views."""
import mujoco, numpy as np, imageio, os, json, math
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
u = open(os.path.join(PKG, 'urdf', 'robocon2027_field.urdf')).read()
u = u.replace('<robot name="robocon2027_field">', '<robot name="robocon2027_field"><mujoco><compiler discardvisual="false" balanceinertia="true"/></mujoco>')
tmp = os.path.join(PKG, 'tools', '_tmp_field.urdf'); open(tmp, 'w').write(u)
spec = mujoco.MjSpec.from_file(tmp)
spec.visual.global_.offwidth, spec.visual.global_.offheight = 1600, 1000
spec.worldbody.add_light(pos=[5.5, 5.5, 15], dir=[-0.2, 0.3, -1], diffuse=[0.8, 0.8, 0.8])
tests = {
  'ball_in_mustika_pillar': ('sphere', [0.105], [5.5, 9.3, 0.75], 0.27),
  'ball_in_central_pillar': ('sphere', [0.105], [5.5, 5.5, 1.95], 0.27),
  'box_on_ramp': ('box', [0.1, 0.1, 0.05], [2.0, 7.25, 0.8], 1.0),
  'box_on_l2': ('box', [0.1, 0.1, 0.1], [5.0, 4.5, 1.2], 0.6),
}
for n, (k, size, pos, kg) in tests.items():
    b = spec.worldbody.add_body(name=n, pos=pos)
    b.add_freejoint()
    g = b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE if k == 'sphere' else mujoco.mjtGeom.mjGEOM_BOX, size=size + [0] * (3 - len(size)), mass=kg, rgba=[1, 1, 0, 1], friction=[0.6, 0.02, 0.001])
m = spec.compile(); d = mujoco.MjData(m)
print('ngeom', m.ngeom)
for i in range(4000): mujoco.mj_step(m, d)
for n in tests:
    p = d.body(n).xpos; v = d.body(n).cvel
    print('%-24s rest at %s  |v|=%.4f' % (n, np.round(p, 4), np.linalg.norm(v[3:])))
r = mujoco.Renderer(m, 1000, 1600)
cam = mujoco.MjvCamera(); cam.lookat[:] = [5.5, 5.5, 0.3]
imgs = []
for az, el, dist in ((-60, -35, 16), (90, -89.5, 15)):
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    r.update_scene(d, cam); imgs.append(r.render())
cam.lookat[:] = [2.0, 5.0, 0.4]; cam.azimuth, cam.elevation, cam.distance = 180, -20, 6.5
r.update_scene(d, cam); imgs.append(r.render())
cam.lookat[:] = [5.5, 5.5, 1.6]; cam.azimuth, cam.elevation, cam.distance = -45, -30, 2.5
r.update_scene(d, cam); imgs.append(r.render())
sheet = np.concatenate([np.concatenate(imgs[:2], 1), np.concatenate(imgs[2:], 1)], 0)[::2, ::2]
imageio.imwrite(os.path.join(PKG, 'docs', 'field_check_mujoco.png'), sheet)
os.remove(tmp)
