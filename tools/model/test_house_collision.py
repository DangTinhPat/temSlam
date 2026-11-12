"""Regression: wall openings stay traversable, while solid walls still collide.

Run with the indoor-map requirements and mujoco installed:
    python tools/model/test_house_collision.py
"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from import_small_house import DEFAULT_OUTPUT, DEFAULT_SOURCE, wall_collision_meshes

root = ET.parse(DEFAULT_OUTPUT).getroot()
wall_geoms = [g for g in root.findall('worldbody/geom')
              if g.get('name', '').startswith('house_wall_collision_')]
mesh_names = {g.get('mesh') for g in wall_geoms}
scene = ET.Element('mujoco')
asset = ET.SubElement(scene, 'asset')
world = ET.SubElement(scene, 'worldbody')
for mesh in root.findall('asset/mesh'):
    if mesh.get('name') in mesh_names:
        asset.append(mesh)
for geom in wall_geoms:
    world.append(geom)
probe = ET.SubElement(world, 'body')
ET.SubElement(probe, 'freejoint')
ET.SubElement(probe, 'geom', type='sphere', size='.1', mass='1', contype='1', conaffinity='2')
model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding='unicode'))
data = mujoco.MjData(model)
for name, position, blocked in [
    ('room opening', (-2.6, -1.2, .25), False),
    ('balcony opening', (0, 3.2, .25), False),
    ('entrance wall opening', (6, -5.6, .25), False),
    ('room wall', (-2.6, -4.5, .25), True),
    ('outer wall', (0, -5.6, .25), True),
    ('balcony parapet', (0, 5.6, .25), True),
]:
    data.qpos[:3] = position
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    assert bool(data.ncon) == blocked, (name, data.ncon)

# Check the complete robot footprint through two furnished-scene corridors.
model = mujoco.MjModel.from_xml_path(str(DEFAULT_OUTPUT))
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'spawn_extended')
for centre, axis in [((-2.6, -1.2), 0), ((-1.5, 3.2), 1)]:
    for offset in np.linspace(-.5, .5, 41):
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.qpos[:2] = centre
        data.qpos[axis] += offset
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            for geom in (contact.geom1, contact.geom2):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ''
                assert not name.startswith('house_'), (centre, offset, name)

# The checked-in scene must match the importer, not just a hand-edited opening.
path = DEFAULT_SOURCE / 'models/aws_robomaker_residential_HouseWallB_01/meshes/aws_HouseWallB_01_collision.DAE'
parts = wall_collision_meshes(path, (1, 1, 1))
assert len(parts) == len(wall_geoms)
for i, part in enumerate(parts):
    mesh = root.find(f"asset/mesh[@name='house_wall_c0_p{i}']")
    np.testing.assert_allclose(np.fromstring(mesh.get('vertex'), sep=' ').reshape(-1, 3),
                               part.vertices, atol=1e-7)
print('PASS: openings, solid walls, robot sweeps, importer/scene agreement')

# Every source instance, including primitives, must have a visible counterpart.
from import_small_house import source_model_paths
import trimesh
_, instances = source_model_paths(DEFAULT_SOURCE, DEFAULT_SOURCE / 'worlds/small_house.world')
visuals = [g for g in root.findall('worldbody/geom') if g.get('name', '').startswith('house_visual_')]
for i, (_, source_name) in enumerate(instances):
    assert any(g.get('name').startswith(f'house_visual_{i}_') for g in visuals), source_name
for geom in visuals:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom.get('name'))
    assert model.geom_rgba[gid, 3] > 0, geom.get('name')
    assert mujoco.MjvOption().geomgroup[model.geom_group[gid]], geom.get('name')
    mid = model.geom_matid[gid]
    assert mid < 0 or model.mat_rgba[mid, 3] > 0, geom.get('name')

# No non-convex furniture may be handed to MuJoCo as one solid convex hull.
collision_meshes = {g.get('mesh') for g in root.findall('worldbody/geom')
                    if g.get('name', '').startswith(('house_collision_', 'house_wall_collision_'))
                    and g.get('type') == 'mesh'}
for mesh in root.findall('asset/mesh'):
    if mesh.get('name') not in collision_meshes:
        continue
    vertices = np.fromstring(mesh.get('vertex'), sep=' ').reshape(-1, 3)
    faces = np.fromstring(mesh.get('face'), sep=' ', dtype=int).reshape(-1, 3)
    part = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    assert part.is_convex, mesh.get('name')

# The two previously invisible balls must now be observable by the ToF ray API.
for i in (2, 3):
    name = f'house_visual_{i}_0_0'
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    collision = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'house_collision_{i}_0')
    np.testing.assert_allclose(model.geom_pos[gid], model.geom_pos[collision])
    np.testing.assert_allclose(model.geom_size[gid], model.geom_size[collision])
    hit = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, data.geom_xpos[gid] + [.4, 0, 0],
                            np.array([-1., 0, 0]), np.ones(6, dtype=np.uint8), 1, -1, hit)
    assert hit[0] == gid and abs(distance - .09) < 1e-6, (name, distance, hit)

for kind in ('kitchentable', 'readingdesk', 'chaira', 'chaird'):
    scene = ET.Element('mujoco')
    asset = ET.SubElement(scene, 'asset')
    world = ET.SubElement(scene, 'worldbody')
    prefix = f'house_aws_robomaker_residential_{kind}_01_c0_'
    meshes = [e for e in root.findall('asset/mesh') if e.get('name', '').startswith(prefix)]
    assert meshes, kind
    for mesh in meshes:
        asset.append(mesh)
        ET.SubElement(world, 'geom', type='mesh', mesh=mesh.get('name'), contype='2', conaffinity='1')
    probe = ET.SubElement(world, 'body')
    ET.SubElement(probe, 'freejoint')
    ET.SubElement(probe, 'geom', type='sphere', size='.04', mass='1', contype='1', conaffinity='2')
    m = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding='unicode'))
    d = mujoco.MjData(m)
    d.qpos[:3] = [0, 0, .12]
    d.qpos[3] = 1
    mujoco.mj_forward(m, d)
    assert d.ncon == 0, (kind, 'gap filled', d.ncon)
    # A probe touching an actual source surface must still collide.
    mesh = meshes[0]
    verts = np.fromstring(mesh.get('vertex'), sep=' ').reshape(-1, 3)
    face = np.fromstring(mesh.get('face'), sep=' ', dtype=int).reshape(-1, 3)[0]
    d.qpos[:3] = verts[face].mean(axis=0)
    mujoco.mj_forward(m, d)
    assert d.ncon > 0, (kind, 'solid surface lost')
print(f'PASS: all {len(instances)} instances / {len(visuals)} visuals, '
      f'{len(collision_meshes)} convex collision assets, visible ToF balls, furniture gaps and surfaces')
