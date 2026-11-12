"""Recheck/repair the fixed scan route against the current MuJoCo collision scene.

Run with a MuJoCo Python environment from the repository root. No online planning.
"""
import argparse
import heapq
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[3]
ROUTE = Path(__file__).with_name('native') / 'fixed_scan_route.json'


def collision_checker(scene, radius=.40):
    tree = ET.parse(scene)
    root = tree.getroot()
    for keyframe in root.findall('keyframe'):
        root.remove(keyframe)
    # MuJoCo 3.2 does not know the newer mesh inertia attribute; collision hulls are unchanged.
    for mesh in root.findall('asset/mesh'):
        mesh.attrib.pop('inertia', None)
    body = ET.SubElement(root.find('worldbody'), 'body', name='route_clearance_probe')
    ET.SubElement(body, 'freejoint', name='route_clearance_joint')
    ET.SubElement(body, 'geom', name='route_clearance_geom', type='cylinder',
                  size=f'{radius} .22', pos='0 0 0', mass='1', contype='1', conaffinity='2')
    with tempfile.NamedTemporaryFile(suffix='.xml', dir=scene.parent, delete=False) as f:
        path = Path(f.name)
    try:
        tree.write(path)
        model = mujoco.MjModel.from_xml_path(str(path))
    finally:
        path.unlink()
    probe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'route_clearance_geom')
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'body_link')
    if robot < 0:
        raise ValueError('Robot body_link is missing')
    for i in range(model.ngeom):
        parent = model.geom_bodyid[i]
        while parent > 0 and parent != robot:
            parent = model.body_parentid[parent]
        if parent == robot:
            model.geom_contype[i] = model.geom_conaffinity[i] = 0
    data = mujoco.MjData(model)
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'route_clearance_joint')
    adr = model.jnt_qposadr[joint]

    def clear(x, y):
        data.qpos[adr:adr+7] = [x, y, .23, 1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        return not any(probe in (c.geom1, c.geom2) for c in data.contact)
    return clear


def segment_clear(a, b, clear):
    n = max(1, math.ceil(math.dist(a, b)/.05))
    return all(clear(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n+1))


def repair(route, clear):
    step = route['grid_step_m']
    original = route['waypoints']
    # Keep the original coverage bounds and scan order; only detour around changed furniture.
    bounds = [min(p[k] for p in original) for k in ('x', 'y')], [max(p[k] for p in original) for k in ('x', 'y')]
    free = {(i, j) for i in range(round(bounds[0][0]/step), round(bounds[1][0]/step)+1)
            for j in range(round(bounds[0][1]/step), round(bounds[1][1]/step)+1) if clear(i*step, j*step)}
    def xy(cell): return (cell[0]*step, cell[1]*step)
    start = tuple(round(v/step) for v in route['start_xy_m'])
    if start not in free:
        raise ValueError('Route start is obstructed')
    connected = {start}; pending = [start]
    while pending:
        a = pending.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            b = (a[0]+dx, a[1]+dy)
            if b in free and b not in connected and segment_clear(xy(a), xy(b), clear):
                connected.add(b); pending.append(b)

    def connect(a, b):
        queue = [(0, a)]; costs = {a: 0}; previous = {}
        while queue:
            _, node = heapq.heappop(queue)
            if node == b:
                path = [b]
                while path[-1] != a: path.append(previous[path[-1]])
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (node[0]+dx, node[1]+dy); cost = costs[node]+1
                if nxt in connected and cost < costs.get(nxt, math.inf) and segment_clear(xy(node), xy(nxt), clear):
                    previous[nxt] = node; costs[nxt] = cost
                    heapq.heappush(queue, (cost+abs(nxt[0]-b[0])+abs(nxt[1]-b[1]), nxt))
        raise ValueError('Disconnected scan target')

    cells = [start]; moved = 0
    for point in original[1:]:
        target = (round(point['x']/step), round(point['y']/step))
        if target not in connected:
            replacement = min(connected, key=lambda c: (math.dist(c, target), c))
            if math.dist(xy(replacement), xy(target)) > 1.5:
                raise ValueError(f'Scan target moved more than 1.5 m: {point}')
            target = replacement; moved += 1
        cells.extend(connect(cells[-1], target)[1:])
    # Keep every turn; redistribute scan stops every metre after the detours.
    output = []; since_scan = 0.0
    for i, cell in enumerate(cells):
        if i: since_scan += math.dist(xy(cells[i-1]), xy(cell))
        scan = i == 0 or i == len(cells)-1 or since_scan >= route['scan_spacing_m']-1e-6
        turn = 0 < i < len(cells)-1 and (cell[0]-cells[i-1][0], cell[1]-cells[i-1][1]) != (cells[i+1][0]-cell[0], cells[i+1][1]-cell[1])
        if scan or turn or i == len(cells)-1:
            output.append(dict(x=xy(cell)[0], y=xy(cell)[1], **({'scan': True} if scan else {})))
        if scan: since_scan = 0
    route['waypoints'] = output
    route['route_length_m'] = sum(math.dist(xy(a), xy(b)) for a, b in zip(cells, cells[1:]))
    route.setdefault('clearance_radius_m', .40)
    return moved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='Repair and replace the route; default only checks')
    args = parser.parse_args()
    route = json.loads(ROUTE.read_text())
    clear = collision_checker(ROOT/route['scene'], route.get('clearance_radius_m', .40))
    if args.write:
        moved = repair(route, clear)
        print(f'Relocated {moved} obstructed targets')
    points = [(p['x'], p['y']) for p in route['waypoints']]
    blocked = [i+1 for i, (a, b) in enumerate(zip(points, points[1:])) if not segment_clear(a, b, clear)]
    if blocked: raise ValueError(f'Obstructed route segments: {blocked}')
    if args.write: ROUTE.write_text(json.dumps(route, indent=2)+'\n')
    print(f'PASS: {len(points)} waypoints, {route["route_length_m"]:.2f} m, {route.get("clearance_radius_m", .40):.2f} m clearance radius')

if __name__ == '__main__': main()
