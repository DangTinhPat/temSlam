"""Build a fixed dense sweep route from scene collision geometry, not online navigation."""
import json
import math
from pathlib import Path

from repair_scan_route import ROOT, collision_checker, repair, segment_clear


def main():
    target = Path(__file__).with_name('native')/'living_room_scan_route.json'
    old = json.loads(target.read_text())
    backup = target.with_name('living_room_perimeter_route.json')
    if old.get('route_style') == 'perimeter loop' and not backup.exists():
        old['clearance_radius_m'] = .25
        backup.write_text(json.dumps(old,indent=2)+'\n')
    dense_backup = target.with_name('living_room_interior_route.json')
    if old.get('route_style') == 'interior serpentine coverage' and not dense_backup.exists():
        dense_backup.write_text(json.dumps(old,indent=2)+'\n')
    route = dict(scene=old['scene'], start_xy_m=[0.,0.], grid_step_m=.15,
                 scan_spacing_m=.75, head_range_deg=[-180,180], clearance_radius_m=.25,
                 route_style='perimeter then interior coverage', room='living_room_sofa',
                 lane_spacing_m=.75, waypoints=[dict(x=0.,y=0.,scan=True)])
    clear = collision_checker(ROOT/route['scene'], route['clearance_radius_m'])
    # The west passage behind the sofa lies between the old 25 cm grid columns.
    perimeter = [(1.05,0), (1.05,-4.65), (-5.4,-4.65), (-5.4,1.05), (1.05,1.05), (1.05,0)]
    for x,y in perimeter:
        if not clear(x,y):
            raise ValueError(f'Perimeter target is obstructed: {(x,y)}')
        route['waypoints'].append(dict(x=x,y=y,scan=True))
    # Visit only traversable targets: projecting blocked grid targets caused repeated stair detours.
    for row in range(8):
        y = -4.5 + .75*row
        columns = [.75 - .75*i for i in range(9)]
        if row % 2:
            columns.reverse()
        for x in columns:
            if clear(x,y):
                route['waypoints'].append(dict(x=x,y=y,scan=True))
    # The narrow strip north of the stairs needs its own pass.
    for x in [.75-.75*i for i in range(9)]:
        if clear(x,1.05):
            route['waypoints'].append(dict(x=x,y=1.05,scan=True))
    route['waypoints'].append(dict(x=0.,y=0.,scan=True))
    relocated = repair(route,clear)
    scans=[]
    for point in route['waypoints']:
        if point.get('scan'):
            xy=(point['x'],point['y'])
            if any(math.dist(xy,previous)<.45 for previous in scans):
                point.pop('scan')
            else:
                scans.append(xy)
    points=[(p['x'],p['y']) for p in route['waypoints']]
    blocked=[i+1 for i,(a,b) in enumerate(zip(points,points[1:])) if not segment_clear(a,b,clear)]
    if relocated or blocked:
        raise RuntimeError(f'Unexpected moved targets={relocated}, blocked segments={blocked}')
    west_scans=[p for p in route['waypoints'] if p.get('scan') and abs(p['x']+5.4)<1e-6]
    if not any(-3.1<p['y']<-1.9 for p in west_scans):
        raise RuntimeError('No scan stop behind the sofa')
    # Slow down in the narrow wall passages; the interior retains its existing speed.
    for p in route['waypoints']:
        if p['x'] < -5.2 or p['x'] > .9 or p['y'] < -4.5 or p['y'] > .9:
            p['speed_m_s']=.12
    target.write_text(json.dumps(route,indent=2)+'\n')
    print(json.dumps(dict(length_m=route['route_length_m'],waypoints=len(points),scan_stops=len(scans),
                         lane_spacing_m=.75,clearance_radius_m=.25,blocked_segments=blocked),indent=2))


if __name__=='__main__':main()
