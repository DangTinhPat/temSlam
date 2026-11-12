"""Legacy inferred-floor experiment; not a fusion-only map and not used by live/finalize."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_closing, binary_dilation, binary_fill_holes, label
import trimesh


def fill_supported_gaps(observed, blocked, resolution=.05):
    if observed.shape != blocked.shape or observed.ndim != 2 or resolution <= 0:
        raise ValueError('Expected matching floor grids and positive resolution')
    observed = np.asarray(observed, bool) & ~blocked
    radius = max(1, round(.25/resolution))
    y,x = np.mgrid[-radius:radius+1, -radius:radius+1]
    disk = x*x+y*y <= radius*radius
    # Close sampling cracks in the measured contour, without a convex hull across concave room boundaries.
    closed = binary_closing(observed, structure=disk) | observed
    components, _ = label(closed)
    areas = np.bincount(components.ravel())*resolution**2
    keep = areas >= .04;keep[0] = False
    # shortcut: one continuous floor per enclosed contour; use multiple plane patches for stairs/multiple levels.
    domain = binary_fill_holes(keep[components]) & ~blocked
    observed &= domain
    inferred = domain & ~observed
    return observed, inferred


def grid_mesh(mask, origin, resolution, plane):
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return None
    cells = np.stack((cols, rows), -1)
    xy = (cells[:,None,:]+np.array([[0,0],[1,0],[1,1],[0,1]])[None,:,:])*resolution+origin
    z = -(xy @ plane[:2]+plane[3])/plane[2]
    vertices = np.concatenate((xy,z[...,None]),axis=2).reshape(-1,3)
    base = np.arange(len(cells))*4
    faces = np.concatenate((base[:,None]+[0,1,2],base[:,None]+[0,2,3]))
    return trimesh.Trimesh(vertices,faces,process=True)


def blocked_cells(hits, obstacles, drops):
    # Clear occasional false obstacles only with repeated, dominant floor evidence; never clear measured drops.
    floor_dominates = (hits>=5) & (hits.astype(np.uint32)>=3*obstacles.astype(np.uint32)) & (drops<2)
    return binary_dilation(((obstacles>=2)&~floor_dominates)|(drops>=2),iterations=1)


def confirmed_drops(hits, drops):
    # A negative-height outlier must recur and dominate floor returns before cutting a hole in the plane.
    return binary_dilation((drops>=3)&(drops>hits),iterations=1)


def complete(session, sequence=None):
    session = Path(session).resolve()
    if sequence is None:
        path = session/'surface_sequence.json'
        sequence = json.loads((path if path.exists() else session/'live_sequence.json').read_text())
    resolution = .05
    bounds = np.array(sequence['bounds_m'])
    origin = np.floor(bounds[0,:2]/resolution)*resolution-.25
    size = np.ceil((bounds[1,:2]+.25-origin)/resolution).astype(int)
    if np.any(size <= 0) or np.prod(size) > 2_000_000:
        raise ValueError('Invalid or excessive floor bounds')
    shape = tuple(size[::-1])
    hits = np.zeros(shape,np.uint16); obstacles = np.zeros_like(hits); drops = np.zeros_like(hits)
    planes = []
    records = []
    for frame in sequence['frames']:
        with np.load(session/frame['file'],allow_pickle=False) as data:
            if 'floor_plane' not in data:
                continue
            plane = data['floor_plane']
        if plane.shape != (4,) or not np.isfinite(plane).all():
            continue
        pose = np.asarray(frame['camera_to_world'],float)
        normal = pose[:3,:3]@plane[:3]
        if normal[2] < np.cos(np.deg2rad(10)):
            continue
        planes.append(np.r_[normal,plane[3]-normal@pose[:3,3]])
        records.append(frame)
    if len(records) < 3:
        raise ValueError('At least three frames with observed floor are required')
    plane = np.median(planes,axis=0);plane /= np.linalg.norm(plane[:3])
    fx,fy,cx,cy = sequence['intrinsics']
    def add(grid, points):
        index = np.floor((points[:,:2]-origin)/resolution).astype(int)
        valid = (index >= 0).all(1) & (index < size).all(1)
        index = np.unique(index[valid],axis=0)
        grid[index[:,1],index[:,0]] += 1
    for frame in sequence['frames']:
        with np.load(session/frame['file'],allow_pickle=False) as data:
            depth = data['depth'][::4,::4].astype(float)/1000
            supported = 'floor_plane' in data and np.isfinite(data['floor_plane']).all()
        y,x = np.mgrid[:depth.shape[0],:depth.shape[1]]*4
        points = np.stack(((x-cx)*depth/fx,(y-cy)*depth/fy,depth),-1)
        pose = np.asarray(frame['camera_to_world'])
        world = points@pose[:3,:3].T+pose[:3,3]
        dy,dx = np.gradient(world,axis=(0,1));normal = np.cross(dx,dy)
        alignment = np.abs(normal@plane[:3])/np.maximum(np.linalg.norm(normal,axis=2),1e-9)
        valid = np.isfinite(depth)&(depth>.02)&(depth<=4)
        height = world@plane[:3]+plane[3]
        if supported:
            add(hits,world[valid & (abs(height)<.02) & (alignment>np.cos(np.deg2rad(25)))])
        add(obstacles,world[valid & ((height>.12)|((height>.04)&(alignment<.5)))])
        add(drops,world[valid & (height<-.06)])
    conflicted = blocked_cells(hits,obstacles,drops)
    blocked = confirmed_drops(hits,drops)
    observed,inferred = fill_supported_gaps(hits>=2,blocked,resolution)
    # Furniture affects confidence/clearance, but its XY footprint is not a missing floor.
    inferred |= observed & conflicted
    observed &= ~conflicted
    target = session/'floor_completion';target.mkdir(exist_ok=True)
    np.savez_compressed(target/'evidence.npz',observed=observed,inferred=inferred,blocked=blocked,conflicted=conflicted,
                        floor_hits=hits,obstacle_hits=obstacles,drop_hits=drops,
                        origin=origin,resolution=resolution,plane=plane)
    meshes = {}
    for name,mask in [('observed',observed),('inferred',inferred)]:
        mesh = grid_mesh(mask,origin,resolution,plane)
        if mesh is not None:
            mesh.export(target/(name+'.stl'));meshes[name]=mesh
    source = trimesh.load(session/'final_mesh.stl',force='mesh',process=True)
    centers=source.triangles_center
    index=np.floor((centers[:,:2]-origin)/resolution).astype(int)
    inside=(index>=0).all(1)&(index<size).all(1)
    replace=np.zeros(len(centers),bool)
    replace[inside]=(observed|inferred)[index[inside,1],index[inside,0]]
    vertex_index=np.floor((source.vertices[:,:2]-origin)/resolution).astype(int)
    vertex_inside=(vertex_index>=0).all(1)&(vertex_index<size).all(1)
    vertex_floor=np.zeros(len(source.vertices),bool)
    vertex_floor[vertex_inside]=(observed|inferred)[vertex_index[vertex_inside,1],vertex_index[vertex_inside,0]]
    replace |= vertex_floor[source.faces].any(1)
    center_blocked=np.zeros(len(centers),bool)
    center_blocked[inside]=blocked[index[inside,1],index[inside,0]]
    replace &= ~center_blocked
    distances=source.vertices[source.faces]@plane[:3]+plane[3]
    floor_faces=replace & (np.abs(distances).max(1)<.08) & (
        (np.abs(source.face_normals@plane[:3])>.7)|(np.abs(distances).max(1)<.03))
    objects=source.copy();objects.update_faces(~floor_faces);objects.remove_unreferenced_vertices()
    objects.export(target/'objects.stl');meshes['objects']=objects
    floor = grid_mesh(observed|inferred,origin,resolution,plane)
    if floor is not None:
        floor.export(target/'floor.stl')
    combined=trimesh.util.concatenate(list(meshes.values()))
    combined.export(target/'completed.stl');combined.export(target/'completed.ply')
    report=dict(source_session=str(session),source='recorded depth and estimated poses; no scene/CAD geometry',
                floor_plane=plane.tolist(),confirmed_frames=len(records),resolution_m=resolution,
                observed_area_m2=float(observed.sum()*resolution**2),inferred_area_m2=float(inferred.sum()*resolution**2),
                boundary_closing_radius_m=.25,removed_floor_faces=int(floor_faces.sum()),
                floor_components=int(label(observed|inferred)[1]),
                unfilled_interior_cells=int((binary_fill_holes(observed|inferred)&~(observed|inferred)&~blocked).sum()),
                protected_drop_cells=int(blocked.sum()),
                conflict_policy='Furniture footprints remain inferred floor, not free space; drops need >=3 frames and more returns than floor',
                layers={k:str(target/(k+'.stl')) for k in meshes},
                limitation='Assumes a continuous single-level plane inside measured contours, including under furniture. Exterior and confirmed drops stay open. Inferred floor is not a clearance guarantee.')
    if floor is not None:
        report['layers']['floor']=str(target/'floor.stl')
    (target/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    (target/'completed.json').write_text(json.dumps(dict(pose_source=sequence.get('pose_source','cuVSLAM')+' + observed/inferred floor',bounds_m=combined.bounds.tolist()))+'\n')
    print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    complete(parser.parse_args().session)
