"""Compare fusion maps with a held-out simulator oracle; never modifies either map."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh


def evaluate(session, oracle):
    sequence=json.loads((session/'live_sequence.json').read_text())
    fx,fy,cx,cy=oracle['K'];y,x=np.indices(oracle['depth'].shape[1:])
    rays=np.stack(((x-cx)/fx,(y-cy)/fy,np.ones_like(x)),-1)
    total=kept=wall_kept=behind=0;absolute=square=0.
    for j,i in enumerate(oracle['indices']):
        with np.load(session/sequence['frames'][i]['file'],allow_pickle=False) as data:
            depth=data['depth'].astype(float)/1000
        pose=oracle['camera_to_world'][j];reference=oracle['depth'][j]
        reference_world=(rays*reference[...,None])@pose[:3,:3].T+pose[:3,3]
        height=((rays*depth[...,None])@pose[:3,:3].T+pose[:3,3])[...,2]
        floor=oracle['floor'][j];observed=floor&(depth>0)
        wall=(reference>.02)&(reference<=4)&(reference_world[...,2]>.15)&(depth>0)
        total+=int(floor.sum());kept+=int(observed.sum());wall_kept+=int(wall.sum())
        absolute+=float(abs(height[observed]).sum());square+=float((height[observed]**2).sum())
        behind+=int((wall&(depth-reference>.1)).sum())
    mesh=trimesh.load(session/'final_mesh.stl',force='mesh',process=True)
    centers=mesh.triangles_center;up=abs(mesh.face_normals[:,2])
    selected=(abs(centers[:,2])<.08)&(up>.7)
    areas=mesh.area_faces[selected]*up[selected];z=centers[selected,2]
    return dict(session=str(session),evaluated_frames=len(oracle['indices']),floor_pixels=total,
        floor_retained_pixels=kept,floor_retained_fraction=kept/total,
        depth_floor_height_mae_m=absolute/kept,depth_floor_height_rmse_m=(square/kept)**.5,
        wall_retained_pixels=wall_kept,wall_behind_10cm_pixels=behind,wall_behind_10cm_fraction=behind/wall_kept,
        mesh_near_floor_projected_triangle_area_m2=float(areas.sum()),
        mesh_near_floor_height_mae_m=float(np.average(abs(z),weights=areas)),
        mesh_near_floor_height_rmse_m=float(np.sqrt(np.average(z*z,weights=areas))),mesh_faces=len(mesh.faces))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path);parser.add_argument('candidate',type=Path)
    parser.add_argument('--oracle',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    with np.load(args.oracle,allow_pickle=False) as oracle:
        report=dict(baseline=evaluate(args.baseline,oracle),candidate=evaluate(args.candidate,oracle),
            limitation='Oracle is evaluation only. Pixel errors use true poses; mesh heights use estimated map coordinates. Near-floor triangle area can overlap and is not room coverage.')
    a=json.loads((args.baseline/'live_sequence.json').read_text())
    b=json.loads((args.candidate/'live_sequence.json').read_text())
    assert a['frames']==b['frames'],'Comparison must keep estimated poses unchanged'
    assert report['candidate']['floor_retained_pixels']>=report['baseline']['floor_retained_pixels']
    assert report['candidate']['wall_behind_10cm_fraction']<=report['baseline']['wall_behind_10cm_fraction']+.001
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
