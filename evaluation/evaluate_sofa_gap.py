"""Compare observed mesh support in the user-identified sofa/table gap; no reconstruction input."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import trimesh

BOUNDS=np.array([[-3.,-3.3],[-1.3,-1.3]])
ROOM_BOUNDS=np.array([[-5.55,-4.8],[1.4,1.35]])
RESOLUTION=.02


def evaluate(session,bounds=BOUNDS):
    mesh=trimesh.load(session/'final_mesh.stl',force='mesh',process=True)
    centers=mesh.triangles_center;normal=abs(mesh.face_normals[:,2])
    mask=(abs(centers[:,2])<.08)&(normal>.7)
    size=np.ceil((bounds[1]-bounds[0])/RESOLUTION).astype(int)
    image=Image.new('1',tuple(size),0);draw=ImageDraw.Draw(image)
    for triangle in mesh.triangles[mask]:
        xy=(triangle[:,:2]-bounds[0])/RESOLUTION
        draw.polygon([tuple(p) for p in xy],fill=1)
    supported=np.array(image,bool)
    region=mask&(centers[:,0]>=bounds[0,0])&(centers[:,0]<=bounds[1,0])&(centers[:,1]>=bounds[0,1])&(centers[:,1]<=bounds[1,1])
    areas=mesh.area_faces[region]*normal[region];z=centers[region,2]
    return dict(supported_xy_area_m2=float(supported.sum()*RESOLUTION**2),
        support_fraction=float(supported.mean()),near_floor_height_rmse_m=float(np.sqrt(np.average(z*z,weights=areas)))),supported


def surrounding_regions(before,after,bounds):
    y,x=np.indices(before.shape)
    x=bounds[0,0]+(x+.5)*RESOLUTION;y=bounds[0,1]+(y+.5)*RESOLUTION
    gap=(x>=BOUNDS[0,0])&(x<=BOUNDS[1,0])&(y>=BOUNDS[0,1])&(y<=BOUNDS[1,1])
    output={}
    for name,mask in [('tây-nam',(x<-2.15)&(y<-2.3)),('đông-nam',(x>=-2.15)&(y<-2.3)),
                      ('tây-bắc',(x<-2.15)&(y>=-2.3)),('đông-bắc',(x>=-2.15)&(y>=-2.3))]:
        mask&=~gap
        output[name]=dict(baseline_fraction=float(before[mask].mean()),candidate_fraction=float(after[mask].mean()),
                         added_supported_area_m2=float((after[mask].sum()-before[mask].sum())*RESOLUTION**2))
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path);parser.add_argument('candidate',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--room',action='store_true',help='Measure the room and four quadrants outside the sofa gap')
    args=parser.parse_args();bounds=ROOM_BOUNDS if args.room else BOUNDS
    before,bmask=evaluate(args.baseline,bounds);after,amask=evaluate(args.candidate,bounds)
    a=json.loads((args.baseline/'live_sequence.json').read_text());b=json.loads((args.candidate/'live_sequence.json').read_text())
    base={f['file']:f for f in a['frames']};candidate={f['file']:f for f in b['frames']}
    assert all(candidate[k]==v for k,v in base.items()),'Existing estimated poses must stay unchanged'
    report=dict(bounds_xy_m=bounds.tolist(),resolution_m=RESOLUTION,baseline=before,candidate=after,
        base_frames=len(base),candidate_frames=len(candidate),existing_poses_unchanged=True,
        limitation='Rasterized XY support of near-floor measured mesh, not CAD coverage or a proof of true flatness. Extra views use propagated estimated poses.')
    if args.room:report['surrounding_quadrants_excluding_gap']=surrounding_regions(bmask,amask,bounds)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    canvas=Image.new('RGB',(1000,650),'#171c23');draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    for i,(name,mask,stats) in enumerate([('Trước',bmask,before),('Thêm góc fusion',amask,after)]):
        rgb=np.where(mask[::-1,:,None],np.array([195,186,164]),np.array([35,42,52])).astype(np.uint8)
        size=(460,407) if args.room else (425,500)
        canvas.paste(Image.fromarray(rgb).resize(size,Image.Resampling.NEAREST),(30+500*i,80))
        draw.text((30+500*i,20),f"{name}: {stats['support_fraction']:.1%} vùng có mesh",font=font,fill='white')
    label='Toàn phòng' if args.room else 'Chỉ vùng khe sofa–bàn'
    draw.text((30,605),label+' • Phần tối: chưa có mesh đủ bằng chứng',font=font,fill='white')
    canvas.save(args.output.with_suffix('.png'))


if __name__=='__main__':main()
