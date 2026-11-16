"""Greater: tune and export a weighted, floor-only surface completion."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_closing, binary_dilation, binary_fill_holes, distance_transform_edt
from scipy.spatial import ConvexHull, Delaunay
import trimesh


DEFAULTS = dict(data_weight=80., smooth_weight=40., flat_weight=15.,
                fill_weight=50., max_fill_gap_m=.25, footprint_radius_m=.70,
                object_preserve=60.)


def fit_floor_plane(points):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('Require at least 100 finite ground points')
    active = np.abs(points[:, 2]-np.median(points[:, 2])) < .04
    for _ in range(8):
        if active.sum() < 100:
            raise ValueError('Too few consistent ground points to fit a floor plane')
        design = np.column_stack((points[active, :2], np.ones(active.sum())))
        coefficients, *_ = np.linalg.lstsq(design, points[active, 2], rcond=None)
        residual = points[:, 2]-(points[:, :2]@coefficients[:2]+coefficients[2])
        center = np.median(residual[active])
        mad = np.median(np.abs(residual[active]-center))
        updated = np.abs(residual-center) <= max(.008, 3*1.4826*mad)
        if np.array_equal(updated, active):
            break
        active = updated
    plane = np.r_[-coefficients[:2], 1., -coefficients[2]]
    plane /= np.linalg.norm(plane[:3])
    if plane[2] < np.cos(np.deg2rad(10)):
        raise ValueError('Ground plane is tilted more than 10 degrees from gravity')
    return plane, active


def _cell_indices(points, origin, resolution, shape):
    indices = np.floor((points[:, :2]-origin)/resolution).astype(int)
    inside = ((indices[:, 0] >= 0) & (indices[:, 0] < shape[1]) &
              (indices[:, 1] >= 0) & (indices[:, 1] < shape[0]))
    return indices, inside


def _face_cells(points, origin, resolution, shape):
    indices, inside = _cell_indices(points.reshape(-1, 3), origin, resolution, shape)
    return indices.reshape(points.shape[0], points.shape[1], 2), inside.reshape(points.shape[0], points.shape[1])


def _grid_mesh(mask, heights, origin, resolution):
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return None
    height_sum = np.zeros((mask.shape[0]+1, mask.shape[1]+1), float)
    count = np.zeros_like(height_sum)
    values = np.where(mask, heights, 0.)
    valid = mask.astype(float)
    for sy, sx in ((slice(None,-1),slice(None,-1)), (slice(None,-1),slice(1,None)),
                   (slice(1,None),slice(1,None)), (slice(1,None),slice(None,-1))):
        height_sum[sy,sx] += values
        count[sy,sx] += valid
    node_z = np.divide(height_sum, count, out=np.zeros_like(height_sum), where=count>0)
    rr, cc = np.mgrid[:mask.shape[0]+1, :mask.shape[1]+1]
    vertices = np.column_stack((origin[0]+cc.ravel()*resolution,
                                origin[1]+rr.ravel()*resolution, node_z.ravel()))
    base = rows*(mask.shape[1]+1)+cols
    faces = np.vstack((np.column_stack((base,base+1,base+mask.shape[1]+2)),
                       np.column_stack((base,base+mask.shape[1]+2,base+mask.shape[1]+1))))
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


class Greater:
    """Prepare a scan once, then tune its 2.5D floor grid without changing obstacles."""
    def __init__(self, mesh_path, layers_path, resolution=.05):
        self.mesh_path, self.layers_path = Path(mesh_path).resolve(), Path(layers_path).resolve()
        if not self.mesh_path.is_file() or not self.layers_path.is_file():
            raise ValueError('Mesh and ground layers must exist')
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError('Resolution must be positive and finite')
        self.resolution = float(resolution)
        with np.load(self.layers_path, allow_pickle=False) as data:
            if str(data['source']) != 'gravity_ransac_labels_v3':
                raise ValueError('Unsupported ground-layer provenance')
            ground, obstacles, uncertain = (np.asarray(data[name], float)
                                             for name in ('ground', 'obstacle', 'uncertain'))
        if any(points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
               for points in (ground, obstacles, uncertain)):
            raise ValueError('Ground layers must be finite Nx3 point arrays')
        self.plane, inliers = fit_floor_plane(ground)
        ground = ground[inliers]
        margin = 1.0
        self.origin = np.floor(ground[:, :2].min(0)/self.resolution)*self.resolution-margin
        size = np.ceil((ground[:, :2].max(0)+margin-self.origin)/self.resolution).astype(int)+1
        if np.any(size <= 0) or np.prod(size) > 2_000_000:
            raise ValueError('Invalid or excessive floor grid bounds')
        self.shape = tuple(size[::-1])
        self.max_fill_gap_m = float(np.hypot(*self.shape)*self.resolution)
        self.source = trimesh.load(self.mesh_path, force='mesh', process=True)
        triangles = self.source.vertices[self.source.faces]
        centers = triangles.mean(axis=1)
        hull = ConvexHull(ground[:, :2])
        violation = centers[:, :2]@hull.equations[:, :2].T+hull.equations[:, 2]
        outside = np.maximum(violation.max(axis=1), 0)
        face_height = centers@self.plane[:3]+self.plane[3]
        self.wall_base = ((np.abs(self.source.face_normals@self.plane[:3]) < .30) &
                          (face_height > -.04) & (face_height < .18) & (outside <= .90))
        support = np.unique(np.round(np.vstack((ground[:, :2], centers[self.wall_base, :2]))/.01)*.01, axis=0)
        self.triangulation = Delaunay(support)
        tri = support[self.triangulation.simplices]
        sides = [np.linalg.norm(tri[:, (i+1)%3]-tri[:, (i+2)%3], axis=1) for i in range(3)]
        area2 = np.abs((tri[:,1,0]-tri[:,0,0])*(tri[:,2,1]-tri[:,0,1])-
                       (tri[:,1,1]-tri[:,0,1])*(tri[:,2,0]-tri[:,0,0]))
        self.circumradius = np.divide(sides[0]*sides[1]*sides[2], 2*area2,
                                      out=np.full_like(area2, np.inf), where=area2>1e-10)
        floor_idx, inside = _cell_indices(ground, self.origin, self.resolution, self.shape)
        flat = floor_idx[inside,1]*self.shape[1]+floor_idx[inside,0]
        self.hits = np.bincount(flat, minlength=np.prod(self.shape)).reshape(self.shape)
        plane_z = -(ground[:,0]*self.plane[0]+ground[:,1]*self.plane[1]+self.plane[3])/self.plane[2]
        residual = ground[:,2]-plane_z
        sums = np.bincount(flat, weights=residual[inside], minlength=np.prod(self.shape))
        self.measured = np.divide(sums, self.hits.ravel(), out=np.zeros_like(sums),
                                  where=self.hits.ravel()>0).reshape(self.shape)
        obstacle_idx, inside = _cell_indices(obstacles, self.origin, self.resolution, self.shape)
        self.obstacles = np.zeros(self.shape, bool)
        self.obstacles[obstacle_idx[inside,1], obstacle_idx[inside,0]] = True
        uncertain_height = uncertain@self.plane[:3]+self.plane[3]
        below = uncertain[uncertain_height < -.06]
        drop_idx, inside = _cell_indices(below, self.origin, self.resolution, self.shape)
        drop_flat = drop_idx[inside,1]*self.shape[1]+drop_idx[inside,0]
        drop_counts = np.bincount(drop_flat, minlength=np.prod(self.shape)).reshape(self.shape)
        self.drops = binary_dilation(drop_counts>=3, iterations=1)
        rr, cc = np.mgrid[:self.shape[0], :self.shape[1]]
        self.centers = self.origin+np.stack((cc+.5,rr+.5),axis=-1)*self.resolution
        self.simplex = self.triangulation.find_simplex(self.centers.reshape(-1,2)).reshape(self.shape)
        self.base_height = -(self.centers[...,0]*self.plane[0]+self.centers[...,1]*self.plane[1]+self.plane[3])/self.plane[2]
        self.triangles = triangles
        self.face_centers = centers
        self.face_cells, self.face_inside = _cell_indices(centers, self.origin, self.resolution, self.shape)
        self.vertex_cells, self.vertex_inside = _face_cells(triangles, self.origin, self.resolution, self.shape)
        self.mesh_sha256 = hashlib.sha256(self.mesh_path.read_bytes()).hexdigest()
        self.layers_sha256 = hashlib.sha256(self.layers_path.read_bytes()).hexdigest()

    def solve(self, params):
        p = {**DEFAULTS, **params}
        if any(not np.isfinite(v) for v in p.values()):
            raise ValueError('Greater weights must be finite')
        if any(not 0 <= p[key] <= 100 for key in ('data_weight','smooth_weight','flat_weight','fill_weight','object_preserve')):
            raise ValueError('Weights must be between 0 and 100')
        if not 0 <= p['max_fill_gap_m'] <= self.max_fill_gap_m or not .15 <= p['footprint_radius_m'] <= 1.2:
            raise ValueError(f'Fill gap must be 0–{self.max_fill_gap_m:.2f} m for this scan; footprint radius must be 0.15–1.2 m')
        valid = self.simplex >= 0
        footprint = np.zeros(self.shape, bool)
        footprint[valid] = self.circumradius[self.simplex[valid]] <= p['footprint_radius_m']
        close_cells = max(1, round(.10/self.resolution))
        yy, xx = np.mgrid[-close_cells:close_cells+1,-close_cells:close_cells+1]
        footprint = binary_fill_holes(binary_closing(footprint, structure=xx*xx+yy*yy<=close_cells**2))
        observed = (self.hits>0)&footprint&~self.drops
        distance = distance_transform_edt(~observed)*self.resolution
        effective_fill_gap=p['max_fill_gap_m']*p['fill_weight']/100
        inferred = footprint&~observed&~self.drops&(distance<=effective_fill_gap)&(p['fill_weight']>0)
        processed = observed|inferred
        if not observed.any():
            raise ValueError('No measured ground cells inside the selected footprint')
        weights = {key: p[key]/10. for key in ('data_weight','smooth_weight','flat_weight','fill_weight')}
        if not any(weights.values()):
            raise ValueError('Set at least one Greater weight above zero')
        confidence = np.clip(self.hits/(self.hits+2.),0,1)
        residual = self.measured.copy()
        height = residual.copy()
        confidence_obs = confidence*observed
        smooth = np.where(observed,weights['smooth_weight'],weights['fill_weight'])
        # Edge-aware Jacobi updates minimize data, neighbor-smoothness, plane and fill penalties.
        for _ in range(32):
            neighbor_sum=np.zeros(self.shape); neighbor_weights=np.zeros(self.shape)
            for dy,dx in ((-1,0),(1,0),(0,-1),(0,1)):
                cy=slice(max(0,dy),self.shape[0]+min(0,dy)); cx=slice(max(0,dx),self.shape[1]+min(0,dx))
                ny=slice(max(0,-dy),self.shape[0]-max(0,dy)); nx=slice(max(0,-dx),self.shape[1]-max(0,dx))
                valid=processed[cy,cx]&processed[ny,nx]
                edge=np.ones(valid.shape)
                both=observed[cy,cx]&observed[ny,nx]
                delta=np.abs(self.measured[cy,cx]-self.measured[ny,nx])
                edge[both]=1/(1+(delta[both]/.04)**2)
                guarded=self.obstacles[cy,cx]|self.obstacles[ny,nx]
                edge[guarded]*=.1
                edge*=valid
                neighbor_sum[cy,cx]+=edge*height[ny,nx]
                neighbor_weights[cy,cx]+=edge
            data=weights['data_weight']*confidence_obs
            neighbor=smooth*neighbor_weights
            denominator=data+neighbor+weights['flat_weight']
            updated=np.divide(data*residual+smooth*neighbor_sum,denominator,
                              out=np.zeros_like(height),where=denominator>0)
            height[processed]=updated[processed]
        height[~processed]=np.nan
        return dict(params=p,footprint=footprint,observed=observed,inferred=inferred,
                    effective_fill_gap_m=effective_fill_gap,
                    processed=processed,distance=distance,height=self.base_height+height/self.plane[2])

    def export(self, output_path, result):
        output_path=Path(output_path).resolve()
        if output_path.exists() or self.mesh_path.parent in output_path.parents:
            raise ValueError('Output must be new and outside the source mesh directory')
        p=result['params']; domain=result['footprint']; processed=result['processed']
        face_domain=np.zeros(len(self.source.faces),bool)
        valid=self.face_inside
        idx=self.face_cells[valid]
        face_domain[valid]=domain[idx[:,1],idx[:,0]]
        for vertex in range(3):
            valid=self.vertex_inside[:,vertex]; idx=self.vertex_cells[valid,vertex]
            face_domain[valid]|=domain[idx[:,1],idx[:,0]]
        signed=self.triangles@self.plane[:3]+self.plane[3]
        band=.10-p['object_preserve']/100*.07
        near=np.max(np.abs(signed),axis=1)<band
        aligned=np.abs(self.source.face_normals@self.plane[:3])>.7
        floor_faces=face_domain&near&(aligned|(np.max(np.abs(signed),axis=1)<.03))
        objects=self.source.copy(); objects.update_faces(~floor_faces); objects.remove_unreferenced_vertices()
        floor=_grid_mesh(processed,result['height'],self.origin,self.resolution)
        observed=_grid_mesh(result['observed'],result['height'],self.origin,self.resolution)
        inferred=_grid_mesh(result['inferred'],result['height'],self.origin,self.resolution)
        if floor is None:
            raise ValueError('No floor surface to export')
        output_path.mkdir(parents=True,exist_ok=False); target=output_path/'floor_completion'; target.mkdir()
        layers={'objects':objects,'floor':floor}
        if observed is not None: layers['observed']=observed
        if inferred is not None: layers['inferred']=inferred
        layer_paths={}
        for name,mesh in layers.items():
            path=target/f'{name}.stl'; mesh.export(path); layer_paths[name]=str(path)
        floor.export(target/'floor_surface.stl')
        completed=trimesh.util.concatenate([objects,floor])
        completed.export(output_path/'final_mesh.stl'); completed.export(output_path/'final_mesh.ply')
        measured_error=np.abs(result['height'][result['observed']]-self.base_height[result['observed']]-self.measured[result['observed']])
        profile=dict(algorithm='Greater',version=1,weights={k:p[k] for k in ('data_weight','smooth_weight','flat_weight','fill_weight')},
                     max_fill_gap_m=p['max_fill_gap_m'],footprint_radius_m=p['footprint_radius_m'],
                     effective_fill_gap_m=result['effective_fill_gap_m'],
                     object_preserve=p['object_preserve'],resolution_m=self.resolution,
                     source_mesh_sha256=self.mesh_sha256,ground_layers_sha256=self.layers_sha256)
        (target/'greater_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
        manifest=dict(**profile,source_mesh=str(self.mesh_path),ground_layers=str(self.layers_path),
                      plane=self.plane.tolist(),observed_cells=int(result['observed'].sum()),
                      inferred_cells=int(result['inferred'].sum()),excluded_drop_cells=int(self.drops.sum()),
                      obstacle_cells=int(self.obstacles.sum()),removed_near_plane_faces=int(floor_faces.sum()),
                      preserved_object_faces=int(len(objects.faces)),p95_observed_height_change_m=float(np.percentile(measured_error,95)),
                      layers=layer_paths,fill_semantics='Inferred cells are not sensor observations or certified free space.')
        (target/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        metadata=self.mesh_path.with_suffix('.json')
        pose_source=json.loads(metadata.read_text()).get('pose_source','estimated scan poses') if metadata.is_file() else 'estimated scan poses'
        (output_path/'final_mesh.json').write_text(json.dumps(dict(algorithm='Greater',pose_source=pose_source+' + Greater floor',
                                                                  bounds_m=completed.bounds.tolist(),source_mesh=str(self.mesh_path)))+'\n')
        return manifest


def tune(scan, output_path):
    import tkinter as tk
    from tkinter import messagebox
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.colors import ListedColormap
    from matplotlib.figure import Figure

    root=tk.Tk(); root.title('Greater · Floor Tuner'); root.geometry('1240x820')
    figure=Figure(figsize=(10,6),dpi=90); ax_height,ax_labels=figure.subplots(1,2)
    canvas=FigureCanvasTkAgg(figure,master=root); canvas.get_tk_widget().pack(fill='both',expand=True)
    controls=tk.Frame(root); controls.pack(fill='x')
    status=tk.StringVar(value='Preparing preview…')
    scales={}
    max_gap_cm=int(np.ceil(scan.max_fill_gap_m*20)*5)
    definitions=(('data_weight','Giữ số đo',0,100,80,1),('smooth_weight','Làm mịn',0,100,40,1),
                 ('flat_weight','Ép phẳng',0,100,15,1),('fill_weight','Nội suy lỗ (%)',0,100,50,1),
                 ('max_fill_gap_m',f'Bán kính fill (cm, max {max_gap_cm})',0,max_gap_cm,min(25,max_gap_cm),5),
                 ('footprint_radius_m','Bán kính biên (cm)',15,120,70,5),
                 ('object_preserve','Giữ vật thấp',0,100,60,1))
    for column,(key,label,lo,hi,default,resolution) in enumerate(definitions):
        frame=tk.Frame(controls); frame.grid(row=0,column=column,sticky='ew',padx=3)
        tk.Label(frame,text=label).pack()
        scale=tk.Scale(frame,from_=lo,to=hi,resolution=resolution,orient='horizontal',length=155,
                       showvalue=True,command=lambda _v: schedule())
        scale.set(default); scale.pack(); scales[key]=scale
        controls.grid_columnconfigure(column,weight=1)
    tk.Label(root,textvariable=status,anchor='w').pack(fill='x',padx=8)
    export_button=tk.Button(root,text='Xuất bản đồ Greater',command=lambda: export(),state='disabled')
    export_button.pack(pady=6)
    pending=[None]
    labels_cmap=ListedColormap(['#343b45','#48a68a','#e6b64c','#d94d5c'])
    height_image=ax_height.imshow(np.zeros(scan.shape),origin='lower',cmap='terrain',vmin=-8,vmax=8)
    labels_image=ax_labels.imshow(np.zeros(scan.shape),origin='lower',cmap=labels_cmap,vmin=-.5,vmax=3.5)
    ax_height.set_title('Sàn ước lượng, cao độ lệch mặt phẳng (cm)')
    ax_labels.set_title('Nguồn ô: unknown / measured / inferred / obstacle')
    figure.colorbar(height_image,ax=ax_height,label='cm so với mặt phẳng')

    def parameters():
        return dict(data_weight=float(scales['data_weight'].get()),smooth_weight=float(scales['smooth_weight'].get()),
                    flat_weight=float(scales['flat_weight'].get()),fill_weight=float(scales['fill_weight'].get()),
                    max_fill_gap_m=float(scales['max_fill_gap_m'].get())/100,
                    footprint_radius_m=float(scales['footprint_radius_m'].get())/100,
                    object_preserve=float(scales['object_preserve'].get()))

    def redraw():
        pending[0]=None
        result=scan.solve(parameters()); masked=np.ma.masked_where(~result['processed'],
                                       (result['height']-scan.base_height)*100)
        height_image.set_data(masked)
        classes=np.zeros(scan.shape,float); classes[result['observed']]=1; classes[result['inferred']]=2
        classes[scan.obstacles]=3; labels_image.set_data(classes)
        changed=np.abs(result['height'][result['observed']]-scan.base_height[result['observed']]-scan.measured[result['observed']])
        status.set(f"Observed {result['observed'].sum():,} | inferred {result['inferred'].sum():,} | "
                   f"fill radius {result['effective_fill_gap_m']:.2f}/{result['params']['max_fill_gap_m']:.2f} m | "
                   f"excluded drops {scan.drops.sum():,} | P95 observed change {np.percentile(changed,95)*1000:.1f} mm")
        export_button.config(state='normal')
        canvas.draw_idle(); root.current_result=result

    def schedule():
        if pending[0] is not None: root.after_cancel(pending[0])
        pending[0]=root.after(120,redraw)

    def export():
        try:
            manifest=scan.export(output_path,root.current_result)
        except (OSError,ValueError) as error:
            messagebox.showerror('Greater export failed',str(error),parent=root); return
        status.set(f"Đã xuất {output_path} | observed {manifest['observed_cells']:,} | inferred {manifest['inferred_cells']:,}")
        export_button.config(state='disabled')

    root.after(100,redraw); root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh',type=Path); parser.add_argument('ground_layers',type=Path); parser.add_argument('output',type=Path)
    parser.add_argument('--interactive',action='store_true',help='Open the Greater slider tuner before export')
    parser.add_argument('--resolution',type=float,default=.05)
    for key,default in DEFAULTS.items():
        flag=key.replace('_','-')
        parser.add_argument('--'+flag,type=float,default=default)
    args=parser.parse_args()
    scan=Greater(args.mesh,args.ground_layers,args.resolution)
    params={key:getattr(args,key) for key in DEFAULTS}
    if args.interactive:
        tune(scan,args.output)
    else:
        print(json.dumps(scan.export(args.output,scan.solve(params)),indent=2))


if __name__=='__main__':
    main()
