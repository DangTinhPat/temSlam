"""Project the existing reconstructed mesh for a kinematic 3-D demonstration.

This is surface support, not observed volumetric clearance. Never authorize
native motion from this grid. The original mesh and strict depth grid stay intact.
"""
import numpy as np
from grid import NavigationGrid, FREE, OCCUPIED, UNKNOWN


def build_mesh_grid(triangles):
    triangles=np.asarray(triangles,float)
    if (triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not len(triangles)
            or not np.isfinite(triangles).all()):
        raise ValueError('Require finite mesh triangles')
    resolution=.05;subdivisions=10
    lower=np.floor(triangles[:,:,:2].min((0,1))/resolution).astype(int)-1
    upper=np.ceil(triangles[:,:,:2].max((0,1))/resolution).astype(int)+1
    shape=(upper-lower)[::-1]
    if np.prod(shape)>250_000:raise ValueError('Mesh bounds too large for demo projection')
    origin=lower*resolution
    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    norm=np.linalg.norm(normal,axis=1)
    floor=(np.abs(triangles[:,:,2]).max(1)<=.025)&(np.abs(normal[:,2])>=np.cos(np.deg2rad(15))*norm)&(norm>1e-12)
    if not floor.any():raise ValueError('No observed near-horizontal floor within 2.5 cm of Z=0')
    support=np.zeros(shape*subdivisions,bool)
    fine=resolution/subdivisions
    for triangle in triangles[floor,:,:2]:
        lo=np.maximum(0,np.floor((triangle.min(0)-origin)/fine).astype(int))
        hi=np.minimum(shape[::-1]*subdivisions,np.ceil((triangle.max(0)-origin)/fine).astype(int)+1)
        row,col=np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
        points=np.stack((col+.5,row+.5),-1)*fine+origin
        edges=np.roll(triangle,-1,axis=0)-triangle
        relative=points[...,None,:]-triangle
        cross=edges[:,0]*relative[...,1]-edges[:,1]*relative[...,0]
        inside=np.all(cross>=-1e-12,axis=-1)|np.all(cross<=1e-12,axis=-1)
        support[lo[1]:hi[1],lo[0]:hi[0]]|=inside
    # shortcut: 5 mm support sampling can miss smaller holes; this grid is demo-only.
    observed=support.reshape(shape[0],subdivisions,shape[1],subdivisions).all((1,3))
    cells=np.full(shape,UNKNOWN,np.int8);cells[observed]=FREE
    obstacles=(triangles[:,:,2].max(1)>.025)&(triangles[:,:,2].min(1)<=.4)
    for triangle in triangles[obstacles,:,:2]:
        lo=np.maximum(0,np.floor((triangle.min(0)-origin)/resolution).astype(int)-1)
        hi=np.minimum(shape[::-1],np.floor((triangle.max(0)-origin)/resolution).astype(int)+1)
        row,col=np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
        centers=np.stack((col+.5,row+.5),-1)*resolution+origin
        edges=np.roll(triangle,-1,axis=0)-triangle
        axes=np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
        tri_projection=triangle@axes.T
        box_projection=centers@axes.T
        radius=np.abs(axes).sum(1)*resolution/2
        intersects=np.all((box_projection+radius>=tri_projection.min(0)-1e-12)&
                          (box_projection-radius<=tri_projection.max(0)+1e-12),axis=-1)
        cells[lo[1]:hi[1],lo[0]:hi[0]][intersects]=OCCUPIED
    return NavigationGrid(cells,origin,resolution,.25,demo_only=True)


def build_height_slice_grid(triangles, support_grid, height_m=.14305):
    """Project mesh intersections at one robot-body height into the observed floor grid."""
    triangles=np.asarray(triangles,float)
    if (triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not len(triangles)
            or not np.isfinite(triangles).all() or not np.isfinite(height_m)
            or not isinstance(support_grid,NavigationGrid)):
        raise ValueError('Require finite mesh triangles, a support grid and finite slice height')
    cells=support_grid.cells.copy()
    cells[cells!=UNKNOWN]=FREE
    origin=support_grid.origin
    resolution=support_grid.resolution
    rows,cols=cells.shape
    for triangle in triangles:
        z=triangle[:,2]-height_m
        if z.min()>1e-6 or z.max() < -1e-6:
            continue
        if np.all(np.abs(z)<=1e-6):
            points=triangle[:,:2]
            lo=np.maximum(0,np.floor((points.min(0)-origin)/resolution).astype(int))
            hi=np.minimum(cells.shape[::-1],np.floor((points.max(0)-origin)/resolution).astype(int)+1)
            if np.any(hi<=lo):
                continue
            row,col=np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
            centers=np.stack((col+.5,row+.5),-1)*resolution+origin
            edges=np.roll(points,-1,axis=0)-points
            axes=np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
            projected=points@axes.T;box=centers@axes.T
            radius=np.abs(axes).sum(1)*resolution/2
            hit=np.all((box+radius>=projected.min(0)-1e-12)&
                       (box-radius<=projected.max(0)+1e-12),axis=-1)
            cells[lo[1]:hi[1],lo[0]:hi[0]][hit]=OCCUPIED
            continue
        intersections=[]
        for i,j in ((0,1),(1,2),(2,0)):
            if abs(z[i])<=1e-6:
                intersections.append(triangle[i,:2])
            if z[i]*z[j]<-1e-12:
                t=-z[i]/(z[j]-z[i])
                intersections.append(triangle[i,:2]+t*(triangle[j,:2]-triangle[i,:2]))
        if not intersections:
            continue
        points=np.unique(np.round(intersections,9),axis=0)
        if len(points)>1:
            pair=np.unravel_index(np.argmax(np.sum((points[:,None]-points[None,:])**2,axis=2)),
                                  (len(points),len(points)))
            start,end=points[pair[0]],points[pair[1]]
            count=max(1,int(np.ceil(np.linalg.norm(end-start)/(resolution*.5))))
            points=start+(end-start)*np.linspace(0,1,count+1)[:,None]
        xy=np.floor((points-origin)/resolution).astype(int)
        inside=(xy[:,0]>=0)&(xy[:,0]<cols)&(xy[:,1]>=0)&(xy[:,1]<rows)
        cells[xy[inside,1],xy[inside,0]]=OCCUPIED
    return NavigationGrid(cells,origin,resolution,support_grid.clearance_m,demo_only=True)
