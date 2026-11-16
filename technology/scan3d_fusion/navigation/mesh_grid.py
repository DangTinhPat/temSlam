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
