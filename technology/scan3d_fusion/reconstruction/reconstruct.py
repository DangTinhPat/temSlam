#!/usr/bin/env python3
"""Offline DELTAR reconstruction with recorded or estimated poses; no ROS runtime."""
import argparse
import json
import time
from pathlib import Path

import h5py
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, binary_dilation
from skimage.measure import marching_cubes
import torch
import trimesh
from depth_support import tof_support


class TSDF:
    def __init__(self, bounds, voxel=.01, truncation=.06, device="cuda"):
        self.bounds = np.asarray(bounds, dtype=np.float32)
        if (self.bounds.shape != (2, 3) or not np.isfinite(self.bounds).all() or
                np.any(self.bounds[1] <= self.bounds[0]) or not np.isfinite([voxel, truncation]).all()
                or voxel <= 0 or truncation < voxel):
            raise ValueError("invalid volume bounds, voxel size or truncation")
        self.shape = tuple(np.ceil((self.bounds[1]-self.bounds[0])/voxel).astype(int)+1)
        if np.prod(self.shape) > 20_000_000:
            raise ValueError("volume exceeds 20 million voxels; use a smaller ROI or larger voxels")
        self.voxel, self.truncation = voxel, truncation
        axes = [torch.arange(n, device=device)*voxel+float(lo)
                for n, lo in zip(self.shape, self.bounds[0])]
        self.points = torch.stack(torch.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        self.sums = torch.zeros(len(self.points), device=device)
        self.weights = torch.zeros_like(self.sums)

    @torch.no_grad()
    def integrate(self, depth, camera_to_world, intrinsics):
        depth = np.asarray(depth, dtype=np.float32)
        pose = np.asarray(camera_to_world, dtype=np.float32)
        if (depth.ndim != 2 or pose.shape != (4, 4) or not np.isfinite(pose).all() or
                not np.allclose(pose[3], [0, 0, 0, 1]) or
                not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3), atol=1e-4) or
                np.linalg.det(pose[:3, :3]) < .99):
            raise ValueError("invalid depth image or rigid camera pose")
        fx, fy, cx, cy = intrinsics
        if not np.isfinite(intrinsics).all() or min(fx, fy) <= 0:
            raise ValueError("invalid camera intrinsics")
        dev = self.points.device
        T = torch.as_tensor(pose, device=dev)
        camera = (self.points-T[:3, 3])@T[:3, :3]
        z = camera[:, 2]
        u = torch.round(camera[:, 0]/z.clamp_min(.001)*fx+cx).long()
        v = torch.round(camera[:, 1]/z.clamp_min(.001)*fy+cy).long()
        h, w = depth.shape
        visible = (z>.02) & (u>=0) & (u<w) & (v>=0) & (v<h)
        ids = torch.nonzero(visible).flatten()
        measured = torch.as_tensor(depth, device=dev)[v[ids], u[ids]]
        sdf = measured-z[ids]
        valid = torch.isfinite(measured) & (measured>.02) & (sdf>=-self.truncation)
        ids, sdf = ids[valid], sdf[valid]
        self.sums[ids] += (sdf/self.truncation).clamp(-1, 1)
        self.weights[ids] += 1

    def mesh(self, min_views=6):
        weight = self.weights.cpu().numpy().reshape(self.shape)
        sums = self.sums.cpu().numpy().reshape(self.shape)
        raw = sums/np.maximum(weight, 1)
        field = gaussian_filter(sums, 1)/np.maximum(gaussian_filter(weight, 1), 1e-6)
        # Lewiner checks mask at the high corner: every corner of that cube must have real observations.
        supported = weight >= min_views
        cubes = np.ones(tuple(n-1 for n in self.shape),bool)
        recovery_support=np.ones_like(cubes)
        raw_negative=np.zeros_like(cubes);raw_positive=np.zeros_like(cubes)
        smooth_negative=np.zeros_like(cubes);smooth_positive=np.zeros_like(cubes)
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    corner=(slice(x,x+cubes.shape[0]),slice(y,y+cubes.shape[1]),slice(z,z+cubes.shape[2]))
                    cubes &= supported[corner]
                    recovery_support &= weight[corner]>=2*min_views
                    raw_negative |= raw[corner]<0;raw_positive |= raw[corner]>=0
                    smooth_negative |= field[corner]<0;smooth_positive |= field[corner]>=0
        gradient=np.gradient(raw)
        norm_squared=sum(g*g for g in gradient)
        horizontal=(norm_squared>1e-10) & (gradient[2]**2>=.81*norm_squared)
        # Preserve well-observed near-horizontal surfaces in the Z-up map; leave steep wall speckle smoothed.
        lost=(cubes & recovery_support & raw_negative & raw_positive & horizontal[1:,1:,1:]
              & ~binary_dilation(cubes & smooth_negative & smooth_positive))
        restore=np.zeros(self.shape,bool)
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    restore[x:x+cubes.shape[0],y:y+cubes.shape[1],z:z+cubes.shape[2]] |= lost
        field[restore]=raw[restore]
        observed=np.zeros(self.shape,bool);observed[1:,1:,1:]=cubes
        if not (lost.any() or np.any(cubes & smooth_negative & smooth_positive)):
            raise RuntimeError("insufficient observed zero crossings for a surface")
        vertices, faces, _, _ = marching_cubes(field, 0, spacing=(self.voxel,)*3,
                                              mask=observed, allow_degenerate=False)
        mesh = trimesh.Trimesh(vertices+self.bounds[0], faces, process=True)
        parts = [p for p in mesh.split(only_watertight=False) if p.area >= .01]
        if not parts:
            raise RuntimeError("no supported surface components")
        mesh = trimesh.util.concatenate(parts)
        trimesh.smoothing.filter_taubin(mesh, iterations=6)
        mesh.fix_normals()
        return mesh

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("prediction", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel", type=float, default=.01)
    parser.add_argument("--trajectory", type=Path, help="cuVSLAM sequence with final estimated poses and bounds")
    parser.add_argument("--reference", action="store_true", help="explicit GT-depth diagnostic; not DELTAR")
    cli = parser.parse_args()
    sequence = json.loads((cli.capture/"sequence.json").read_text())
    if cli.trajectory:
        if cli.reference:
            parser.error("--reference cannot be combined with an estimated trajectory")
        estimated = json.loads(cli.trajectory.read_text())
        frames = {f['file']: f for f in sequence['frames']}
        if (not estimated['frames'] or len({f['file'] for f in estimated['frames']}) != len(estimated['frames'])
                or estimated['image_size'] != sequence['image_size']
                or not np.allclose(estimated['intrinsics'], sequence['intrinsics'])
                or any(f['file'] not in frames or f['time_s'] != frames[f['file']]['time_s']
                       for f in estimated['frames'])):
            raise ValueError("Estimated trajectory does not match this capture")
        sequence = estimated
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA required for this GPU reconstruction example")
    volume = TSDF(sequence["bounds_m"], voxel=cli.voxel, truncation=6*cli.voxel)
    calibration = json.loads((Path(__file__).resolve().parents[3]/
                             "technology/scan3d_fusion/config/sim_calibration.json").read_text())["camera_to_tof"]
    start = time.monotonic()
    accepted, total = 0, 0
    for i, frame in enumerate(sequence["frames"]):
        with h5py.File(cli.capture/frame["file"]) as h5:
            if cli.reference:
                depth = h5["depth"][:]
            else:
                filename = Path(frame["file"]).stem+"_depth.png"
                depth = np.asarray(Image.open(cli.prediction/filename), np.float32)/1000
                depth = tof_support(depth, h5["hist_data"][:], h5["fr"][:], h5["mask"][:],
                                    sequence["intrinsics"], calibration)
        accepted += int((depth>0).sum())
        total += depth.size
        volume.integrate(depth, frame["camera_to_world"], sequence["intrinsics"])
        if i % 12 == 0:
            print(f"integrate {i+1}/{len(sequence['frames'])}", flush=True)
    mesh = volume.mesh()
    cli.output.mkdir(parents=True, exist_ok=True)
    stem = "reference" if cli.reference else "clay"
    for suffix in ("ply", "stl", "obj"):
        mesh.export(cli.output/f"{stem}.{suffix}")
    info = {"source": "MuJoCo ground-truth depth diagnostic" if cli.reference else "DELTAR + ToF bias alignment and consistency mask",
            "trajectory_file": str(cli.trajectory.resolve()) if cli.trajectory else None,
            "pose_source": sequence["pose_source"], "method": "projective TSDF; not neural ToF-SLAM",
            "frames": len(sequence["frames"]), "vertices": len(mesh.vertices), "faces": len(mesh.faces),
            "bounds_m": mesh.bounds.tolist(),
            "min_views": 6, "sdf_smoothing_sigma_voxels": 1, "recover_horizontal_crossings": True,
            "min_component_area_m2": .01,
            "voxel_m": cli.voxel, "accepted_pixel_fraction": accepted/total,
            "fusion_mesh_seconds": time.monotonic()-start,
            "gpu": torch.cuda.get_device_name(), "peak_gpu_mib": torch.cuda.max_memory_allocated()/2**20}
    (cli.output/f"{stem}.json").write_text(json.dumps(info, indent=2)+"\n")
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
