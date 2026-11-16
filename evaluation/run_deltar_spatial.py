#!/usr/bin/env python3
"""Isolated, seeded legacy/calibrated A/B over immutable DELTAR H5 inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'technology/scan3d_fusion/model/deltar'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['legacy','calibrated'],required=True)
    parser.add_argument('--data-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-frames',type=int,default=0)
    cli=parser.parse_args()
    if cli.output.exists():parser.error('output must be a new directory')
    # Same initialization/RNG policy and checkpoint for BOTH branches. Histogram
    # quantiles are deterministic; no change to their measurement semantics.
    sys.path.insert(0,str(MODEL))
    sys.argv=['evaluate.py',str(ROOT/'technology/scan3d_fusion/config/deltar_eval.txt')]
    import numpy as np
    import torch
    np.random.seed(2026);torch.manual_seed(2026)
    if torch.cuda.is_available():torch.cuda.manual_seed_all(2026)
    import src.models.decoder as decoder
    if cli.mode=='calibrated':
        from deltar_spatial import CalibratedTransformerFusion
        decoder.TransformerFusion=CalibratedTransformerFusion
    # Reuse the existing exact loader, model construction, strict state loading,
    # interpolation and PNG units. Substitution is local to this process only.
    import importlib.util
    spec=importlib.util.spec_from_file_location('spatial_run_demo',ROOT/'technology/scan3d_fusion/inference/run_demo.py')
    demo=importlib.util.module_from_spec(spec);spec.loader.exec_module(demo)
    import h5py
    associations={}
    for path in sorted(cli.data_root.rglob('*.h5')):
        with h5py.File(str(path),'r') as h5:
            rect=np.asarray(h5['fr']);valid=np.asarray(h5['mask']).astype(bool)
            height,width=np.asarray(h5['rgb']).shape[:2]
        per_scale={}
        for stride in (4,8,16):
            ys=(np.arange(height//stride)+.5)*stride;xs=(np.arange(width//stride)+.5)*stride
            counts=[]
            for box,keep in zip(rect,valid):
                if not keep:counts.append(0);continue
                y0,x0,y1,x1=box
                counts.append(int(np.count_nonzero((ys>=y0)&(ys<y1))*np.count_nonzero((xs>=x0)&(xs<x1))))
            per_scale[str(stride)]=dict(feature_cells_per_zone=counts,
                valid_zones_without_feature_cells=np.flatnonzero(valid&(np.asarray(counts)==0)).tolist())
        associations[str(path.relative_to(cli.data_root))]=per_scale
    inputs=sorted(cli.data_root.rglob('*.h5'))
    hashes={str(p.relative_to(cli.data_root)):digest(p) for p in inputs}
    checkpoint=MODEL/'weights/nyu.pt';checkpoint_hash=digest(checkpoint)
    sys.argv=['run_demo.py','--data-root',str(cli.data_root),'--output',str(cli.output),'--max-frames',str(cli.max_frames)]
    index_hash=digest(cli.data_root/'data.json')
    adapter_path=Path(__file__).with_name('deltar_spatial.py')
    adapter_hash=digest(adapter_path)
    started=time.monotonic();result=demo.main();elapsed=time.monotonic()-started
    if digest(cli.data_root/'data.json')!=index_hash or (cli.mode=='calibrated' and digest(adapter_path)!=adapter_hash):
        raise RuntimeError('input index/loaded adapter changed during inference')
    if hashes!={str(p.relative_to(cli.data_root)):digest(p) for p in inputs} or digest(checkpoint)!=checkpoint_hash:
        raise RuntimeError('immutable input/checkpoint changed during inference')
    report=dict(mode=cli.mode,seed=2026,checkpoint_sha256=checkpoint_hash,input_h5_sha256=hashes,
                input_index_sha256=index_hash,elapsed_seconds=elapsed,
                output_frames=len(list(cli.output.glob('*_depth.png'))),diagnostic_only=True,
                production_forward_changed=False,feature_center_convention='(index+0.5)*stride in half-open calibrated rectangle',
                zone_association=associations,adapter_sha256=adapter_hash)
    (cli.output/'spatial_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n')
    return result

if __name__=='__main__':raise SystemExit(main())
