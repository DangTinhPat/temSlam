#!/usr/bin/env python3
"""Generate four diagnostic rows for30 native context frames; no classifier/SDK."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import h5py
import numpy as np
from PIL import Image
from diagnose_return_consistency import ROOT, diagnose_frame, tof_support, diagnostic_consistency_mask

TARGETS=(7,22,37,52,67,82,97,112,127,142)
CONTEXT=tuple(sorted({i for target in TARGETS for i in (target-2,target-1,target)}))
VARIANTS=('baseline','selected_bias_legacy_slope','no_bias_selected_slope','selected_bias_selected_slope')


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(raw,hist,rect,valid,K,cal,variant):
    if variant=='baseline':
        return dict(variant=variant,selected_bias_enabled=False,selected_slope_enabled=False),raw.copy(),tof_support(raw,hist,rect,valid,K,cal)
    if variant not in VARIANTS:raise ValueError('Unknown frozen variant')
    bias=variant!='no_bias_selected_slope';slope=variant!='selected_bias_legacy_slope'
    report,candidate=diagnose_frame(raw,hist,rect,valid,K,cal,bias_enabled=bias,selected_slope=slope,owner_policy='strict')
    mask=diagnostic_consistency_mask(candidate,hist,valid,K,cal,np.where(valid[:,None],rect,0),selected_slope=slope)
    supported=np.where(mask&np.isfinite(raw)&(raw>.02),candidate,0).astype(np.float32)
    report['variant']=variant
    return report,candidate,supported


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path,help='Full150-frame calibrated baseline, with immutable supported keyframes')
    parser.add_argument('predictions',type=Path,help='Same calibrated raw prediction directory')
    parser.add_argument('output',type=Path)
    cli=parser.parse_args();source=cli.prepared.resolve();predictions=cli.predictions.resolve();out=cli.output.resolve()
    if out.exists() or source==out or source in out.parents or predictions==out or predictions in out.parents:
        parser.error('Output must be new and separate from immutable inputs')
    manifest=source/'localization_input.json';sequence=json.loads(manifest.read_text())
    if len(sequence['frames'])!=150:parser.error('Frozen diagnostic requires the full150-frame native cohort')
    source_record=json.loads((source/'source.json').read_text())
    if source_record.get('ground_truth_depth_used') or source_record.get('ground_truth_pose_used_by_tracker'):
        parser.error('Oracle estimator input forbidden')
    spatial_path=predictions/'spatial_diagnostic.json';spatial=json.loads(spatial_path.read_text())
    if spatial['mode']!='calibrated':parser.error('Frozen four rows require calibrated spatial predictions')
    checkpoint=ROOT/'technology/scan3d_fusion/model/deltar/weights/nyu.pt'
    if spatial['checkpoint_sha256']!=digest(checkpoint):parser.error('Checkpoint does not match prediction provenance')
    calpath=ROOT/'technology/scan3d_fusion/config/sim_calibration.json'
    cal=json.loads(calpath.read_text())['camera_to_tof'];K=sequence['intrinsics']
    tracked=[manifest,source/'source.json',spatial_path,checkpoint,calpath,Path(__file__),
             Path(__file__).with_name('diagnose_return_consistency.py'),ROOT/'technology/scan3d_fusion/capture/capture_mujoco.py',
             ROOT/'technology/scan3d_fusion/reconstruction/depth_support.py']
    for i in CONTEXT:
        tracked.extend((source/'inputs'/f'{i:05d}.h5',predictions/f'{i:05d}_depth.png',source/sequence['frames'][i]['file']))
    hashes={str(p):digest(p) for p in tracked}
    report=dict(diagnostic_only=True,ground_truth_used=False,ground_truth_depth_used=False,ground_truth_pose_used_by_tracker=False,
                map_accepted=False,not_for_mapping=True,complete=False,checkpoint_sha256=digest(checkpoint),
                source=str(source),predictions=str(predictions),targets=list(TARGETS),generated_source_indices=list(CONTEXT),
                full_source_frame_count=150,intrinsics=K,calibration=cal,source_sha256=hashes,
                classification_required='Merge these30 context depths into copies of full150 baseline frames; rerun unchanged classifier chronologically. No sparse classification.',
                owner_policy='strict',surface_owner_exception_approved=False,correction_scope='Independent per-zone in-sample fit; no held-out-zone validation claim',rows={})
    out.mkdir(parents=True);reportpath=out/'quality.json'
    def persist():reportpath.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    persist();started=time.monotonic()
    try:
        for variant in VARIANTS:
            folder=out/variant;folder.mkdir();(folder/'keyframes').mkdir()
            row=dict(complete=False,frames=[],fit_proposals=0,accepted_zones=0,rejection_counts={},checkpoint_sha256=report['checkpoint_sha256'],
                     ground_truth_used=False,map_accepted=False,source_indices=list(CONTEXT),baseline_depth_bitwise_equal=True if variant=='baseline' else None)
            report['rows'][variant]=row
            for count,index in enumerate(CONTEXT,1):
                frame=sequence['frames'][index]
                with h5py.File(str(source/'inputs'/f'{index:05d}.h5'),'r') as h5:
                    hist=h5['hist_data'][:];rect=h5['fr'][:];valid=h5['mask'][:]
                # Exact production float32 input and millimeter rounding.
                raw=np.array(Image.open(predictions/f'{index:05d}_depth.png'),dtype=np.float32)/1000
                detail,candidate,supported=generate(raw,hist,rect,valid,K,cal,variant)
                depth=np.rint(supported*1000).astype(np.uint16)
                if variant=='baseline':
                    with np.load(source/frame['file']) as saved:
                        if not np.array_equal(depth,saved['depth']):raise ValueError(f'Baseline depth mismatch frame{index}')
                np.savez_compressed(folder/'keyframes'/f'{index:05d}.npz',depth=depth,
                    raw_candidate_depth_m=candidate.astype(np.float32),original_valid_mask=np.isfinite(raw)&(raw>.02),
                    tof_hist=hist,tof_rect=rect,tof_valid=valid,source_index=index,diagnostic_only=True,not_for_mapping=True)
                for zone in detail.get('zones',[]):
                    row['fit_proposals']+=int('proposal' in zone);row['accepted_zones']+=int(zone['candidate_accepted'])
                    for reason in zone['rejection_reasons']:row['rejection_counts'][reason]=row['rejection_counts'].get(reason,0)+1
                detail.update(index=index,time_s=frame['time_s'],valid_supported_pixels=int(np.count_nonzero(depth)))
                row['frames'].append(detail)
                print(f'{variant} {count}/{len(CONTEXT)} source={index} accepted_zones={row["accepted_zones"]}',flush=True)
            row['complete']=True;persist()
        if any(digest(Path(path))!=value for path,value in hashes.items()):raise RuntimeError('Immutable source/code/checkpoint changed')
        report['source_sha256_verified_after']=True;report['complete']=True;report['elapsed_seconds']=time.monotonic()-started;persist()
    except Exception as exc:
        report['failure']=str(exc);persist();raise
    return 0

if __name__=='__main__':raise SystemExit(main())
