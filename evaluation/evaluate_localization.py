"""Score localization against isolated simulator truth using a fixed map alignment.

The alignment must be supplied independently (e.g. from the mapping session's
first frame). This evaluator never fits an alignment to the query trajectory.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'technology/scan3d_fusion/mapping/odometry'),
                str(ROOT/'technology/scan3d_fusion/reconstruction')]
from localization import odom_body, rigid
from surface_matching import planar_change


def pose_errors(estimate, simulator_pose, map_from_simulator):
    estimate, truth = rigid(estimate), rigid(map_from_simulator) @ rigid(simulator_pose)
    delta = np.arctan2(estimate[1,0],estimate[0,0])-np.arctan2(truth[1,0],truth[0,0])
    return dict(xy_m=float(np.linalg.norm(estimate[:2,3]-truth[:2,3])),
                yaw_deg=float(np.rad2deg(np.arctan2(np.sin(delta),np.cos(delta)))))


def aggregate_errors(rows):
    def stats(values):
        xy=np.asarray([r['xy_m'] for r in values]);yaw=np.asarray([r['yaw_deg'] for r in values])
        return dict(samples=len(values), xy_rmse_m=float(np.sqrt(np.mean(xy**2))) if len(xy) else None,
                    xy_p95_m=float(np.quantile(xy,.95)) if len(xy) else None,
                    yaw_rmse_deg=float(np.sqrt(np.mean(yaw**2))) if len(yaw) else None)
    return dict(all=stats(rows), usable=stats([r for r in rows if r['usable']]))


def evaluate(measurements, results, alignment, output):
    measurements, results, alignment, output = map(Path,(measurements,results,alignment,output))
    alignment_data=json.loads(alignment.read_text())
    mapped=rigid(alignment_data['map_from_simulator'])
    manifest=json.loads((measurements/'localization_input.json').read_text())
    truth=json.loads((measurements/'evaluator_truth.json').read_text())
    run=json.loads((results/'run.json').read_text())
    records=[json.loads(line) for line in (results/'poses.jsonl').read_text().splitlines()]
    key=lambda t:round(t*1e9)
    actual={key(f['time_s']):f['simulator_from_body'] for f in truth['frames']}
    packets={key(f['time_s']):f['odometry'] for f in manifest['frames']}
    first=next(odom_body(f['odometry']) for f in manifest['frames'] if f['odometry']['valid'])
    x,y,yaw=run['initial_body_pose_xyyaw_rad']
    origin=planar_change([0,0,yaw-np.arctan2(first[1,0],first[0,0])],np.zeros(3))
    origin[:2,3]=[x,y]-(origin@first)[:2,3]
    errors=[];baseline=[];positions=[]
    for record in records:
        t=record['timestamp_ns']
        if t not in actual or t not in packets:
            raise ValueError('Results require exact corresponding truth and odometry timestamps')
        if record['world_from_body'] is None:
            continue
        pose=rigid(record['world_from_body']);odom=origin@odom_body(packets[t]);true_pose=mapped@rigid(actual[t])
        errors.append(dict(time_s=record['time_s'],usable=record['usable'],**pose_errors(pose,actual[t],mapped)))
        baseline.append(dict(time_s=record['time_s'],usable=record['usable'],**pose_errors(odom,actual[t],mapped)))
        positions.append(dict(time_s=record['time_s'],truth_xy=true_pose[:2,3].tolist(),
                              localized_xy=pose[:2,3].tolist(),odometry_xy=odom[:2,3].tolist()))
    report=dict(measurements=str(measurements.resolve()),results=str(results.resolve()),
                alignment=alignment_data,alignment_fitted_to_query=False,frames=len(records),
                usable_frames=sum(r['usable'] for r in records),localization=aggregate_errors(errors),
                odometry_same_initial_pose=aggregate_errors(baseline),errors=errors,positions=positions,
                limitation='Simulator-only, fixed gauge from mapping session. Map deformation contributes to error; no hardware claim.')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements',type=Path);parser.add_argument('results',type=Path)
    parser.add_argument('--alignment',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=evaluate(args.measurements,args.results,args.alignment,args.output)
    print(json.dumps({k:report[k] for k in ('frames','usable_frames','localization','odometry_same_initial_pose')},indent=2))


if __name__=='__main__':main()
