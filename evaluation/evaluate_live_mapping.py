"""Frozen live-mapping regression evaluator; simulator poses are evaluation-only.

Usage: python evaluation/evaluate_live_mapping.py SESSION OUTPUT_DIRECTORY
"""
import json,re,sys,tempfile
from pathlib import Path
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'technology/scan3d_fusion/mapping/cuvslam'))
from evaluate import metrics
session=Path(sys.argv[1]).resolve();output=Path(sys.argv[2]).resolve()
assert output!=session and session not in output.parents, 'Keep evaluator artifacts outside raw measurements'
output.mkdir(exist_ok=True,parents=True)
meta=json.loads((session/'metadata.json').read_text());cmd=meta['runner_command']
scene=Path(cmd[cmd.index('--model')+1]);scene=scene if scene.is_absolute() else root/scene
with tempfile.NamedTemporaryFile('w',suffix='.xml',dir=scene.parent,delete=False) as stream:
 stream.write(re.sub(r'\s+inertia="[^"]+"','',scene.read_text()));xml=Path(stream.name)
try:model=mujoco.MjModel.from_xml_path(str(xml))
finally:xml.unlink()
data=mujoco.MjData(model)
cam=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'head_camera')
assert cam>=0, 'Missing evaluator camera'
joints={k:model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)] for k,n in [('lh','left_hip'),('lk','left_knee'),('lw','left_wheel'),('rh','right_hip'),('rk','right_knee'),('rw','right_wheel'),('hj','head_yaw')]}
captures=[json.loads(x) for x in (session/'live_capture.jsonl').read_text().splitlines()]
times=np.array([float(x['t']) for x in captures]);report={'session':str(session),'scope':'Evaluator only; simulator poses never enter mapping; rigid SE3, no scale fitting','trajectories':{}}
for name in ['live_sequence','surface_sequence']:
 f=session/(name+'.json')
 if not f.exists():continue
 seq=json.loads(f.read_text());est=[];truth=[];offsets=[]
 for frame in seq['frames']:
  i=int(np.argmin(abs(times-frame['time_s'])));v=captures[i];offsets.append(abs(times[i]-frame['time_s']))
  assert offsets[-1]<=.02,'No synchronized evaluator truth'
  data.qpos[:7]=[float(v[k]) for k in ['x','y','z','qw','qx','qy','qz']]
  for k,adr in joints.items():data.qpos[adr]=float(v[k])
  mujoco.mj_forward(model,data)
  t=np.eye(4);t[:3,:3]=data.cam_xmat[cam].reshape(3,3)@np.diag([1,-1,-1]);t[:3,3]=data.cam_xpos[cam]
  est.append(frame['camera_to_world']);truth.append(t)
 assert len(est)>2
 assert np.isfinite([est,truth]).all(), 'Nonfinite evaluator poses'
 est,truth=np.asarray(est),np.asarray(truth)
 result,aligned=metrics(est,truth)
 first=truth[0]@np.linalg.inv(est[0])@est
 errors=np.linalg.inv(truth)@first
 angles=np.rad2deg(Rotation.from_matrix(errors[:,:3,:3]).magnitude())
 result.update(endpoint_translation_error_m=float(np.linalg.norm(errors[-1,:3,3])),
               endpoint_rotation_error_deg=float(angles[-1]),
               first_pose_rotation_p95_deg=float(np.quantile(angles,.95)),
               first_pose_translation_max_m=float(np.linalg.norm(errors[:,:3,3],axis=1).max()),
               max_timestamp_offset_s=max(offsets),pose_source=seq.get('pose_source'))
 report['trajectories'][name]=result
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 truth=np.asarray(truth);fig,ax=plt.subplots(figsize=(7,7))
 ax.plot(truth[:,0,3],truth[:,1,3],label='Simulator reference (evaluation only)',lw=1.5)
 ax.plot(aligned[:,0,3],aligned[:,1,3],label='Estimated camera (SE3 aligned)',lw=1,alpha=.8)
 ax.set(xlabel='X (m)',ylabel='Y (m)',title=f'{name}: ATE {result["ate_se3_rmse_m"]:.3f} m',aspect='equal');ax.legend();ax.grid(alpha=.3)
 fig.savefig(output/(name+'.png'),dpi=130,bbox_inches='tight');plt.close(fig)
(output/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
