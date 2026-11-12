#!/usr/bin/env python3
"""Evaluate estimated poses against simulator truth after tracking has finished."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def align_se3(estimated, truth):
    """Rigid alignment only: never fit scale to conceal depth-scale errors."""
    x, y = estimated[:, :3, 3], truth[:, :3, 3]
    u, _, vt = np.linalg.svd((x-x.mean(0)).T @ (y-y.mean(0)))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = y.mean(0) - rotation @ x.mean(0)
    return transform @ estimated


def metrics(estimated, truth):
    aligned = align_se3(estimated, truth)
    first = truth[0] @ np.linalg.inv(estimated[0]) @ estimated
    relative_est = np.linalg.inv(estimated[:-1]) @ estimated[1:]
    relative_gt = np.linalg.inv(truth[:-1]) @ truth[1:]
    relative_error = np.linalg.inv(relative_gt) @ relative_est
    def rms(values):
        return float(np.sqrt(np.mean(np.square(values))))
    result = {'alignment': 'SE3, no scale fitting; ground truth used only by evaluator',
              'matched_frames': len(estimated),
              'ate_se3_rmse_m': rms(np.linalg.norm(aligned[:, :3, 3]-truth[:, :3, 3], axis=1)),
              'ate_first_pose_rmse_m': rms(np.linalg.norm(first[:, :3, 3]-truth[:, :3, 3], axis=1)),
              'rpe_adjacent_translation_rmse_m': rms(np.linalg.norm(relative_error[:, :3, 3], axis=1)),
              'rpe_adjacent_rotation_rmse_deg': rms(np.rad2deg(Rotation.from_matrix(relative_error[:, :3, :3]).magnitude())),
              'ground_truth_path_m': float(np.linalg.norm(np.diff(truth[:, :3, 3], axis=0), axis=1).sum()),
              'estimated_path_m': float(np.linalg.norm(np.diff(estimated[:, :3, 3], axis=0), axis=1).sum())}
    return result, aligned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('trajectory', type=Path)
    args = parser.parse_args()
    sequence = json.loads((args.capture / 'sequence.json').read_text())
    trajectory = json.loads(args.trajectory.read_text())
    estimates = trajectory['frames']
    truth = {f['file']: f for f in sequence['frames']}
    if len(estimates) < 3 or any(f['file'] not in truth or f['time_s'] != truth[f['file']]['time_s'] for f in estimates):
        raise ValueError('Need at least three timestamp-matched poses')
    gt = np.asarray([truth[f['file']]['camera_to_world'] for f in estimates])
    predicted = np.asarray([f['camera_to_world'] for f in estimates])
    if not np.isfinite([gt, predicted]).all():
        raise ValueError('Non-finite poses')
    report, aligned = metrics(predicted, gt)
    report['input_frames'] = len(sequence['frames'])
    report['coverage_fraction'] = len(estimates) / len(sequence['frames'])
    report['pose_source'] = trajectory.get('pose_source', 'unspecified')
    output = args.trajectory.parent
    (output / 'evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, (a, b), title in zip(axes, [(0, 1), (0, 2)], ['XY trajectory', 'XZ trajectory']):
        ax.plot(gt[:, a, 3], gt[:, b, 3], label='MuJoCo reference', color='#3a6cba')
        ax.plot(aligned[:, a, 3], aligned[:, b, 3], label='cuVSLAM (SE3 aligned)', color='#de7a22')
        ax.set(xlabel='XYZ'[a]+' (m)', ylabel='XYZ'[b]+' (m)', title=title, aspect='equal')
        ax.grid(alpha=.3)
        ax.legend()
    fig.suptitle(f"cuVSLAM trajectory | {len(estimates)}/{len(sequence['frames'])} frames | ATE {report['ate_se3_rmse_m']:.3f} m")
    fig.tight_layout()
    fig.savefig(output / 'trajectory.png', dpi=140)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
