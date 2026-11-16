"""Compare the classified scan with the existing conservative raw-depth grid."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/navigation'))
from grid import NavigationGrid, build_grid, FREE, OCCUPIED, UNKNOWN


def stats(grid):
    return dict(free=int((grid.cells==FREE).sum()), occupied=int((grid.cells==OCCUPIED).sum()),
                unknown=int((grid.cells==UNKNOWN).sum()), traversable=int((~grid.blocked).sum()))


def main():
    session = ROOT/'datasets/processed/scan3d_fusion/room-ground-segmented-20261114'
    report = json.loads((session/'ground_report.json').read_text())
    assert report['frames']==3198 and report['depth_modified'] is False
    assert report['source_depth_sha256']==report['output_depth_sha256']
    original = json.loads(Path(report['sequence']).read_text())
    sequence = json.loads((session/'live_sequence.json').read_text())
    assert sequence['intrinsics']==original['intrinsics']
    assert [(f['camera_to_world'], f['time_s']) for f in sequence['frames']]==[(f['camera_to_world'], f['time_s']) for f in original['frames']]
    before = NavigationGrid.load(ROOT/'datasets/processed/scan3d_fusion/navigation-grid-20261114.npz')
    grid_path = session/'grid.npz'
    after = build_grid(session)
    if grid_path.exists():
        saved = NavigationGrid.load(grid_path)
        np.testing.assert_array_equal(saved.cells, after.cells)
        np.testing.assert_array_equal(saved.origin, after.origin)
        assert (saved.resolution, saved.clearance_m)==(after.resolution, after.clearance_m)
    else:
        after.save(grid_path)
    assert not after.demo_only
    result = dict(replay=report, poses_unchanged=True, raw_depth_grid=stats(before), classified_depth_grid=stats(after),
                  resolution_m=after.resolution, clearance_m=after.clearance_m,
                  scope='Estimated poses and measured fusion depth; no ground truth geometry or floor completion')
    (ROOT/'evaluation/ground_segmentation_results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), constrained_layout=True)
    cmap = ListedColormap(['#c6c9cd', '#faf8f2', '#b45146'])
    for ax, grid, title in zip(axes, [before, after], ['Raw depth height cuts', 'Ground / obstacle / uncertain labels']):
        states = np.where(grid.cells==UNKNOWN, 0, np.where(grid.cells==FREE, 1, 2))
        extent = [grid.origin[0], grid.origin[0]+grid.cells.shape[1]*grid.resolution,
                  grid.origin[1], grid.origin[1]+grid.cells.shape[0]*grid.resolution]
        ax.imshow(states, origin='lower', extent=extent, cmap=cmap, vmin=0, vmax=2, interpolation='nearest')
        numbers = stats(grid)
        ax.set(title=f"{title}\nFree {numbers['free']:,} | occupied {numbers['occupied']:,} | unknown {numbers['unknown']:,}",
               xlabel='X (m)', ylabel='Y (m)')
        ax.set_aspect('equal')
    fig.suptitle(f"Measured-depth occupancy — unknown stays blocked; {stats(after)['traversable']} traversable cells after 25cm inflation", fontsize=12)
    fig.legend(handles=[Patch(color=c, label=t) for c,t in zip(cmap.colors, ['Unknown', 'Free', 'Occupied'])], loc='outside lower center', ncol=3)
    fig.savefig(ROOT/'evaluation/ground_segmentation_costmaps.png', dpi=160)
    plt.close(fig)
    best, most = None, -1
    for frame in sequence['frames'][::64]:
        with np.load(session/frame['file']) as sample:
            count = int((sample['ground_labels']==1).sum())
            if count>most:
                best, most = dict(sample), count
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    image = axes[0].imshow(np.ma.masked_equal(best['depth'], 0)/1000, vmin=0, vmax=4, cmap='cividis')
    fig.colorbar(image, ax=axes[0], label='Optical depth (m)', shrink=.8)
    axes[0].set_title('Original fusion depth — no plane replacement')
    axes[1].imshow(best['ground_labels'], cmap=ListedColormap(['#b4b4b4', '#bc974c', '#b44e43']), vmin=0, vmax=2)
    axes[1].set_title('Uncertain (gray) / ground (gold) / obstacle (red)')
    for ax in axes:
        ax.set_axis_off()
    fig.savefig(ROOT/'evaluation/ground_segmentation_frame.png', dpi=160)
    plt.close(fig)
    with np.load(session/'ground_layers.npz') as layers:
        fig = plt.figure(figsize=(10, 7), constrained_layout=True)
        ax = fig.add_subplot(projection='3d')
        for name, color, alpha in [('uncertain', '#b4b4b4', .12), ('ground', '#bc974c', .8), ('obstacle', '#b44e43', .35)]:
            xyz = layers[name]
            xyz = xyz[::max(1, len(xyz)//40000)]
            ax.scatter(*xyz.T, s=.3, color=color, alpha=alpha, label=name)
        ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', zlim=(-.2, 2.1),
               title='Measured 3D layers — original depth and saved poses unchanged')
        ax.view_init(elev=48, azim=-50); ax.legend(markerscale=8)
        fig.savefig(ROOT/'evaluation/ground_segmentation_layers.png', dpi=160)
        plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__=='__main__':
    main()
