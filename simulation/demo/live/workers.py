"""Dispatch the independent RGB-D inference, SLAM, and mesh workers."""
import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from worker_common import (FUSION_DEPTH_SOURCE, cloud_voxel_indices, fusion_depth,
                           keep_keyframe, map_refresh_due,
                           startup_tracking_state, status, write_json, write_npz)
from worker_inference import inference
from worker_tracking import tracking
from worker_reconstruction import build_surface, finalize, mesh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['infer', 'slam', 'mesh', 'finalize'])
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    status(args.session, args.stage, 'Đang khởi động')
    try:
        if args.stage == 'slam':
            tracking(args.session)
        else:
            {'infer': inference, 'mesh': mesh, 'finalize': finalize}[args.stage](args.session)
    except Exception as error:
        status(args.session, args.stage, 'LỖI: ' + str(error))
        traceback.print_exc()
        raise


if __name__ == '__main__':
    main()
