"""Run the shared DELTAR RGB + ToF fusion without a simulator dependency."""
from copy import copy
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from .depth_support import tof_support

BASE = Path(__file__).resolve().parent
IMAGE_SHAPE = (480, 640)


class Fusion:
    def __init__(self, intrinsics, camera_to_tof, weights=None):
        self.intrinsics = np.asarray(intrinsics, dtype=float)
        if (self.intrinsics.shape != (4,) or not np.isfinite(self.intrinsics).all()
                or np.any(self.intrinsics[:2] <= 0)):
            raise ValueError('K must contain finite fx, fy, cx, cy with positive focal lengths')
        rotation = np.asarray(camera_to_tof['rotation_matrix'], dtype=float)
        translation = np.asarray(camera_to_tof['translation_m'], dtype=float)
        if (rotation.shape != (3, 3) or translation.shape != (3,)
                or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
                or not np.allclose(rotation, Rotation.from_matrix(rotation).as_matrix(), atol=1e-4)):
            raise ValueError('camera_to_tof must contain a proper rotation and translation in metres')
        self.camera_to_tof = dict(rotation_matrix=rotation, translation_m=translation)

        import timm
        import torch

        self.torch = torch
        torch.set_num_threads(2)
        bundle = BASE / 'deltar'
        weights = Path(weights) if weights is not None else bundle / 'weights/nyu.pt'
        if not weights.is_file():
            raise FileNotFoundError(f'Missing DELTAR checkpoint: {weights}')
        sys.path.insert(0, str(bundle))
        argv = sys.argv
        try:
            sys.argv = [str(bundle / 'evaluate.py'), str(BASE / 'deltar_eval.txt')]
            from src.config import args
            from src.models.deltar import Deltar
            from src.dataloader.zjuL5 import ToTensor
            from src.utils.dataloader import patch_info_from_rect_data, sample_point_from_hist_parallel
        finally:
            sys.argv = argv
        args = copy(args)
        create = timm.create_model
        try:
            def create_local(name, *positional, **keywords):
                keywords['pretrained'] = False
                return create(name, *positional, **keywords)

            timm.create_model = create_local
            self.model = Deltar(n_bins=args.n_bins, min_val=args.min_depth,
                                max_val=args.max_depth, norm='linear').cuda().eval()
        finally:
            timm.create_model = create
        self.model.load_state_dict(torch.load(str(weights), map_location='cuda'))
        self.args = args
        self.normalize = ToTensor('online_eval').normalize
        self.patch_info = patch_info_from_rect_data
        self.sample_hist = sample_point_from_hist_parallel

    def depth_mm(self, rgb, hist, rect, valid):
        if rgb.shape != (*IMAGE_SHAPE, 3) or rgb.dtype != np.uint8:
            raise ValueError('Expected a rectified 640x480 uint8 RGB frame')
        if hist.shape != (64, 2) or rect.shape != (64, 4) or valid.shape != (64,):
            raise ValueError('Expected 64 ToF distributions, rectangles and validity flags')
        if (not np.isfinite(hist).all() or not np.isfinite(rect).all()
                or not np.isin(valid, [0, 1]).all() or np.any(hist[:, 1] < 0)):
            raise ValueError('Invalid ToF statistics, rectangles or validity flags')
        valid = valid.astype(bool, copy=False)
        if np.any(hist[valid, 1] <= 0):
            raise ValueError('Valid ToF ranges require positive standard deviation')
        if not valid.any():
            return np.zeros(IMAGE_SHAPE, dtype=np.uint16)
        torch = self.torch
        image = torch.from_numpy((rgb.astype(np.float32) / 255.).transpose(2, 0, 1))
        image = self.normalize(image).unsqueeze(0).cuda()
        hist_tensor, rect_tensor, mask = map(torch.from_numpy, (hist, rect, valid))
        patch_info = self.patch_info(rect_tensor)
        for key, value in patch_info.items():
            patch_info[key] = ({k: v.unsqueeze(0) for k, v in value.items()}
                               if isinstance(value, dict) else torch.tensor([value]))
        with torch.no_grad():
            prediction = self.model({'rgb': image, 'additional': {
                'hist_data': self.sample_hist(hist_tensor, mask, self.args).unsqueeze(0).cuda(),
                'rect_data': rect_tensor.float().unsqueeze(0).cuda(),
                'mask': mask.unsqueeze(0).cuda(),
                'patch_info': patch_info}})[1]
            depth = torch.nn.functional.interpolate(
                prediction, size=IMAGE_SHAPE, mode='bilinear', align_corners=True
            ).squeeze().cpu().numpy()
        depth = tof_support(depth, hist, rect, valid, self.intrinsics, self.camera_to_tof)
        return np.rint(np.clip(depth, 0, 65.535) * 1000).astype(np.uint16)
