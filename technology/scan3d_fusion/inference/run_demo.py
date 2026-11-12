#!/usr/bin/env python3
"""Run the bundled DELTAR NYU checkpoint on demo or simulated data."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
DELTAR = ROOT / "technology/scan3d_fusion/model/deltar"
DEFAULT_OUTPUT = ROOT / "datasets/processed/scan3d_fusion/pretrained_demo"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-frames", type=int, default=3,
                        help="frames to run; use 0 for the full demo sequence")
    parser.add_argument("--data-root", type=Path, default=DELTAR / "data/demo",
                        help="DELTAR-format directory containing data.json and capture H5 files")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    cli = parser.parse_args()
    if cli.max_frames < 0:
        parser.error("--max-frames must be non-negative")

    weights = DELTAR / "weights/nyu.pt"
    dataset_index = cli.data_root / "data.json"
    if not weights.is_file() or not dataset_index.is_file():
        raise SystemExit("missing DELTAR model/data; see technology/scan3d_fusion/README.md")

    # DELTAR's global config parser reads a single @config argument at import time.
    config_path = ROOT / "technology/scan3d_fusion/config/deltar_eval.txt"
    sys.argv = [str(DELTAR / "evaluate.py"), str(config_path)]
    sys.path.insert(0, str(DELTAR))

    import torch
    import timm

    # The pretrained DELTAR checkpoint supplies the image encoder weights itself.
    original_create_model = timm.create_model

    def create_model(name, *args, **kwargs):
        kwargs["pretrained"] = False
        return original_create_model(name, *args, **kwargs)

    timm.create_model = create_model

    from src.config import args as model_args
    from src.dataloader.zjuL5 import ZJUL5
    from src.models.deltar import Deltar

    model_args.data_path_eval = str(cli.data_root.resolve())
    model_args.filenames_file_eval = str(dataset_index.resolve())

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = Deltar(n_bins=model_args.n_bins, min_val=model_args.min_depth,
                   max_val=model_args.max_depth, norm="linear").to(device)
    state = torch.load(str(weights), map_location=device)
    model.load_state_dict(state)
    model.eval()

    loader = ZJUL5(model_args, "online_eval").data
    cli.output.mkdir(parents=True, exist_ok=True)
    count = 0
    abs_error_sum = 0.0
    squared_error_sum = 0.0
    absolute_relative_error_sum = 0.0
    valid_pixel_count = 0
    with torch.no_grad():
        for batch in loader:
            image = batch["image"].to(device)
            extra = batch["additional"]
            prediction = model({
                "rgb": image,
                "additional": {
                    "hist_data": extra["hist_data"].to(device),
                    "rect_data": extra["rect_data"].to(device),
                    "mask": extra["mask"].to(device),
                    "patch_info": extra["patch_info"],
                },
            })[1]
            prediction = torch.nn.functional.interpolate(
                prediction, size=image.shape[-2:], mode="bilinear", align_corners=True
            ).squeeze().clamp(model_args.min_depth, model_args.max_depth)
            prediction_np = prediction.cpu().numpy()
            ground_truth = batch["depth"].squeeze().numpy()
            valid = np.isfinite(ground_truth) & (ground_truth > model_args.min_depth) & (
                ground_truth < model_args.max_depth)
            if valid.any():
                error = prediction_np[valid] - ground_truth[valid]
                absolute_error = np.abs(error)
                abs_error_sum += float(absolute_error.sum())
                squared_error_sum += float(np.square(error).sum())
                absolute_relative_error_sum += float(
                    (absolute_error / ground_truth[valid]).sum())
                valid_pixel_count += int(error.size)
            depth_mm = (prediction_np * 1000).astype("uint16")
            filename = Path(batch["image_path"][0]).stem + "_depth.png"
            from PIL import Image
            Image.fromarray(depth_mm).save(cli.output / filename)
            count += 1
            if cli.max_frames and count >= cli.max_frames:
                break

    if count == 0:
        raise RuntimeError("DELTAR demo loader returned no frames")
    metrics = (f"mae_m={abs_error_sum / valid_pixel_count:.4f}; "
               f"rmse_m={np.sqrt(squared_error_sum / valid_pixel_count):.4f}; "
               f"abs_rel={absolute_relative_error_sum / valid_pixel_count:.4f}") \
        if valid_pixel_count else "metrics=n/a"
    print(f"device={device}; frames={count}; {metrics}; depth_png={cli.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
