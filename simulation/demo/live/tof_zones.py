"""Render-time projection and reduction for the simulated ToF zones."""
import math

import numpy as np


def closest_cluster(values, sensor):
    """Pure current closest-significant-return selection, before sensor noise.

    Bounds are inclusive, matching the existing capture reducer. Returned moments
    are over the original in-range rays inside the chosen histogram-bin interval.
    """
    values = np.asarray(values).ravel()
    values = values[np.isfinite(values) & (values >= sensor["min_range_m"]) &
                    (values <= sensor["max_range_m"])]
    if len(values) < 32:
        return None
    counts, edges = np.histogram(values, bins=np.arange(0, 4.04, .04))
    occupied = np.flatnonzero(counts >= max(3, counts.max() * .1))
    if not len(occupied):
        return None
    clusters = np.split(occupied, np.flatnonzero(np.diff(occupied) > 1) + 1)
    cluster = next((c for c in clusters if counts[c].sum() >= len(values)*.2), None)
    if cluster is None:
        return None
    low, high = edges[cluster[0]], edges[cluster[-1]+1]
    selected = values[(values >= low) & (values <= high)]
    return dict(mean_m=float(selected.mean()), std_m=float(selected.std()),
                low_m=float(low), high_m=float(high), selected_count=len(selected),
                valid_count=len(values), significant_clusters=len(clusters),
                whole_mean_m=float(values.mean()), whole_std_m=float(values.std()))


def zone_statistics(depth, sensor, rng):
    """Approximate a zone return with many rays, in ToF optical-Z metres."""
    if depth.shape != (128, 128):
        raise ValueError("ToF sampling image must be 128 x 128")
    hist = np.zeros((64, 2), np.float32)
    valid = np.zeros(64, bool)
    for row in range(8):
        for col in range(8):
            values = depth[row*16:(row+1)*16, col*16:(col+1)*16].ravel()
            selected = closest_cluster(values, sensor)
            if selected is None:
                continue
            mean = selected["mean_m"]
            noise = sensor["noise"]
            bound = float(noise["near_accuracy_m"]) if mean <= .2 else mean*float(noise["far_accuracy_fraction"])
            sigma_noise = bound * float(noise["sigma_fraction_of_bound"])
            index = row*8+col
            hist[index] = [np.clip(mean+rng.normal(0, sigma_noise), .02, 4),
                           max(.005, math.hypot(selected["std_m"], sigma_noise))]
            valid[index] = True
    return hist, valid


def zone_rectangles(hist, rotation, translation, fovy, width, height, tof_fovy):
    """Project measured ToF zone corners through calibration, without RGB ground-truth depth."""
    if width != int(width) or height != int(height) or min(width, height) < 16:
        raise ValueError("image width/height must be pixel dimensions, not camera height in metres")
    rect = np.zeros((64, 4), np.float32)
    f = height / (2*math.tan(math.radians(fovy)/2))
    h = math.tan(math.radians(tof_fovy)/2)
    for i in range(64):
        row, col = divmod(i, 8)
        z = max(.1, float(hist[i, 0]))
        corners = np.array([[(2*c/8-1)*h*z, (1-2*r/8)*h*z, -z]
                            for r in (row, row+1) for c in (col, col+1)])
        camera = (corners - translation) @ rotation
        if np.any(camera[:, 2] >= 0):
            raise ValueError("zone projected behind camera; check extrinsic calibration")
        x = camera[:, 0] / -camera[:, 2]*f + width/2
        y = -camera[:, 1] / -camera[:, 2]*f + height/2
        rect[i] = [math.floor(y.min()), math.floor(x.min()),
                   math.ceil(y.max()), math.ceil(x.max())]
    return rect
