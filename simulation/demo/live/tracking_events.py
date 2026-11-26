"""Bounded RGB-D flight recorder for tracking failures."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from worker_common import write_npz


class TrackingEvents:
    def __init__(self, session):
        self.session = session
        self.history = deque(maxlen=12)
        self.writer = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.remaining = 0
        self.previous = 'accepted'
        self.event = 0

    def add(self, t, batch, raw, corrected, state, sensor, stats=None):
        missing = np.full((4, 4), np.nan)
        stats = stats or {}
        self.history.append(dict(features=stats.get('features', -1), flow_pairs=stats.get('flow_pairs', -1),
                                 inliers=stats.get('inliers', -1), depth_inliers=stats.get('depth_inliers', -1), t=t, rgb=batch['rgb'], depth=batch['depth'], K=batch['K'],
                                 mount=batch['mount'], odometry=batch['odometry'], sensor=sensor,
                                 raw=missing if raw is None else raw,
                                 corrected=missing if corrected is None else corrected, state=state))
        if self.remaining:
            self.remaining -= 1
            if not self.remaining:
                self.save()
        elif self.previous in ('accepted', 'recovered') and state in ('hold', 'lost'):
            self.remaining = 3
        self.previous = state

    def save(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            self.pending.result()
        rows = list(self.history)
        path = self.session / f'tracking_event_{self.event % 4}.npz'
        def write():
            write_npz(path, **{k: np.stack([r[k] for r in rows]) for k in rows[0]})
        self.pending = self.writer.submit(write)
        self.event += 1

    def close(self):
        if self.remaining:
            self.save()
        self.writer.shutdown()
        if self.pending is not None:
            self.pending.result()
