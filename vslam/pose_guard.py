"""Decide whether a visual pose may enter the map."""
from collections import deque

from .tracker import local_motion_error


class PoseGuard:
    def __init__(self):
        self.accepted = None
        self.recent = deque(maxlen=6)
        self.hold_since = None
        self.candidate = None
        self.good_frames = 0
        self.lost = False

    def disagreement(self, visual, sensor):
        last_visual, last_sensor = self.accepted
        latest = local_motion_error(last_visual, visual, last_sensor, sensor)
        if len(self.recent) < 2:
            return latest
        old_visual, old_sensor = self.recent[0]
        earlier = local_motion_error(old_visual, visual, old_sensor, sensor)
        return max(latest[0], earlier[0]), max(latest[1], earlier[1])

    def update(self, timestamp, visual, sensor, *, verified_match=False):
        error = None
        if visual is not None and self.accepted is not None:
            error = self.disagreement(visual, sensor)
        plausible = visual is not None and (error is None or self.trusted(error))
        expired = self.hold_since is not None and timestamp - self.hold_since >= 3.0
        # A delayed frame can still provide positive visual evidence after a gap in capture.
        if expired and not (verified_match and plausible):
            self.lost = True
            self.good_frames = 0
            self.candidate = None
            return 'lost', error
        if not plausible:
            if self.hold_since is None:
                self.hold_since = timestamp
            self.candidate = None
            self.good_frames = 0
            return 'hold', error

        state = 'accepted'
        if self.hold_since is not None:
            if self.candidate is not None:
                step = local_motion_error(self.candidate[0], visual, self.candidate[1], sensor)
                if not self.plausible(step):
                    if expired:
                        self.good_frames = 0
                        self.candidate = None
                        self.lost = True
                        return 'lost', error
                    self.good_frames = 0
            self.candidate = visual, sensor
            self.good_frames += 1
            if self.good_frames < 3:
                return 'hold', error
            self.hold_since = None
            self.candidate = None
            self.good_frames = 0
            self.lost = False
            state = 'recovered'
            self.recent.clear()

        self.accepted = visual, sensor
        self.recent.append(self.accepted)
        return state, error

    @staticmethod
    def trusted(error):
        # Keep near-limit samples provisional until the displacement from the old anchor is verified.
        return error[0] <= .08 and error[1] <= .12

    @staticmethod
    def plausible(error):
        # shortcut: fixed simulation thresholds; calibrate against real synchronized sensor logs.
        translation, angle = error
        return translation <= .10 and angle <= .15
