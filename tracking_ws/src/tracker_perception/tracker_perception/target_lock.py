"""Operator target lock with appearance-based re-identification.

While the locked track is visible we learn its colour appearance (only from
frames where it does not overlap other people, so the model is not polluted).
If the tracker loses it -- occlusion behind a tree, leaving the frame, an ID
switch -- we look for it among *new* tracks:

  * a candidate must be the same class, never seen at the same time as the
    target (two people visible together cannot be the same person),
  * pass an appearance gate (Bhattacharyya distance), a size gate, and a
    spatial gate around the last seen position that widens with time lost,
  * and win for `confirm_frames` consecutive frames before the lock moves to it.

After `max_lost_time` seconds without a match the lock is released.
"""
import numpy as np

from tracker_perception.appearance import AppearanceModel, descriptor
from tracker_perception.byte_tracker import TRACKED, iou_matrix

NONE, TRACKING, LOST = 0, 1, 2


class TargetLock:
    def __init__(self, max_app_dist=0.35, max_lost_time=20.0, confirm_frames=3,
                 gate_base=0.15, gate_growth=0.08):
        self.max_app_dist = max_app_dist
        self.max_lost_time = max_lost_time
        self.confirm_frames = confirm_frames
        self.gate_base, self.gate_growth = gate_base, gate_growth
        self.release()

    def release(self):
        self.track_id = -1
        self.cls = None
        self.status = NONE
        self.appearance = AppearanceModel()
        self.last_box = None
        self.last_seen = None
        self.lost_for = 0.0
        self.co_visible = set()      # ids seen together with the target -> not the target
        self._candidate, self._candidate_count = None, 0
        self.debug = []              # last re-identification candidates, for logging

    def lock(self, track, frame, now):
        self.release()
        self.track_id, self.cls = track.id, track.cls
        self.status = TRACKING
        self.last_box = track.box.copy()
        self.last_seen = now
        self.appearance.update(descriptor(frame, track.box))

    def update(self, tracks, frame, now):
        """tracks: confirmed tracks of this frame. Returns the locked Track or None."""
        if self.status == NONE:
            return None
        by_id = {t.id: t for t in tracks}
        t = by_id.get(self.track_id)

        if t is not None and t.state == TRACKED:
            self._on_visible(t, tracks, frame, now)
            return t

        self.status = LOST                         # last_box stays the last *observed* box
        self.lost_for = now - self.last_seen
        if self.lost_for > self.max_lost_time:
            self.release()
            return None
        new = self._reidentify(tracks, frame)
        if new is not None:
            self.track_id = new.id
            self._on_visible(new, tracks, frame, now)
            return new
        return t

    # ------------------------------------------------------------------ internals

    def _on_visible(self, t, tracks, frame, now):
        self.status = TRACKING
        self.last_box = t.box.copy()
        self.last_seen = now
        self.lost_for = 0.0
        self._candidate, self._candidate_count = None, 0
        others = [o for o in tracks if o.id != t.id and o.state == TRACKED]
        self.co_visible.update(o.id for o in others if o.cls == t.cls)
        # learn appearance only when the target is not overlapping anyone
        if not others or iou_matrix(t.box[None], np.array([o.box for o in others])).max() < 0.05:
            self.appearance.update(descriptor(frame, t.box))

    def _reidentify(self, tracks, frame):
        H, W = frame.shape[:2]
        diag = float(np.hypot(W, H))
        gate = (self.gate_base + self.gate_growth * self.lost_for) * diag
        lb = self.last_box
        lc = np.array([(lb[0] + lb[2]) / 2, (lb[1] + lb[3]) / 2])
        lh = lb[3] - lb[1]

        best, best_score = None, np.inf
        self.debug = []
        for c in tracks:
            if c.state != TRACKED or c.cls != self.cls or c.id == self.track_id:
                continue
            if c.id in self.co_visible:
                self.debug.append((c.id, 'co-visible'))
                continue
            b = c.box
            h = b[3] - b[1]
            ratio = h / max(lh, 1.0)
            d_px = np.hypot((b[0] + b[2]) / 2 - lc[0], (b[1] + b[3]) / 2 - lc[1])
            d_app = self.appearance.distance(descriptor(frame, b))
            self.debug.append((c.id, f'size x{ratio:.2f} dist {d_px:.0f}/{gate:.0f}px app {d_app:.2f}'))
            if not 1 / 3 < ratio < 3.0 or d_px > gate or d_app > self.max_app_dist:
                continue
            score = d_app + 0.5 * d_px / gate
            if score < best_score:
                best, best_score = c, score

        if best is None:
            self._candidate, self._candidate_count = None, 0
            return None
        if self._candidate == best.id:
            self._candidate_count += 1
        else:
            self._candidate, self._candidate_count = best.id, 1
        return best if self._candidate_count >= self.confirm_frames else None
