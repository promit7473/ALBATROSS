"""Multi-object tracker in the style of ByteTrack (Zhang et al., ECCV 2022).

Per frame:
  1. Kalman-predict every track, then warp the predictions by the estimated
     camera motion (the drone moves, so the whole image shifts).
  2. Stage 1 -- associate HIGH-confidence detections with all confirmed tracks
     (tracked + lost) by Hungarian matching on 1 - BIoU * score, same class
     only. BIoU = IoU of boxes enlarged by a buffer (C-BIoU, Yang et al. 2023),
     which still overlaps for small, fast-moving boxes.
  3. Stage 2 -- associate the LOW-confidence leftovers with tracks still
     unmatched that were tracked last frame (occluded people often score low;
     using these keeps their identity instead of dropping the track).
  4. Tentative (new) tracks get the remaining high detections; a tentative
     track is confirmed after `min_hits` consecutive matches, else deleted.
  5. Unmatched confirmed tracks become LOST and coast on their Kalman
     prediction for `max_lost` frames before being removed.
  6. Unmatched high detections above `new_track_thresh` start tentative tracks.
"""
import itertools

import numpy as np
from scipy.optimize import linear_sum_assignment

from tracker_perception.kalman_box import KalmanBox

TENTATIVE, TRACKED, LOST = 0, 1, 2


def buffered(boxes, b):
    """Enlarge xyxy boxes by fraction b of their width/height on every side."""
    w = (boxes[:, 2] - boxes[:, 0])[:, None]
    h = (boxes[:, 3] - boxes[:, 1])[:, None]
    return boxes + np.hstack([-w, -h, w, h]) * b


def iou_matrix(a, b):
    """Pairwise IoU between boxes a (N,4) and b (M,4), xyxy."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), np.float32)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / np.maximum(area_a[:, None] + area_b[None, :] - inter, 1e-6)


def assign(cost, thresh):
    """Hungarian assignment; returns matches [(row, col)], unmatched rows, unmatched cols."""
    if cost.size == 0:
        return [], list(range(cost.shape[0])), list(range(cost.shape[1]))
    rows, cols = linear_sum_assignment(cost)
    matches = [(r, c) for r, c in zip(rows, cols) if cost[r, c] <= thresh]
    mr = {r for r, _ in matches}
    mc = {c for _, c in matches}
    return (matches, [r for r in range(cost.shape[0]) if r not in mr],
            [c for c in range(cost.shape[1]) if c not in mc])


class Track:
    _ids = itertools.count(1)

    def __init__(self, box, score, cls, confirmed):
        self.id = next(Track._ids)
        self.kf = KalmanBox(box)
        self.score, self.cls = float(score), int(cls)
        self.state = TRACKED if confirmed else TENTATIVE
        self.hits, self.lost_frames, self.age = 1, 0, 0

    @property
    def box(self):
        return self.kf.box

    def predict(self):
        if self.state != TRACKED:
            self.kf.x[7] = 0.0           # no size change while unobserved (else the box collapses)
        self.kf.predict()
        self.age += 1

    def update(self, box, score):
        self.kf.update(box)
        self.score = float(score)
        self.hits += 1
        self.lost_frames = 0
        if self.state == LOST:
            self.state = TRACKED


class ByteTracker:
    def __init__(self, high_thresh=0.45, low_thresh=0.1, new_track_thresh=0.55,
                 match_thresh=0.8, second_match_thresh=0.5, tentative_match_thresh=0.7,
                 min_hits=3, max_lost=45, buffer=0.3):
        self.high, self.low, self.new_thresh = high_thresh, low_thresh, new_track_thresh
        self.match_thresh, self.second_thresh = match_thresh, second_match_thresh
        self.tentative_thresh = tentative_match_thresh
        self.min_hits, self.max_lost = min_hits, max_lost
        self.buffer = buffer
        self.tracks = []
        self.frame = 0

    def _cost(self, tracks, boxes, scores, classes, fuse_score, buffer=0.0):
        if not tracks or len(boxes) == 0:
            return np.zeros((len(tracks), len(boxes)), np.float32)
        tb = np.array([t.box for t in tracks])
        iou = iou_matrix(buffered(tb, buffer), buffered(boxes, buffer)) if buffer else iou_matrix(tb, boxes)
        sim = iou * scores[None, :] if fuse_score else iou
        cost = 1.0 - sim
        cls_t = np.array([t.cls for t in tracks])
        cost[cls_t[:, None] != classes[None, :]] = 1e3      # never match across classes
        return cost

    def update(self, dets, camera_motion=None):
        """dets: Detections; camera_motion: 2x3 affine prev->current frame.
        Returns the list of confirmed (TRACKED or LOST) tracks."""
        self.frame += 1
        for t in self.tracks:
            t.predict()
            if camera_motion is not None:
                t.kf.warp(camera_motion)

        hi = dets.scores >= self.high
        lo = (dets.scores >= self.low) & ~hi
        D_hi, D_lo = dets.subset(hi), dets.subset(lo)

        confirmed = [t for t in self.tracks if t.state != TENTATIVE]
        tentative = [t for t in self.tracks if t.state == TENTATIVE]

        # stage 1: high detections <-> confirmed tracks
        cost = self._cost(confirmed, D_hi.boxes, D_hi.scores, D_hi.classes, fuse_score=True,
                          buffer=self.buffer)
        m1, um_t, um_d = assign(cost, self.match_thresh)
        for ti, di in m1:
            confirmed[ti].update(D_hi.boxes[di], D_hi.scores[di])

        # stage 2: low detections <-> still-unmatched tracks that were visible last frame
        rest = [confirmed[i] for i in um_t if confirmed[i].state == TRACKED]
        cost = self._cost(rest, D_lo.boxes, D_lo.scores, D_lo.classes, fuse_score=False)
        m2, um_rest, _ = assign(cost, self.second_thresh)
        for ti, di in m2:
            rest[ti].update(D_lo.boxes[di], D_lo.scores[di])
        matched_ids = {rest[ti].id for ti, _ in m2} | {confirmed[ti].id for ti, _ in m1}
        for t in confirmed:
            if t.id not in matched_ids:
                t.state = LOST
                t.lost_frames += 1

        # stage 3: remaining high detections <-> tentative tracks
        left = D_hi.subset(np.array([i in um_d for i in range(len(D_hi))], bool)) if len(D_hi) else D_hi
        cost = self._cost(tentative, left.boxes, left.scores, left.classes, fuse_score=True,
                          buffer=self.buffer)
        m3, um_tent, um_left = assign(cost, self.tentative_thresh)
        for ti, di in m3:
            t = tentative[ti]
            t.update(left.boxes[di], left.scores[di])
            if t.hits >= self.min_hits:
                t.state = TRACKED
        dead = {tentative[i].id for i in um_tent}

        # births
        first_frame = self.frame == 1
        for di in um_left:
            if left.scores[di] >= self.new_thresh:
                self.tracks.append(Track(left.boxes[di], left.scores[di], left.classes[di],
                                         confirmed=first_frame))

        self.tracks = [t for t in self.tracks
                       if t.id not in dead and not (t.state == LOST and t.lost_frames > self.max_lost)]
        return [t for t in self.tracks if t.state != TENTATIVE]
