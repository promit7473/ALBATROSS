"""Colour-histogram appearance model used to re-identify the locked person.

A hue/saturation histogram of the torso and of the legs (separately, so "red
shirt + blue jeans" differs from "blue shirt + red trousers"), compared with the
Bhattacharyya distance (0 = identical, 1 = disjoint).
"""
import cv2
import numpy as np

H_BINS, S_BINS = 16, 8


def _hist(hsv_patch):
    if hsv_patch.size == 0:
        return np.zeros(H_BINS * S_BINS, np.float32)
    h = cv2.calcHist([hsv_patch], [0, 1], None, [H_BINS, S_BINS], [0, 180, 0, 256])
    h = h.flatten().astype(np.float32)
    s = h.sum()
    return h / s if s > 0 else h


def descriptor(bgr, box):
    """Concatenated torso + legs histograms for a box (x1, y1, x2, y2) in pixels."""
    H, W = bgr.shape[:2]
    x1, y1, x2, y2 = box
    # shrink horizontally to keep background out of the person's silhouette
    w = x2 - x1
    x1, x2 = int(max(0, x1 + 0.2 * w)), int(min(W, x2 - 0.2 * w))
    y1, y2 = int(max(0, y1)), int(min(H, y2))
    if x2 - x1 < 2 or y2 - y1 < 4:
        return None
    hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    h = y2 - y1
    torso = hsv[int(0.18 * h):int(0.55 * h)]
    legs = hsv[int(0.55 * h):int(0.95 * h)]
    return np.concatenate([_hist(torso), _hist(legs)])


def distance(a, b):
    """Mean Bhattacharyya distance of the torso and legs parts, in [0, 1]."""
    if a is None or b is None:
        return 1.0
    n = H_BINS * S_BINS
    d = [cv2.compareHist(a[i:i + n], b[i:i + n], cv2.HISTCMP_BHATTACHARYYA) for i in (0, n)]
    return float(np.mean(d))


class AppearanceModel:
    """Exponential moving average of a target's descriptor."""

    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.mean = None

    def update(self, desc):
        if desc is None:
            return
        self.mean = desc.copy() if self.mean is None else (1 - self.alpha) * self.mean + self.alpha * desc

    def distance(self, desc):
        return distance(self.mean, desc)
