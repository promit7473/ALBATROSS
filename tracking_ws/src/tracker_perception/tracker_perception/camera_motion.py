"""Global camera-motion estimation between consecutive frames (as in BoT-SORT's GMC).

The camera is on a drone that yaws and translates, so every tracked box shifts
in the image even when the person stands still. Without compensation a 10 deg
yaw between two frames moves a box ~80 px -- more than a distant person's width
-- and IoU association fails. We track sparse corners from the previous frame
with pyramidal Lucas-Kanade optical flow, fit a similarity transform with
RANSAC (moving people are outliers; their boxes are also masked out), and the
tracker warps its Kalman predictions with it.
"""
import cv2
import numpy as np

IDENTITY = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class CameraMotion:
    def __init__(self, downscale=2, max_corners=300):
        self.ds = downscale
        self.max_corners = max_corners
        self.prev = None

    def estimate(self, bgr, boxes=()):
        """2x3 affine mapping previous-frame pixels to this frame (full resolution)."""
        small = cv2.resize(bgr, None, fx=1 / self.ds, fy=1 / self.ds, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        prev, self.prev = self.prev, gray
        if prev is None:
            return IDENTITY.copy()

        mask = np.full(prev.shape, 255, np.uint8)
        for x1, y1, x2, y2 in boxes:                 # do not track features on moving objects
            mask[int(y1 / self.ds):int(y2 / self.ds) + 1, int(x1 / self.ds):int(x2 / self.ds) + 1] = 0
        pts = cv2.goodFeaturesToTrack(prev, self.max_corners, 0.01, 8, mask=mask, blockSize=3)
        if pts is None or len(pts) < 10:
            return IDENTITY.copy()
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(prev, gray, pts, None, winSize=(21, 21), maxLevel=3)
        ok = status.ravel() == 1
        if ok.sum() < 10:
            return IDENTITY.copy()
        A, inliers = cv2.estimateAffinePartial2D(pts[ok], nxt[ok], method=cv2.RANSAC,
                                                 ransacReprojThreshold=2.0)
        if A is None or inliers.sum() < 8:
            return IDENTITY.copy()
        A[:, 2] *= self.ds                           # translation back to full resolution
        return A
