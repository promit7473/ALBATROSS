import numpy as np

from tracker_perception.byte_tracker import ByteTracker, iou_matrix
from tracker_perception.detector import Detections


def dets(boxes, scores, classes=None):
    n = len(boxes)
    return Detections(np.array(boxes, np.float32).reshape(n, 4), np.array(scores, np.float32),
                      np.array(classes if classes is not None else [0] * n, np.int32))


def test_iou():
    m = iou_matrix(np.array([[0, 0, 10, 10]], float), np.array([[0, 0, 10, 10], [5, 0, 15, 10], [20, 20, 30, 30]], float))
    assert np.allclose(m, [[1.0, 1 / 3, 0.0]])


def test_ids_stable_for_two_walkers_crossing_vertically_apart():
    trk = ByteTracker(min_hits=1)
    ids = None
    for k in range(30):
        a = [100 + 3 * k, 100, 130 + 3 * k, 180]
        b = [400 - 3 * k, 250, 430 - 3 * k, 330]
        out = trk.update(dets([a, b], [0.9, 0.9]))
        cur = sorted((t.box[1] < 200, t.id) for t in out)
        ids = ids or cur
        assert cur == ids


def test_low_score_detection_keeps_track_alive():
    trk = ByteTracker(min_hits=1)
    first = trk.update(dets([[100, 100, 130, 180]], [0.9]))[0].id
    for k in range(1, 10):                    # partially occluded: weak detections only
        out = trk.update(dets([[100 + k, 100, 130 + k, 180]], [0.2]))
        assert [t.id for t in out if t.state == 1] == [first]


def test_lost_track_recovers_same_id_after_short_gap():
    trk = ByteTracker(min_hits=1)
    first = trk.update(dets([[100, 100, 130, 180]], [0.9]))[0].id
    for _ in range(5):
        trk.update(dets([], []))
    out = trk.update(dets([[101, 100, 131, 180]], [0.9]))
    assert out[0].id == first


def test_classes_never_associate():
    trk = ByteTracker(min_hits=1)
    p = trk.update(dets([[100, 100, 130, 180]], [0.9], [0]))[0].id
    out = trk.update(dets([[100, 100, 130, 180]], [0.9], [2]))
    assert all(t.id != p or t.state == 2 for t in out)


def test_camera_motion_compensation_keeps_id_under_large_pan():
    trk = ByteTracker(min_hits=1)
    first = trk.update(dets([[300, 200, 315, 240]], [0.9]))[0].id
    # camera yaws: the static person jumps 80 px left (more than its width)
    A = np.array([[1.0, 0.0, -80.0], [0.0, 1.0, 0.0]])
    out = trk.update(dets([[220, 200, 235, 240]], [0.9]), camera_motion=A)
    assert [t.id for t in out if t.state == 1] == [first]


def test_camera_motion_estimate_recovers_shift():
    import cv2
    from tracker_perception.camera_motion import CameraMotion
    rng = np.random.default_rng(1)
    img = (rng.random((480, 640)) * 255).astype(np.uint8)
    img = cv2.GaussianBlur(img, (7, 7), 2)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    shifted = np.roll(img, (6, -10), axis=(0, 1))
    gmc = CameraMotion()
    gmc.estimate(img)
    A = gmc.estimate(shifted)
    assert abs(A[0, 2] + 10) < 1.5 and abs(A[1, 2] - 6) < 1.5
