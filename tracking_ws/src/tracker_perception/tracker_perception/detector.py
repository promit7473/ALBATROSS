"""YOLO object detector (Ultralytics) returning plain numpy arrays."""
from dataclasses import dataclass

import numpy as np


@dataclass
class Detections:
    boxes: np.ndarray      # (N, 4) x1, y1, x2, y2 [px]
    scores: np.ndarray     # (N,)
    classes: np.ndarray    # (N,) int class ids

    def __len__(self):
        return len(self.scores)

    def subset(self, mask):
        return Detections(self.boxes[mask], self.scores[mask], self.classes[mask])


class YoloDetector:
    """Thin wrapper around an Ultralytics model (.pt, ONNX or OpenVINO export).

    Only `classes` (COCO names) are kept; the confidence threshold is low on
    purpose -- the tracker's second association stage uses the weak detections
    to keep tracks alive through partial occlusion.
    """

    def __init__(self, weights, classes=('person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle'),
                 conf=0.1, iou=0.6, imgsz=640, device='auto', threads=4):
        import torch
        from ultralytics import YOLO
        if device == 'auto':
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        if device == 'cpu':
            torch.set_num_threads(threads)
        self.model = YOLO(weights, task='detect')
        self.names = self.model.names
        wanted = set(classes)
        self.class_ids = [i for i, n in self.names.items() if n in wanted]
        self.conf, self.iou, self.imgsz, self.device = conf, iou, imgsz, device

    def __call__(self, bgr):
        r = self.model.predict(bgr, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                               classes=self.class_ids, device=self.device, verbose=False)[0]
        b = r.boxes
        return Detections(b.xyxy.cpu().numpy().astype(np.float32),
                          b.conf.cpu().numpy().astype(np.float32),
                          b.cls.cpu().numpy().astype(np.int32))
