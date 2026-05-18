"""
yolo_inferencer.py
──────────────────
YOLOv8 instance-segmentation wrapper.

Detects discrete objects (person, car, bicycle, …) and returns a uint8 mask
where each pixel that belongs to a detected object carries the corresponding
SemanticClass ID.  Background pixels remain 0 (UNKNOWN).

Backends
────────
• PyTorch  — default; works on RTX desktop and Jetson Orin out of the box.
• TensorRT — load a pre-exported .engine file for maximum throughput on
             Jetson.  Export once with scripts/export_tensorrt.py.

COCO → SemanticClass mapping (configurable in YAML)
────────────────────────────────────────────────────
COCO class IDs used by the default YOLOv8 weights trained on COCO-2017:
  0  person        → 7  PEDESTRIAN
  1  bicycle       → 9  BICYCLE
  2  car           → 8  VEHICLE
  3  motorcycle    → 9  BICYCLE
  5  bus           → 8  VEHICLE
  7  truck         → 8  VEHICLE
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch

log = logging.getLogger(__name__)

# Default COCO-id → SemanticClass-id pairs (flat list, same format as YAML)
_DEFAULT_MAP_PAIRS = [
    (0, 7),   # person      → PEDESTRIAN
    (1, 9),   # bicycle     → BICYCLE
    (2, 8),   # car         → VEHICLE
    (3, 9),   # motorcycle  → BICYCLE
    (5, 8),   # bus         → VEHICLE
    (7, 8),   # truck       → VEHICLE
]


class YOLOInferencer:
    """
    Thin wrapper around Ultralytics YOLO that returns a semantic class mask.

    Parameters
    ----------
    model_path : str
        Path to a .pt (PyTorch) or .engine (TensorRT) YOLO model.
        Ultralytics will download the default weights on first run if a bare
        model name such as 'yolov8n-seg.pt' is given.
    device : str
        'cuda:0', 'cuda', or 'cpu'.
    conf_threshold : float
        Minimum detection confidence to include a mask.
    class_map_pairs : list[tuple[int,int]]
        [(coco_id, semantic_id), …] remapping table.
    half : bool
        Use FP16 inference.  Recommended for Jetson Orin.
    img_size : int
        Inference resolution (square).  Lower = faster; 640 is the YOLO default.
    """

    def __init__(self,
                 model_path: str = 'yolov8n-seg.pt',
                 device: str = 'cuda:0',
                 conf_threshold: float = 0.35,
                 class_map_pairs: list[tuple[int, int]] | None = None,
                 half: bool = False,
                 img_size: int = 640) -> None:

        from ultralytics import YOLO  # imported here so the package is optional

        self._conf   = conf_threshold
        self._half   = half
        self._device = device
        self._size   = img_size

        log.info(f'Loading YOLO model: {model_path}  device={device}  '
                 f'half={half}  size={img_size}')
        self._model = YOLO(model_path)

        # Build 256-entry LUT: coco_id → semantic_id
        pairs = class_map_pairs or _DEFAULT_MAP_PAIRS
        self._lut = np.zeros(256, dtype=np.uint8)
        for coco_id, sem_id in pairs:
            if 0 <= coco_id < 256:
                self._lut[coco_id] = sem_id

        log.info(f'YOLO ready | mapped classes: '
                 f'{[(c, s) for c, s in pairs if s > 0]}')

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        """
        Run instance segmentation and return a uint8 semantic mask (H, W).

        Pixels belonging to a detected object carry the SemanticClass ID.
        Undetected background pixels are 0 (UNKNOWN).

        Parameters
        ----------
        bgr_image : np.ndarray  shape (H, W, 3), dtype uint8, BGR

        Returns
        -------
        mask : np.ndarray  shape (H, W), dtype uint8
        """
        H, W = bgr_image.shape[:2]
        mask = np.zeros((H, W), dtype=np.uint8)

        results = self._model(
            bgr_image,
            imgsz=self._size,
            conf=self._conf,
            half=self._half,
            device=self._device,
            verbose=False,
        )

        result = results[0]

        # No detections or model has no masks (detection-only model)
        if result.masks is None or len(result.masks) == 0:
            return mask

        # masks.data : (N, H_mask, W_mask) — binary masks, values 0/1
        raw_masks = result.masks.data.cpu().numpy()          # (N, Hm, Wm)
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

        for i, (bin_mask, coco_id) in enumerate(zip(raw_masks, class_ids)):
            sem_id = int(self._lut[min(coco_id, 255)])
            if sem_id == 0:
                continue   # class not mapped → skip

            # Resize binary mask to original image size
            bin_mask_full = cv2.resize(
                bin_mask.astype(np.uint8),
                (W, H),
                interpolation=cv2.INTER_NEAREST,
            )
            # Objects painted in priority order (highest class-priority last
            # is fine here; semantic_bev fusion handles priority globally)
            mask[bin_mask_full > 0] = sem_id

        return mask

    @property
    def device(self) -> str:
        return self._device
