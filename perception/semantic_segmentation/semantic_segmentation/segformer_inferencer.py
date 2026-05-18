"""
segformer_inferencer.py
───────────────────────
SegFormer semantic-segmentation wrapper.

Assigns a surface / scene class to EVERY pixel in the image.
Provides the classes that YOLO cannot: road, sidewalk, crosswalk, grass, curb.

Pretrained model
────────────────
Default: nvidia/segformer-b0-finetuned-cityscapes-512-1024
  • Cityscapes dataset: 19 classes (road, sidewalk, vegetation, sky, …)
  • b0 = smallest / fastest  (≈ 30 ms on RTX, ≈ 80 ms on Jetson Orin)
  • b2 = better accuracy     (≈ 70 ms on RTX, ≈ 150 ms on Jetson Orin)

Alternative for even faster inference on Jetson: export to ONNX/TensorRT
via scripts/export_tensorrt.py (provides 2–4× speedup).

Cityscapes id → SemanticClass id mapping
─────────────────────────────────────────
  0  road         → 4  ROAD
  1  sidewalk     → 3  SIDEWALK
  2  building     → 0  (ignored)
  3  wall         → 2  OBSTACLE
  4  fence        → 2  OBSTACLE
  5  pole         → 0  (ignored)
  6  traffic light→ 0  (ignored)
  7  traffic sign → 0  (ignored)
  8  vegetation   → 6  GRASS
  9  terrain      → 6  GRASS
  10 sky          → 0  (ignored)
  11 person       → 7  PEDESTRIAN  (YOLO will override with better mask)
  12 rider        → 7  PEDESTRIAN
  13 car          → 8  VEHICLE     (YOLO will override)
  14 truck        → 8  VEHICLE
  15 bus          → 8  VEHICLE
  16 train        → 8  VEHICLE
  17 motorcycle   → 9  BICYCLE
  18 bicycle      → 9  BICYCLE

Note: 'crosswalk' is not a standard Cityscapes class.  Use a model
trained on BDD100K or a custom dataset if crosswalk detection is needed.
"""

from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np
import torch

log = logging.getLogger(__name__)

# Cityscapes (19 classes) → SemanticClass ID
_CITYSCAPES_TO_SEMANTIC = np.array([
    4,   # 0  road         → ROAD
    3,   # 1  sidewalk     → SIDEWALK
    0,   # 2  building     → UNKNOWN
    2,   # 3  wall         → OBSTACLE
    2,   # 4  fence        → OBSTACLE
    0,   # 5  pole         → UNKNOWN
    0,   # 6  traffic light→ UNKNOWN
    0,   # 7  traffic sign → UNKNOWN
    6,   # 8  vegetation   → GRASS
    6,   # 9  terrain      → GRASS
    0,   # 10 sky          → UNKNOWN
    7,   # 11 person       → PEDESTRIAN
    7,   # 12 rider        → PEDESTRIAN
    8,   # 13 car          → VEHICLE
    8,   # 14 truck        → VEHICLE
    8,   # 15 bus          → VEHICLE
    8,   # 16 train        → VEHICLE
    9,   # 17 motorcycle   → BICYCLE
    9,   # 18 bicycle      → BICYCLE
], dtype=np.uint8)


class SegFormerInferencer:
    """
    HuggingFace SegFormer wrapper that returns a SemanticClass uint8 mask.

    Parameters
    ----------
    model_name : str
        HuggingFace model ID or local path.
        First run downloads weights to ~/.cache/huggingface/hub/.
    device : str
        'cuda:0', 'cuda', or 'cpu'.
    half : bool
        FP16 inference.  Cuts memory and latency roughly in half on Ampere
        GPUs (RTX 30/40 series, Jetson Orin).  Slightly lower accuracy.
    infer_width : int
        Image width fed to the model.  Model was trained at 1024; lower
        values (e.g. 512) are faster with modest accuracy loss.
    infer_height : int
        Image height fed to the model.  Trained at 512.
    cityscapes_lut : np.ndarray | None
        19-entry uint8 array mapping Cityscapes label → SemanticClass.
        Pass None to use the built-in table above.
    """

    def __init__(self,
                 model_name: str = 'nvidia/segformer-b0-finetuned-cityscapes-512-1024',
                 device: str = 'cuda:0',
                 half: bool = False,
                 infer_width: int = 1024,
                 infer_height: int = 512,
                 cityscapes_lut: Optional[np.ndarray] = None) -> None:

        # HuggingFace imports deferred so the package is optional at import time
        from transformers import (SegformerImageProcessor,
                                   SegformerForSemanticSegmentation)

        self._device = torch.device(device)
        self._half   = half
        self._iw     = infer_width
        self._ih     = infer_height
        self._lut    = cityscapes_lut if cityscapes_lut is not None \
                        else _CITYSCAPES_TO_SEMANTIC

        log.info(f'Loading SegFormer: {model_name}  device={device}  '
                 f'half={half}  size={infer_width}×{infer_height}')

        self._processor = SegformerImageProcessor.from_pretrained(model_name)
        self._model     = SegformerForSemanticSegmentation.from_pretrained(
            model_name)
        self._model.eval()
        self._model.to(self._device)

        if half:
            # FP16 only on CUDA; stays FP32 on CPU
            if self._device.type == 'cuda':
                self._model = self._model.half()
            else:
                log.warning('half=True ignored: device is CPU')

        log.info('SegFormer ready')

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        """
        Run semantic segmentation and return a uint8 SemanticClass mask (H, W).

        The output is upsampled back to the original image resolution using
        nearest-neighbour interpolation to preserve crisp class boundaries.

        Parameters
        ----------
        bgr_image : np.ndarray  shape (H, W, 3), dtype uint8, BGR

        Returns
        -------
        mask : np.ndarray  shape (H, W), dtype uint8
        """
        H_orig, W_orig = bgr_image.shape[:2]

        # ── Pre-process ───────────────────────────────────────────────────────
        # Resize to inference resolution (keep aspect ratio optional, here we
        # stretch to match the training resolution exactly)
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        if rgb.shape[:2] != (self._ih, self._iw):
            rgb = cv2.resize(rgb, (self._iw, self._ih),
                             interpolation=cv2.INTER_LINEAR)

        # SegformerImageProcessor normalises and converts to tensor
        inputs = self._processor(images=rgb, return_tensors='pt')
        pixel_values = inputs['pixel_values'].to(self._device)
        if self._half and self._device.type == 'cuda':
            pixel_values = pixel_values.half()

        # ── Inference ─────────────────────────────────────────────────────────
        with torch.no_grad():
            outputs = self._model(pixel_values=pixel_values)

        # logits: (1, num_labels, H/4, W/4) — SegFormer stride-4 output
        logits = outputs.logits  # shape (1, 19, Hm, Wm)

        # ── Post-process ──────────────────────────────────────────────────────
        # argmax → predicted class per pixel at model resolution
        pred = logits.argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        # pred shape: (Hm, Wm), values in [0, 18]

        # Upsample to original image resolution
        pred_full = cv2.resize(pred, (W_orig, H_orig),
                               interpolation=cv2.INTER_NEAREST)

        # Remap Cityscapes IDs → SemanticClass IDs using LUT
        semantic_mask = self._lut[pred_full]   # vectorised numpy lookup
        return semantic_mask                    # shape (H, W), uint8

    @property
    def device(self) -> str:
        return str(self._device)
