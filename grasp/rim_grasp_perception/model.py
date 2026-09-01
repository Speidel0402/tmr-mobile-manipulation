from __future__ import annotations

from pathlib import Path
from typing import List
import os

import numpy as np
from PIL import Image

from .config import ModelConfig
from .types import Detection


class GroundedSAM2:
    """Local-only Grounding DINO boxes followed by SAM 2 masks."""

    def __init__(self, config: ModelConfig):
        cache = str(Path(config.model_cache).expanduser().resolve())
        os.environ.setdefault("HF_HOME", cache)
        if config.local_files_only:
            os.environ["HF_HUB_OFFLINE"] = "1"
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self.torch = torch
        self.cfg = config
        device = config.device if config.device == "cpu" or torch.cuda.is_available() else "cpu"
        self.device = device
        self.processor = AutoProcessor.from_pretrained(
            config.grounding_model, cache_dir=cache, local_files_only=config.local_files_only)
        self.detector = AutoModelForZeroShotObjectDetection.from_pretrained(
            config.grounding_model, cache_dir=cache, local_files_only=config.local_files_only).to(device).eval()
        self.sam = SAM2ImagePredictor.from_pretrained(config.sam2_model, device=device)

    def infer(self, rgb: np.ndarray) -> List[Detection]:
        image = Image.fromarray(rgb)
        prompt = ". ".join(self.cfg.categories) + "."
        inputs = self.processor(images=image, text=prompt, return_tensors="pt").to(self.device)
        with self.torch.inference_mode():
            outputs = self.detector(**inputs)
        target = self.torch.tensor([[rgb.shape[0], rgb.shape[1]]], device=self.device)
        kwargs = dict(box_threshold=self.cfg.box_threshold, text_threshold=self.cfg.text_threshold,
                      target_sizes=target)
        result = self.processor.post_process_grounded_object_detection(outputs, inputs.input_ids, **kwargs)[0]
        boxes = result["boxes"].detach().cpu().numpy()
        scores = result["scores"].detach().cpu().numpy()
        labels = result.get("text_labels", result.get("labels", []))
        labels = [str(x) for x in labels]
        if len(boxes) == 0:
            return []
        self.sam.set_image(rgb)
        detections: List[Detection] = []
        for box, score, label in zip(boxes, scores, labels):
            masks, mask_scores, _ = self.sam.predict(box=box, multimask_output=True)
            idx = int(np.argmax(mask_scores))
            mask = masks[idx].astype(bool)
            category = next((c for c in self.cfg.categories if c in label.lower()), label.lower())
            if category in self.cfg.categories and int(mask.sum()) >= self.cfg.min_mask_pixels:
                detections.append(Detection(category, float(score), box.astype(float), mask))
        return detections
