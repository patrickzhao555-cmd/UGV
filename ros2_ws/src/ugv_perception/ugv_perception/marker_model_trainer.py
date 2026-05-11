#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import List

import numpy as np


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}


def _iter_images(image_dir: Path) -> List[Path]:
    if not image_dir.exists():
        raise FileNotFoundError(f'Marker image directory does not exist: {image_dir}')
    return sorted(
        path for path in image_dir.rglob('*')
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def _score_marker_crop_candidate(cv2, gray, contour, source: str):
    h, w = gray.shape[:2]
    image_area = float(max(1, h * w))
    area = float(cv2.contourArea(contour))
    area_frac = area / image_area
    if area_frac < 0.002 or area_frac > 0.70:
        return None
    x, y, bw, bh = cv2.boundingRect(contour)
    if bw <= 0 or bh <= 0:
        return None
    center_y_frac = (y + 0.5 * bh) / max(1.0, float(h))
    if center_y_frac < 0.20:
        return None
    aspect = bw / float(bh)
    if aspect < 0.25 or aspect > 4.5:
        return None
    touches_borders = int(x <= 2) + int(y <= 2) + int(x + bw >= w - 2) + int(y + bh >= h - 2)
    if touches_borders >= 3 and area_frac > 0.15:
        return None

    roi = gray[y:y + bh, x:x + bw]
    if roi.size == 0:
        return None
    p10, p90 = np.percentile(roi, [10.0, 90.0])
    contrast = float(p90 - p10)
    if contrast < 45.0:
        return None
    dark_cut = float(p10 + 0.30 * contrast)
    light_cut = float(p90 - 0.25 * contrast)
    dark_ratio = max(float(np.mean(roi < 85)), float(np.mean(roi <= dark_cut)))
    light_ratio = max(float(np.mean(roi > 170)), float(np.mean(roi >= light_cut)))
    if dark_ratio < 0.10 or light_ratio < 0.03:
        return None

    square_score = max(0.0, 1.0 - min(abs(np.log(max(aspect, 1e-6))), 1.15))
    rect_area = float(max(1, bw * bh))
    extent = area / rect_area
    source_bonus = 0.10 if source == 'edge_region' else 0.0
    score = (
        0.38 * min(1.0, contrast / 150.0)
        + 0.22 * square_score
        + 0.18 * min(1.0, area_frac / 0.08)
        + 0.12 * min(1.0, extent / (0.35 if source == 'edge_region' else 0.60))
        + 0.10 * min(1.0, dark_ratio + light_ratio)
        + source_bonus
    )
    return score, (x, y, bw, bh)


def _find_marker_crop(cv2, gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark_mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    dark_contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    median = float(np.median(blur))
    lower = int(max(20.0, 0.55 * median))
    upper = int(min(255.0, max(lower + 30.0, 1.35 * median)))
    edges = cv2.Canny(blur, lower, upper)
    edge_kernel = np.ones((5, 5), np.uint8)
    edge_mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, edge_kernel, iterations=2)
    edge_mask = cv2.dilate(edge_mask, edge_kernel, iterations=1)
    edge_contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = -1.0
    for source, contours in (('dark_region', dark_contours), ('edge_region', edge_contours)):
        for contour in contours:
            scored = _score_marker_crop_candidate(cv2, gray, contour, source)
            if scored is None:
                continue
            score, bbox = scored
            if score > best_score:
                best_score = score
                best = bbox
    return best


def _cap_descriptors(descriptors: np.ndarray, max_descriptors: int) -> tuple[np.ndarray, int]:
    before_count = int(descriptors.shape[0])
    if max_descriptors <= 0 or before_count <= max_descriptors:
        return descriptors, before_count

    # Deterministic spread through the merged descriptor bank. This keeps older
    # classroom images and newer grass images represented without adding a
    # random seed dependency to runtime behavior.
    indices = np.linspace(0, before_count - 1, num=max_descriptors)
    indices = np.unique(np.round(indices).astype(np.int64))
    if indices.size > max_descriptors:
        indices = indices[:max_descriptors]
    return descriptors[indices], before_count


def train_marker_model(image_dir: Path, model_out: Path, max_features: int, max_descriptors: int) -> dict:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            'OpenCV is required for marker training. Install python3-opencv on the Jetson/Nano.'
        ) from exc

    orb = cv2.ORB_create(nfeatures=max_features)
    descriptors = []
    image_summaries = []

    for image_path in _iter_images(image_dir):
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            image_summaries.append({'file': str(image_path), 'keypoints': 0, 'status': 'unreadable'})
            continue

        crop_bbox = _find_marker_crop(cv2, image)
        train_image = image
        status = 'ok_full_image'
        if crop_bbox is not None:
            x, y, w, h = crop_bbox
            pad = max(8, int(0.08 * max(w, h)))
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(image.shape[1], x + w + pad)
            y1 = min(image.shape[0], y + h + pad)
            train_image = image[y0:y1, x0:x1]
            status = 'ok_marker_crop'

        keypoints, desc = orb.detectAndCompute(train_image, None)
        keypoint_count = 0 if keypoints is None else len(keypoints)
        if desc is not None and len(desc) > 0:
            descriptors.append(desc.astype(np.uint8, copy=False))
        image_summaries.append({
            'file': str(image_path),
            'keypoints': keypoint_count,
            'status': status,
            'crop_bbox': None if crop_bbox is None else list(map(int, crop_bbox)),
        })

    if not descriptors:
        raise RuntimeError(
            f'No ORB descriptors found in {image_dir}. Add sharper marker images from multiple angles/distances.'
        )

    merged = np.vstack(descriptors).astype(np.uint8, copy=False)
    merged, descriptor_count_before_cap = _cap_descriptors(merged, int(max_descriptors))
    metadata = {
        'type': 'ugv_marker_orb_v1',
        'image_dir': str(image_dir),
        'image_count': len(image_summaries),
        'descriptor_count': int(merged.shape[0]),
        'descriptor_count_before_cap': int(descriptor_count_before_cap),
        'descriptor_width': int(merged.shape[1]),
        'max_features': int(max_features),
        'max_descriptors': int(max_descriptors),
        'images': image_summaries,
    }

    model_out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(model_out, descriptors=merged, metadata=json.dumps(metadata))
    return metadata


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Train a lightweight ORB marker model from marker photos.')
    parser.add_argument(
        '--image-dir',
        type=Path,
        default=Path('training/marker_images'),
        help='Folder containing marker photos from different angles/distances.',
    )
    parser.add_argument(
        '--model-out',
        type=Path,
        default=Path('models/marker_orb_model.npz'),
        help='Output .npz model used by marker_vision_node.',
    )
    parser.add_argument('--max-features', type=int, default=900)
    parser.add_argument(
        '--max-descriptors',
        type=int,
        default=65000,
        help='Maximum ORB descriptors stored in the model. Use 0 to keep all descriptors.',
    )
    parsed = parser.parse_args(args=args)

    metadata = train_marker_model(
        parsed.image_dir.expanduser(),
        parsed.model_out.expanduser(),
        parsed.max_features,
        parsed.max_descriptors,
    )
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
