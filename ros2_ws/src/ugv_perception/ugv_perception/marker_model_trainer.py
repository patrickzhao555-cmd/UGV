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


def train_marker_model(image_dir: Path, model_out: Path, max_features: int) -> dict:
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

        keypoints, desc = orb.detectAndCompute(image, None)
        keypoint_count = 0 if keypoints is None else len(keypoints)
        if desc is not None and len(desc) > 0:
            descriptors.append(desc.astype(np.uint8, copy=False))
        image_summaries.append({'file': str(image_path), 'keypoints': keypoint_count, 'status': 'ok'})

    if not descriptors:
        raise RuntimeError(
            f'No ORB descriptors found in {image_dir}. Add sharper marker images from multiple angles/distances.'
        )

    merged = np.vstack(descriptors).astype(np.uint8, copy=False)
    metadata = {
        'type': 'ugv_marker_orb_v1',
        'image_dir': str(image_dir),
        'image_count': len(image_summaries),
        'descriptor_count': int(merged.shape[0]),
        'descriptor_width': int(merged.shape[1]),
        'max_features': int(max_features),
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
    parsed = parser.parse_args(args=args)

    metadata = train_marker_model(parsed.image_dir.expanduser(), parsed.model_out.expanduser(), parsed.max_features)
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
