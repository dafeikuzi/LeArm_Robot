#!/usr/bin/env python3
"""Create a visually enhanced copy of the raw LeArm VLA dataset."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import cv2
import numpy as np


JPEG_QUALITY = 95
WHITE_BALANCE_MIN_GAIN = 0.90
WHITE_BALANCE_MAX_GAIN = 1.10
CLAHE_CLIP_LIMIT = 1.5
CLAHE_TILE_GRID = (8, 8)
CONTRAST_ALPHA = 1.05
BRIGHTNESS_BETA = 2
GAMMA = 0.98
SATURATION_SCALE = 1.15


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("datasets/learm_vla/raw"),
        help="raw dataset root",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("datasets/learm_vla/processed_v1"),
        help="processed dataset root (must not already exist)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=max(1, min(8, os.cpu_count() or 1)),
        help="number of image processing threads",
    )
    return parser.parse_args()


def gamma_lut(gamma: float) -> np.ndarray:
    values = np.arange(256, dtype=np.float32) / 255.0
    return np.clip((values**gamma) * 255.0, 0, 255).astype(np.uint8)


def enhance(image: np.ndarray, lut: np.ndarray) -> np.ndarray:
    channel_means = image.mean(axis=(0, 1))
    target_mean = float(channel_means.mean())
    gains = np.clip(
        target_mean / np.maximum(channel_means, 1.0),
        WHITE_BALANCE_MIN_GAIN,
        WHITE_BALANCE_MAX_GAIN,
    )
    balanced = np.clip(image.astype(np.float32) * gains, 0, 255).astype(np.uint8)

    lab = cv2.cvtColor(balanced, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(
        clipLimit=CLAHE_CLIP_LIMIT,
        tileGridSize=CLAHE_TILE_GRID,
    )
    l_channel = clahe.apply(l_channel)
    contrast_lab = cv2.cvtColor(
        cv2.merge((l_channel, a_channel, b_channel)),
        cv2.COLOR_LAB2BGR,
    )

    adjusted = cv2.convertScaleAbs(
        contrast_lab,
        alpha=CONTRAST_ALPHA,
        beta=BRIGHTNESS_BETA,
    )
    corrected = cv2.LUT(adjusted, lut)

    denoised = cv2.bilateralFilter(
        corrected,
        d=5,
        sigmaColor=18,
        sigmaSpace=18,
    )
    blurred = cv2.GaussianBlur(denoised, (0, 0), 1.0)
    sharpened = cv2.addWeighted(denoised, 1.12, blurred, -0.12, 0)

    hsv = cv2.cvtColor(sharpened, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * SATURATION_SCALE, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def process_one(source: Path, destination: Path, lut: np.ndarray) -> None:
    image = cv2.imread(str(source), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot decode image: {source}")

    enhanced = enhance(image, lut)
    if enhanced.shape != image.shape:
        raise RuntimeError(f"unexpected output shape for {source}: {enhanced.shape}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(
        str(destination),
        enhanced,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    ):
        raise RuntimeError(f"cannot write image: {destination}")


def validate_input(input_root: Path) -> list[Path]:
    if not input_root.is_dir():
        raise RuntimeError(f"input directory does not exist: {input_root}")

    episodes = sorted(
        path for path in input_root.iterdir()
        if path.is_dir() and path.name.startswith("episode_")
    )
    if not episodes:
        raise RuntimeError(f"no episode directories found in {input_root}")
    for episode in episodes:
        for filename in ("metadata.json", "records.jsonl"):
            if not (episode / filename).is_file():
                raise RuntimeError(f"missing {filename}: {episode}")
        if not (episode / "frames").is_dir():
            raise RuntimeError(f"missing frames directory: {episode}")
    return episodes


def main() -> int:
    args = parse_args()
    input_root = args.input.resolve()
    output_root = args.output.resolve()
    temporary_root = output_root.with_name(f"{output_root.name}.tmp-{os.getpid()}")
    if output_root.exists():
        raise RuntimeError(f"output already exists; refusing to overwrite: {output_root}")
    if temporary_root.exists():
        raise RuntimeError(f"temporary output already exists: {temporary_root}")

    episodes = validate_input(input_root)
    jobs: list[tuple[Path, Path]] = []
    for episode in episodes:
        destination_episode = temporary_root / episode.name
        destination_episode.mkdir(parents=True, exist_ok=True)
        shutil.copy2(episode / "metadata.json", destination_episode / "metadata.json")
        shutil.copy2(episode / "records.jsonl", destination_episode / "records.jsonl")
        for source in sorted((episode / "frames").glob("*.jpg")):
            jobs.append((source, destination_episode / "frames" / source.name))
    if not jobs:
        raise RuntimeError("no JPG frames found")

    lut = gamma_lut(GAMMA)
    cv2.setNumThreads(1)
    completed = 0
    print(
        f"processing {len(jobs)} images from {len(episodes)} episodes "
        f"with {args.workers} workers",
        flush=True,
    )
    try:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(process_one, source, destination, lut): source
                for source, destination in jobs
            }
            for future in as_completed(futures):
                source = futures[future]
                future.result()
                completed += 1
                if completed % 500 == 0 or completed == len(jobs):
                    print(f"processed {completed}/{len(jobs)}: {source}", flush=True)
    except Exception:
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise

    temporary_root.rename(output_root)
    print(f"created {output_root}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
