#!/usr/bin/env python3
"""Convert the processed LeArm episodes to a local LeRobot v3 dataset."""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import re
import tempfile
from pathlib import Path

import cv2
import numpy as np
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets import LeRobotDataset


EPISODE_RE = re.compile(r"episode_(\d+)$")
EXPECTED_FRAME_SHAPE = (480, 640, 3)
JOINT_COUNT = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("datasets/learm_vla/processed_v1"),
        help="Processed episode directory.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("datasets/learm_vla/lerobot_v3"),
        help="Destination LeRobot dataset directory.",
    )
    parser.add_argument("--fps", type=int, default=8, help="Dataset FPS used for LeRobot timestamps.")
    parser.add_argument(
        "--keep-staging-on-error",
        action="store_true",
        help="Keep the temporary output directory if conversion fails.",
    )
    return parser.parse_args()


def episode_dirs(source: Path) -> list[Path]:
    episodes = []
    for path in source.iterdir():
        match = EPISODE_RE.fullmatch(path.name)
        if match and path.is_dir():
            episodes.append((int(match.group(1)), path))
    episodes.sort(key=lambda item: item[0])
    expected = list(range(1, len(episodes) + 1))
    actual = [index for index, _ in episodes]
    if actual != expected:
        raise ValueError(f"Episode numbering must be contiguous from 1; found {actual[:5]}...{actual[-5:]}")
    return [path for _, path in episodes]


def load_records(episode_dir: Path) -> tuple[dict, list[dict]]:
    metadata_path = episode_dir / "metadata.json"
    records_path = episode_dir / "records.jsonl"
    with metadata_path.open(encoding="utf-8") as handle:
        metadata = json.load(handle)
    records = []
    with records_path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON in {records_path}:{line_number}: {exc}") from exc
    if not records:
        raise ValueError(f"No records found in {records_path}")
    if metadata.get("sample_count") != len(records):
        raise ValueError(
            f"{episode_dir.name}: metadata sample_count={metadata.get('sample_count')} "
            f"but records.jsonl has {len(records)} rows"
        )
    return metadata, records


def vector(record: dict, key: str) -> np.ndarray:
    state = record["observation"][key]
    joints = state["joint_positions_rad"]
    gripper = state["gripper_opening"]
    if len(joints) != JOINT_COUNT:
        raise ValueError(f"Expected {JOINT_COUNT} joints, got {len(joints)}")
    return np.asarray([*joints, gripper], dtype=np.float32)


def build_features() -> dict[str, dict]:
    return {
        "observation.images.camera": {
            "dtype": "video",
            "shape": EXPECTED_FRAME_SHAPE,
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (JOINT_COUNT + 1,),
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
        },
        "action": {
            "dtype": "float32",
            "shape": (JOINT_COUNT + 1,),
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
        },
    }


def block_streaming_encoder(dataset: LeRobotDataset) -> None:
    """Make streaming writes lossless when conversion is faster than encoding.

    LeRobot's recorder intentionally drops frames when a live camera outruns the
    encoder. A file conversion must preserve every source frame, so replace that
    live-recording policy with a blocking queue put.
    """
    encoder = dataset.writer._streaming_encoder
    if encoder is None:
        raise RuntimeError("Streaming encoder was not initialized")

    def feed_frame(video_key: str, image: np.ndarray) -> None:
        thread = encoder._threads[video_key]
        frame_queue = encoder._frame_queues[video_key]
        while True:
            if not thread.is_alive():
                try:
                    status, message = encoder._result_queues[video_key].get_nowait()
                except queue.Empty:
                    status, message = "error", "encoder thread stopped unexpectedly"
                raise RuntimeError(f"Encoder thread failed: {status}: {message}")
            try:
                frame_queue.put(image.copy(), timeout=1.0)
                return
            except queue.Full:
                continue

    encoder.feed_frame = feed_frame


def convert(source: Path, output: Path, fps: int, keep_staging_on_error: bool) -> None:
    source = source.resolve()
    output = output.resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"Source directory does not exist: {source}")
    if output.exists():
        raise FileExistsError(f"Output already exists; refusing to overwrite: {output}")

    episodes = episode_dirs(source)
    if len(episodes) != 150:
        raise ValueError(f"Expected 150 episodes, found {len(episodes)}")

    output.parent.mkdir(parents=True, exist_ok=True)
    staging_parent = Path(tempfile.mkdtemp(prefix="lerobot_v3.", dir=output.parent))
    staging = staging_parent / "dataset"
    dataset: LeRobotDataset | None = None
    total_frames = 0
    task_names: set[str] = set()
    try:
        # H.264 is broadly decodable and keeps CPU conversion practical in the VM.
        encoder = RGBEncoderConfig(vcodec="h264", crf=18, preset="ultrafast", g=12)
        dataset = LeRobotDataset.create(
            repo_id="local/learm_vla",
            fps=fps,
            root=staging,
            robot_type="learm",
            features=build_features(),
            use_videos=True,
            image_writer_processes=0,
            image_writer_threads=0,
            rgb_encoder=encoder,
            encoder_threads=4,
            streaming_encoding=True,
            encoder_queue_maxsize=128,
        )
        block_streaming_encoder(dataset)

        for episode_number, episode_dir in enumerate(episodes, 1):
            metadata, records = load_records(episode_dir)
            episode_task = metadata.get("task")
            if not isinstance(episode_task, str) or not episode_task:
                raise ValueError(f"{episode_dir.name}: missing metadata.task")
            task_names.add(episode_task)

            for expected_index, record in enumerate(records):
                if record.get("frame_index") != expected_index:
                    raise ValueError(f"{episode_dir.name}: non-sequential frame_index at row {expected_index}")
                task = record.get("task", episode_task)
                if task != episode_task:
                    raise ValueError(f"{episode_dir.name}: task changed within episode")
                image_path = episode_dir / record["image"]
                image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                if image_bgr is None:
                    raise ValueError(f"Could not decode image: {image_path}")
                image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                if image_rgb.shape != EXPECTED_FRAME_SHAPE:
                    raise ValueError(
                        f"{image_path}: expected {EXPECTED_FRAME_SHAPE}, got {image_rgb.shape}"
                    )
                dataset.add_frame(
                    {
                        "observation.images.camera": image_rgb,
                        "observation.state": vector(record, "state"),
                        "action": vector(record, "target_state"),
                        "task": task,
                    }
                )
                total_frames += 1

            dataset.save_episode(parallel_encoding=False)
            logging.info(
                "Converted %03d/150 (%s, %d frames)", episode_number, episode_dir.name, len(records)
            )

        dataset.finalize()
        if len(task_names) != 3:
            raise ValueError(f"Expected 3 task labels, found {sorted(task_names)}")
        os.replace(staging, output)
        staging_parent.rmdir()
        logging.info("Converted %d frames across %d episodes", total_frames, len(episodes))
    except Exception:
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception:
                logging.exception("Failed to finalize partial dataset")
        if not keep_staging_on_error:
            # The staging path is uniquely generated by this process.
            import shutil

            shutil.rmtree(staging_parent, ignore_errors=True)
        raise


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    convert(args.source, args.output, args.fps, args.keep_staging_on_error)


if __name__ == "__main__":
    main()
