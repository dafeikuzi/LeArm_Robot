import argparse
from pathlib import Path

import cv2


IMAGE_SUFFIXES = {'.bmp', '.jpeg', '.jpg', '.png'}


def find_dataset_root():
    for base in [Path.cwd(), *Path.cwd().parents]:
        candidate = base / 'datasets' / 'learm_objects'
        if candidate.exists():
            return candidate
    return Path.cwd() / 'datasets' / 'learm_objects'


def write_frame(output_dir, prefix, index, frame):
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f'{prefix}_{index:06d}.jpg'
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f'failed to write {path}')
    return path


def open_capture(args):
    backend = cv2.CAP_V4L2 if args.camera_backend == 'v4l2' else cv2.CAP_ANY
    source = int(args.source) if str(args.source).isdigit() else args.source
    capture = cv2.VideoCapture(source, backend)
    if args.pixel_format:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.pixel_format[:4]))
    if args.frame_width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    if args.frame_height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    if args.fps > 0:
        capture.set(cv2.CAP_PROP_FPS, args.fps)
    return capture


def capture_from_stream(args, output_dir):
    capture = open_capture(args)
    if not capture.isOpened():
        raise RuntimeError(f'cannot open source {args.source}')

    saved = 0
    frame_index = 0
    while args.max_frames <= 0 or saved < args.max_frames:
        ok, frame = capture.read()
        if not ok:
            if args.loop:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break

        if frame_index % max(args.every_n, 1) == 0:
            path = write_frame(output_dir, args.prefix, args.start_index + saved, frame)
            print(path)
            saved += 1

        frame_index += 1
        if args.show:
            cv2.imshow('LeArm Frame Capture', frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break

    capture.release()
    cv2.destroyAllWindows()
    print(f'saved {saved} frame(s) to {output_dir}')


def capture_from_images(args, output_dir):
    source = Path(args.source).expanduser()
    if source.is_file():
        images = [source]
    else:
        images = sorted(path for path in source.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)

    saved = 0
    for image_path in images:
        if args.max_frames > 0 and saved >= args.max_frames:
            break
        frame = cv2.imread(str(image_path))
        if frame is None:
            print(f'skip unreadable image: {image_path}')
            continue
        path = write_frame(output_dir, args.prefix, args.start_index + saved, frame)
        print(path)
        saved += 1
    print(f'saved {saved} image(s) to {output_dir}')


def parse_args():
    dataset_root = find_dataset_root()
    parser = argparse.ArgumentParser(description='Capture or normalize images for YOLO training.')
    parser.add_argument('--source', default='/dev/video0', help='image, image directory, video, or camera')
    parser.add_argument('--output', default=str(dataset_root / 'images' / 'raw'))
    parser.add_argument('--prefix', default='learm')
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--every-n', type=int, default=10)
    parser.add_argument('--max-frames', type=int, default=100)
    parser.add_argument('--loop', action='store_true')
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--camera-backend', default='v4l2', choices=['v4l2', 'any'])
    parser.add_argument('--pixel-format', default='MJPG')
    parser.add_argument('--frame-width', type=int, default=320)
    parser.add_argument('--frame-height', type=int, default=240)
    parser.add_argument('--fps', type=int, default=15)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output).expanduser()
    source = Path(str(args.source)).expanduser()
    if source.exists() and (source.is_dir() or source.suffix.lower() in IMAGE_SUFFIXES):
        capture_from_images(args, output_dir)
    else:
        capture_from_stream(args, output_dir)


if __name__ == '__main__':
    main()
