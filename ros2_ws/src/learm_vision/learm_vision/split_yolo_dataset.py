import argparse
import random
import shutil
from pathlib import Path


IMAGE_SUFFIXES = {'.bmp', '.jpeg', '.jpg', '.png'}


def find_dataset_root():
    for base in [Path.cwd(), *Path.cwd().parents]:
        candidate = base / 'datasets' / 'learm_objects'
        if candidate.exists():
            return candidate
    return Path.cwd() / 'datasets' / 'learm_objects'


def copy_or_move(source, destination, move):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if move:
        shutil.move(str(source), str(destination))
    else:
        shutil.copy2(source, destination)


def parse_args():
    dataset_root = find_dataset_root()
    parser = argparse.ArgumentParser(description='Split raw YOLO images and labels into train/val.')
    parser.add_argument('--dataset-root', default=str(dataset_root))
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--move', action='store_true')
    parser.add_argument('--create-empty-labels', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_root = Path(args.dataset_root).expanduser()
    image_raw = dataset_root / 'images' / 'raw'
    label_raw = dataset_root / 'labels' / 'raw'
    images = sorted(path for path in image_raw.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise SystemExit(f'no images found in {image_raw}')

    rng = random.Random(args.seed)
    rng.shuffle(images)
    val_count = max(1, int(round(len(images) * args.val_ratio))) if len(images) > 1 else 0
    val_stems = {path.stem for path in images[:val_count]}

    missing_labels = 0
    for image_path in images:
        split = 'val' if image_path.stem in val_stems else 'train'
        target_image = dataset_root / 'images' / split / image_path.name
        copy_or_move(image_path, target_image, args.move)

        label_path = label_raw / f'{image_path.stem}.txt'
        target_label = dataset_root / 'labels' / split / f'{image_path.stem}.txt'
        if label_path.exists():
            copy_or_move(label_path, target_label, args.move)
        elif args.create_empty_labels:
            target_label.parent.mkdir(parents=True, exist_ok=True)
            target_label.write_text('', encoding='utf-8')
        else:
            missing_labels += 1

    print(f'train images: {len(images) - val_count}')
    print(f'val images: {val_count}')
    if missing_labels:
        print(f'warning: {missing_labels} image(s) had no matching raw label file')


if __name__ == '__main__':
    main()
