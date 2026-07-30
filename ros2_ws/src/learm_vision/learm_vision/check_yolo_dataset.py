import argparse
from pathlib import Path


IMAGE_SUFFIXES = {'.bmp', '.jpeg', '.jpg', '.png'}


def find_dataset_root():
    for base in [Path.cwd(), *Path.cwd().parents]:
        candidate = base / 'datasets' / 'learm_objects'
        if candidate.exists():
            return candidate
    return Path.cwd() / 'datasets' / 'learm_objects'


def count_classes(data_yaml):
    if not data_yaml.exists():
        return 0
    count = 0
    in_names = False
    for raw_line in data_yaml.read_text(encoding='utf-8').splitlines():
        line = raw_line.strip()
        if line.startswith('names:'):
            in_names = True
            continue
        if in_names and line and line[0].isdigit() and ':' in line:
            count += 1
        elif in_names and line and not raw_line.startswith(' '):
            break
    return count


def validate_label(label_path, class_count):
    errors = []
    for line_number, line in enumerate(label_path.read_text(encoding='utf-8').splitlines(), 1):
        stripped = line.strip()
        if not stripped:
            continue
        fields = stripped.split()
        if len(fields) != 5:
            errors.append(f'{label_path}:{line_number}: expected 5 fields')
            continue
        try:
            class_id = int(fields[0])
            values = [float(value) for value in fields[1:]]
        except ValueError:
            errors.append(f'{label_path}:{line_number}: invalid numeric value')
            continue
        if class_id < 0 or (class_count and class_id >= class_count):
            errors.append(f'{label_path}:{line_number}: invalid class id {class_id}')
        x_center, y_center, width, height = values
        if not (0.0 <= x_center <= 1.0 and 0.0 <= y_center <= 1.0):
            errors.append(f'{label_path}:{line_number}: center must be normalized 0..1')
        if not (0.0 < width <= 1.0 and 0.0 < height <= 1.0):
            errors.append(f'{label_path}:{line_number}: width/height must be normalized 0..1')
    return errors


def parse_args():
    dataset_root = find_dataset_root()
    parser = argparse.ArgumentParser(description='Check YOLO dataset image/label completeness.')
    parser.add_argument('--dataset-root', default=str(dataset_root))
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_root = Path(args.dataset_root).expanduser()
    data_yaml = dataset_root / 'data.yaml'
    class_count = count_classes(data_yaml)
    errors = []
    total_images = 0
    total_labels = 0

    for split in ('train', 'val'):
        image_dir = dataset_root / 'images' / split
        label_dir = dataset_root / 'labels' / split
        images = sorted(path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
        total_images += len(images)
        for image_path in images:
            label_path = label_dir / f'{image_path.stem}.txt'
            if not label_path.exists():
                errors.append(f'missing label for {image_path}')
                continue
            total_labels += 1
            errors.extend(validate_label(label_path, class_count))

    print(f'dataset: {dataset_root}')
    print(f'classes: {class_count}')
    print(f'images: {total_images}')
    print(f'labels: {total_labels}')
    if errors:
        print('errors:')
        for error in errors:
            print(f'  {error}')
        raise SystemExit(1)
    print('dataset check passed')


if __name__ == '__main__':
    main()
