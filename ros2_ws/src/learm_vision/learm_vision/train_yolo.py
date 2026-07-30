import argparse
import tempfile
from pathlib import Path


def find_dataset_root():
    for base in [Path.cwd(), *Path.cwd().parents]:
        candidate = base / 'datasets' / 'learm_objects'
        if candidate.exists():
            return candidate
    return Path.cwd() / 'datasets' / 'learm_objects'


def make_resolved_data_yaml(data_yaml):
    data_yaml = Path(data_yaml).expanduser().resolve()
    dataset_root = data_yaml.parent
    lines = data_yaml.read_text(encoding='utf-8').splitlines()
    replaced = False
    output = []
    for line in lines:
        if line.strip().startswith('path:'):
            output.append(f'path: {dataset_root.as_posix()}')
            replaced = True
        else:
            output.append(line)
    if not replaced:
        output.insert(0, f'path: {dataset_root.as_posix()}')

    temp_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='learm_yolo_', encoding='utf-8', delete=False)
    temp_file.write('\n'.join(output) + '\n')
    temp_file.close()
    return Path(temp_file.name)


def parse_args():
    dataset_root = find_dataset_root()
    parser = argparse.ArgumentParser(description='Train a YOLO nano detector for LeArm objects.')
    parser.add_argument('--data', default=str(dataset_root / 'data.yaml'))
    parser.add_argument('--model', default='yolov8n.pt')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--project', default='runs/detect')
    parser.add_argument('--name', default='learm_objects')
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            'Missing dependency: ultralytics. Install with: '
            'python3 -m pip install --user ultralytics') from exc

    data_yaml = make_resolved_data_yaml(args.data)
    model = YOLO(args.model)
    result = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
    )
    print(result)


if __name__ == '__main__':
    main()
