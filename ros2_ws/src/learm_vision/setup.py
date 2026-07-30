from glob import glob

from setuptools import find_packages, setup


package_name = 'learm_vision'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md', 'requirements.txt']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhiwei',
    maintainer_email='liuzhiwei@todo.todo',
    description='OpenCV camera display and simple object detection for LeArm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'capture_frames = learm_vision.capture_frames:main',
            'check_yolo_dataset = learm_vision.check_yolo_dataset:main',
            'color_tracker = learm_vision.color_tracker:main',
            'model_detector = learm_vision.model_detector:main',
            'split_yolo_dataset = learm_vision.split_yolo_dataset:main',
            'train_yolo = learm_vision.train_yolo:main',
        ],
    },
)
