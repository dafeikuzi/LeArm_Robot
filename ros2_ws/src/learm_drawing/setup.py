from glob import glob

from setuptools import find_packages, setup


package_name = 'learm_drawing'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md', 'TRAINING.md']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/trajectories', glob('trajectories/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhiwei',
    maintainer_email='liuzhiwei@todo.todo',
    description='Open-loop drawing trajectory training tools for LeArm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_player = learm_drawing.trajectory_player:main',
        ],
    },
)
