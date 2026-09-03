from glob import glob

from setuptools import find_packages, setup


package_name = 'learm_vla_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhiwei',
    maintainer_email='liuzhiwei@todo.todo',
    description='Local observation bridge for a remotely hosted LeArm VLA policy.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'episode_recorder = learm_vla_bridge.episode_recorder:main',
            'observation_node = learm_vla_bridge.observation_node:main',
            'vla_collection_gui = learm_vla_bridge.collection_gui:main',
        ],
    },
)
