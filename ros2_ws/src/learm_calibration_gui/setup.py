from setuptools import find_packages, setup


package_name = 'learm_calibration_gui'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhiwei',
    maintainer_email='liuzhiwei@todo.todo',
    description='PyQt5 calibration panel for the LeArm ROS 2 driver.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'learm_calibration_gui = learm_calibration_gui.main:main',
        ],
    },
)
