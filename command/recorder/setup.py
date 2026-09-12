from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'recorder'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuhao Huang',
    maintainer_email='huangyuhaowork@outlook.com',
    description='Input-agnostic ROS 2 bag recorder with a joystick adapter.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = recorder.recorder_node:main',
            'joy_record_trigger = recorder.joy_record_trigger:main',
        ],
    },
)
