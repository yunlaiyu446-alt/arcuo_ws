import os
from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = 'd435_python_camera'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/d435_python_camera'],
        ),
        (f'share/{PACKAGE_NAME}', ['package.xml']),
        (
            os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', PACKAGE_NAME, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuyunlai',
    maintainer_email='yuyunlai@example.com',
    description=(
        'Publish a single RealSense D435/D435i color stream with pyrealsense2.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'd435_python_camera_node = d435_python_camera.camera_node:main',
        ],
    },
)
