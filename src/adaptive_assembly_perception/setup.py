from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'adaptive_assembly_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_object_pose_node = '
            'adaptive_assembly_perception.fake_object_pose_node:main',
            'simulated_marker_pose_node = '
            'adaptive_assembly_perception.simulated_marker_pose_node:main',
            'aruco_detector_node = '
            'adaptive_assembly_perception.aruco_detector_node:main',
        ],
    },
)
