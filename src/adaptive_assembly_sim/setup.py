from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'adaptive_assembly_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Gazebo workcell assets and launch files for adaptive assembly.',
    license='TODO: License declaration',
    entry_points={'console_scripts': []},
)
