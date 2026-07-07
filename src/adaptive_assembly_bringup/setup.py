from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'adaptive_assembly_bringup'

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
            [path for path in glob('launch/*.launch.py') if os.path.isfile(path)],
        ),
        (
            os.path.join('share', package_name, 'config'),
            [path for path in glob('config/*.yaml') if os.path.isfile(path)],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chunzhi Wu',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Launch entry points for the adaptive assembly pipeline.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
