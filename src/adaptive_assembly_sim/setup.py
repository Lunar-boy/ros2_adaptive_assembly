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
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro') + glob('urdf/*.urdf'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chunzhi Wu',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Gazebo workcell assets and launch files for adaptive assembly.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gazebo_target_pose_sync_node = '
            'adaptive_assembly_sim.gazebo_target_pose_sync_node:main',
            'gazebo_entity_pose_observer_node = '
            'adaptive_assembly_sim.gazebo_entity_pose_observer_node:main',
            'activate_gazebo_controllers_node = '
            'adaptive_assembly_sim.activate_gazebo_controllers_node:main',
            'fake_panda_finger_joint_state_node = adaptive_assembly_sim.fake_panda_finger_joint_state_node:main',
        ],
    },
)
