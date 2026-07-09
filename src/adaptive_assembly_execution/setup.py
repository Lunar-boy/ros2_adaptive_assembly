"""Set up adaptive assembly trajectory execution interfaces."""

from setuptools import find_packages, setup


package_name = 'adaptive_assembly_execution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chunzhi Wu',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Optional trajectory execution interfaces for assembly.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dry_run_sequence_executor_node = '
            'adaptive_assembly_execution.dry_run_sequence_executor_node:main',
            'ros2_control_sequence_executor_node = '
            'adaptive_assembly_execution.'
            'ros2_control_sequence_executor_node:main',
            'physical_pick_place_executor_node = '
            'adaptive_assembly_execution.'
            'physical_pick_place_executor_node:main',
            'physical_grasp_preflight_node = '
            'adaptive_assembly_execution.'
            'physical_grasp_preflight_node:main',
            'gazebo_grasp_contact_status_node = '
            'adaptive_assembly_execution.'
            'gazebo_grasp_contact_status_node:main',
            'grasp_verifier_node = '
            'adaptive_assembly_execution.grasp_verifier_node:main',
            'simulated_follow_joint_trajectory_server_node = '
            'adaptive_assembly_execution.'
            'simulated_follow_joint_trajectory_server_node:main',
            'wait_for_gazebo_controller_ready_node = '
            'adaptive_assembly_execution.'
            'wait_for_gazebo_controller_ready_node:main',
            'wait_for_gazebo_controller_ready_status_node = '
            'adaptive_assembly_execution.'
            'wait_for_gazebo_controller_ready_status_node:main',
        ],
    },
)
