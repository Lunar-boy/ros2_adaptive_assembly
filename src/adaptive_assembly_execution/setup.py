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
    maintainer='nate',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Optional trajectory execution interfaces for assembly.',
    license='TODO: License declaration',
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
            'simulated_follow_joint_trajectory_server_node = '
            'adaptive_assembly_execution.'
            'simulated_follow_joint_trajectory_server_node:main',
        ],
    },
)
