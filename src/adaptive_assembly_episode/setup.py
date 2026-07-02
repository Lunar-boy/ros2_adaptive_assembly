"""Set up the passive adaptive assembly episode supervisor package."""

from setuptools import find_packages, setup


package_name = 'adaptive_assembly_episode'

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
        (
            'share/' + package_name + '/launch',
            ['launch/assembly_episode_supervisor.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='chunzhi.wu@mailbox.tu-dresden.de',
    description='Passive simulator-only assembly episode aggregation.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'assembly_episode_supervisor_node = '
            'adaptive_assembly_episode.assembly_episode_supervisor_node:main',
        ],
    },
)
