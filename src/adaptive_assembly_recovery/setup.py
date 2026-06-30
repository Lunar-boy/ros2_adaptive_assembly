"""Set up the adaptive assembly recovery supervisor package."""

from setuptools import find_packages, setup


package_name = 'adaptive_assembly_recovery'

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
    description='Closed-loop recovery supervision for adaptive assembly.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'recovery_supervisor_node = '
            'adaptive_assembly_recovery.recovery_supervisor_node:main',
        ],
    },
)
