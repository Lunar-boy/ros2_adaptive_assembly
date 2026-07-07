from setuptools import find_packages, setup


package_name = 'adaptive_assembly_benchmark'

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
    description='Contact-lite geometric benchmarks for adaptive assembly.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'contact_lite_insertion_evaluator_node = '
        'adaptive_assembly_benchmark.'
        'contact_lite_insertion_evaluator_node:main',
    ]},
)
