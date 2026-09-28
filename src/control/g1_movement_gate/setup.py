from setuptools import find_packages, setup

package_name = 'g1_movement_gate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fk',
    maintainer_email='carlos_hxk@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'g1_movement_gate_node = g1_movement_gate.g1_movement_gate:main',
        ],
    },
)
