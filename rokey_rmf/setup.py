from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rokey_rmf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/' + package_name + '/config', [
        'config/test.building.yaml',
        'config/turtlebot4_config.yaml',
        'config/P1.png',
        'config/P2.png',
    ]),
    ('share/' + package_name + '/config/nav_graphs', [
        'config/nav_graphs/0.yaml',
    ]),
    ('share/' + package_name + '/dashboard', ['dashboard/dashboard.json']),
    ('share/' + package_name + '/launch', [
        'launch/rmf.launch.xml',
        'launch/test.launch.xml',
    ]),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weed',
    maintainer_email='jjoonmo0212@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb4_adapter=rokey_rmf.tb4_adapter:main',
        ],
    },
)
