from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 필수 패키지 인덱스와 package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # config, maps 파일 포함
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
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
            'map_switcher = rokey_pjt.map_switcher:main',
            'bfs_path_finder = rokey_pjt.bfs_path_finder:main',
            'pose_publisher = rokey_pjt.pose_publisher:main',
            'path_follower = rokey_pjt.path_follower:main',
            'dock_controller = rokey_pjt.dock_controller:main',
            'pose_publisher_v2 = rokey_pjt.pose_publisher_v2:main',
            'bfs_path_finder_v2 = rokey_pjt.bfs_path_finder_v2:main',
            
        ],
    },
)
