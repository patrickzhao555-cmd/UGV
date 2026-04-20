from setuptools import setup
import os
from glob import glob

package_name = 'ugv_sensor_sync'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'zed_sync_node   = ugv_sensor_sync.zed_sync_node:main',
            'lidar_sync_node = ugv_sensor_sync.lidar_sync_node:main',
            'uwb_node        = ugv_sensor_sync.uwb_node:main',
            'fusion_node     = ugv_sensor_sync.fusion_node:main',
        ],
    },
)
