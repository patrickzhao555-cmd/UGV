from setuptools import find_packages, setup

package_name = 'ugv_serial_odom'

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
    maintainer='bluelule',
    maintainer_email='patrickzhao555@gmail.com',
    description='Legacy serial odometry publisher for the UGV ROS 2 stack.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['serial_odom = ugv_serial_odom.serial_odom_node:main',
        ],
    },
)
