import os

from setuptools import find_packages, setup


package_name = 'ugv_motor_controller'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'TEENSY_SIDE_PID_ARCHITECTURE.md']),
        (
            os.path.join('share', package_name, 'launch'),
            [os.path.join('launch', name) for name in os.listdir('launch') if name.endswith('.py')],
        ),
        (
            os.path.join('share', package_name, 'firmware', 'teensy_4_1_side_pid_controller'),
            [
                os.path.join(
                    'firmware',
                    'teensy_4_1_side_pid_controller',
                    'teensy_4_1_side_pid_controller.ino',
                )
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.com',
    description='Jetson bridge node and Teensy 4.1 firmware for UGV motor control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_controller_bridge = ugv_motor_controller.motor_controller_bridge:main',
        ],
    },
)
