from setuptools import find_packages, setup

package_name = 'ugv_perception'

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
    maintainer_email='bluelule@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
    'zed_obj_distance = ugv_perception.zed_obj_distance:main',
    'obstacle_warning = ugv_perception.obstacle_warning:main',

],

    },
)
