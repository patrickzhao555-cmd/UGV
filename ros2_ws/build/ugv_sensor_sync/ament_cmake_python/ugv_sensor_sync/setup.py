from setuptools import find_packages
from setuptools import setup

setup(
    name='ugv_sensor_sync',
    version='0.0.1',
    packages=find_packages(
        include=('ugv_sensor_sync', 'ugv_sensor_sync.*')),
)
