# gyroscope.py
from zed_utils import init_zed, get_imu_data

zed = init_zed()

while True:
    accel, gyro, mag = get_imu_data(zed)
    if gyro:
        print(f"Gyro X: {gyro[0]}, Y: {gyro[1]}, Z: {gyro[2]}")