# magnetometer.py
from zed_utils import init_zed, get_imu_data

zed = init_zed()

while True:
    accel, gyro, mag = get_imu_data(zed)
    if mag:
        print(f"Mag X: {mag[0]}, Y: {mag[1]}, Z: {mag[2]}")