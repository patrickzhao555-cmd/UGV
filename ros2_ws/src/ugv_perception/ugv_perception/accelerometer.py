# accelerometer.py
from zed_utils import init_zed, get_imu_data

zed = init_zed()

while True:
    accel, gyro, mag = get_imu_data(zed)
    if accel:
        print(f"Accel X: {accel[0]}, Y: {accel[1]}, Z: {accel[2]}")