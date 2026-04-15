# zed_utils.py
import pyzed.sl as sl

def init_zed():
    zed = sl.Camera()
    status = zed.open()
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED: {status}")
        exit(1)
    return zed

def get_imu_data(zed):
    sensors_data = sl.SensorsData()
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
        imu_data = sensors_data.get_imu_data()
        accel = imu_data.get_linear_acceleration()
        gyro = imu_data.get_angular_velocity()
        mag = imu_data.get_magnetic_field()
        return accel, gyro, mag
    return None, None, None