
#include "IMU.h"

using namespace project_namespace;

IMU::IMU(BNO080& imu_hardware_init):
    imu_hardware(imu_hardware_init)
{ }

Eigen::Vector3f IMU::getAccel() {
    // This returns the acceleration of the object with gravity reading removed!!
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    if (imu_hardware.dataAvailable() == true) {
        x = imu_hardware.getLinAccelX();
        y = imu_hardware.getLinAccelY();
        z = imu_hardware.getLinAccelZ();
    } else {
        x = last_accel(0);
        y = last_accel(1);
        z = last_accel(2);
    }
    Eigen::Vector3f accel_vals(x, y, z);
    last_accel = accel_vals;
    return accel_vals;
}
Eigen::Vector3f IMU::getGyro() {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    if (imu_hardware.dataAvailable() == true) {
        x = imu_hardware.getGyroX();
        y = imu_hardware.getGyroY();
        z = imu_hardware.getGyroZ();
    } else {
        x = last_gyro(0);
        y = last_gyro(1);
        z = last_gyro(2);
    }
    Eigen::Vector3f gyro_vals(x, y, z);
    last_gyro = gyro_vals;
    return gyro_vals;
}
Eigen::Quaternionf IMU::getHeadingQuaternion() {
    float quat_x = 0.0;
    float quat_y = 0.0;
    float quat_z = 0.0;
    float quat_w = 0.0;
    float quat_accuracy = 0.0;
    if (imu_hardware.dataAvailable() == true) {
        quat_w = imu_hardware.getQuatReal();
        quat_x = imu_hardware.getQuatI();
        quat_y = imu_hardware.getQuatJ();
        quat_z = imu_hardware.getQuatK();
        quat_accuracy = imu_hardware.getQuatRadianAccuracy();
        // Eigen packages Quaternions in the format [w, x, y, z]
        Eigen::Quaternionf heading_quaternion(quat_w, quat_x, quat_y, quat_z);
        last_quaternion = heading_quaternion;
        return heading_quaternion;
    } else {
        return last_quaternion;
    }
}