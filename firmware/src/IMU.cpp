
#include "IMU.h"

using namespace project_namespace;

IMU::IMU() {}


Eigen::Vector3f IMU::getAccel() {
    // TODO:
    // Grab value from IMU, package it and return it.
    Eigen::Vector3f accel_vals;
    return accel_vals;
}
Eigen::Vector3f getGyro() {
    // TODO:
    // Grab value from IMU, package it and return it.
    Eigen::Vector3f gyro_vals;
    return gyro_vals;
}
Eigen::Vector4f getHeadingQuaternion() {
    // TODO:
    // Grab value from IMU, package it and return it.
    
    // Quaternion returned is in the format [x, y, z, w]
    Eigen::Vector4f heading_quaternion;
    return heading_quaternion;
}