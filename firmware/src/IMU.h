
#ifndef IMU_H
#define IMU_H

#include <Eigen.h>
#include <Geometry>
#include <Arduino.h>

#include <SparkFun_BNO080_Arduino_Library.h>

namespace project_namespace {
    class IMU {
        private:
            Eigen::Vector3f last_accel;
            Eigen::Vector3f last_gyro;
            Eigen::Quaternionf last_quaternion;
        protected:
        public:
            BNO080& imu_hardware;
            // CONSTRUCTORS
            IMU(BNO080& imu_hardware_init);
            Eigen::Vector3f getAccel();
            Eigen::Vector3f getGyro();
            Eigen::Quaternionf getHeadingQuaternion();
    };
}
#endif