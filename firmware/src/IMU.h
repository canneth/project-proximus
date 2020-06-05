
#ifndef IMU_H
#define IMU_H

#include <Eigen.h>
#include <Geometry>

#include <SparkFun_BNO080_Arduino_Library.h>

namespace project_namespace {
    class IMU {
        private:
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