
#ifndef IMU_H
#define IMU_h

#include <Eigen.h>

namespace project_namespace {
    class IMU {
        private:
        protected:
        public:
            // CONSTRUCTORS
            IMU();
            Eigen::Vector3f getAccel();
            Eigen::Vector3f getGyro();
            Eigen::Vector4f getHeadingQuaternion();
    };
}
#endif