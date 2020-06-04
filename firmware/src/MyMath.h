
#ifndef MY_MATH_H
#define MY_MATH_H

#include <Eigen.h>
#include <Geometry>

namespace project_namespace {
    class MyMath {
        private:
        protected:
        public:
            static Eigen::Matrix<float, 3, 3> eulerToMatrix(float x, float y, float z);
    };
}
#endif