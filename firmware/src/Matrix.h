
#ifndef MATRIX_H
#define MATRIX_H

#include <arm_math.h>

namespace project_namespace {
    template<uint16_t n, uint16_t m>
    class Matrix {
        private:
            
        protected:
        public:
            // PUBLIC ATTRIBUTES
            arm_matrix_instance_f32 src;

            // CONSTRUCTORS
            Matrix() {
                float32_t matrix_data_array[n*m] = {};
                arm_mat_init_f32(&src, n, m, (float32_t*)matrix_data_array);   
            }
            
            // METHODS
            float32_t& getIndexPtr(uint16_t i, uint16_t j) {
                return src.pData[i*src.numRows + j];
            }
    };
}

#endif