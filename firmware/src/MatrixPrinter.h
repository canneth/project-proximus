#ifndef MATRIX_PRINTER_H
#define MATRIX_PRINTER_H

#include <Arduino.h>
#include <Eigen.h>

namespace project_namespace {
    class MatrixPrinter {
        private:
        protected:
        public:
            // CONSTRUCTORS
            MatrixPrinter() {}

            // METHODS
            template<typename T>
            static void print(T matrix_to_print) {
   
                uint8_t num_of_rows = matrix_to_print.rows();
                uint8_t num_of_cols = matrix_to_print.cols();

                Serial.print("num_of_rows: "); Serial.println(num_of_rows);
                Serial.print("num_of_cols: "); Serial.println(num_of_cols);       
                Serial.println();
            
                for (uint8_t i = 0; i < num_of_rows; i++)
                {
                    for (uint8_t j = 0; j < num_of_cols; j++)
                    {
                        Serial.print(matrix_to_print(i, j), 5); // To 5 dp
                        Serial.print(", ");
                    }
                    Serial.println();
                }
                Serial.println();
            }
    };

}
#endif