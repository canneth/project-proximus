
#ifndef CONVENIENCE_FUNCTIONS_H
#define CONVENIENCE_FUNCTIONS_H

#include <Arduino.h>
#include <arm_math.h>

namespace project_namespace {
    template<class T>
    void serialPrintMatrix(T matrix_to_print) {
        
        float32_t* data_array = matrix_to_print.src.pData;
        uint16_t num_of_rows = matrix_to_print.src.numRows;
        uint16_t num_of_cols = matrix_to_print.src.numCols;

        Serial.print("[");
        for (uint16_t i = 0; i < num_of_rows; i++) {
            for (uint16_t j = 0; j < num_of_cols; j++) {
                Serial.print(data_array[i*num_of_rows + j], 3); // Print to 3 dp
                if (j != num_of_cols - 1) {
                    Serial.print(", ");
                }
            }
            if (i == num_of_rows - 1) {
                Serial.println("]");
            } else {
                Serial.println("");
            }
        }
    }
}
#endif