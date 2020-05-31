
#ifndef CONVENIENCE_FUNCTIONS_H
#define CONVENIENCE_FUNCTIONS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs" 
#include <Eigen30.h>
#pragma GCC diagnostic pop

#include <Arduino.h>

namespace project_namespace {

    void serialPrintMatrix(const Eigen::MatrixXd& matrix);
    void serialPrintMatrix(const Eigen::MatrixXf& matrix);

}
#endif