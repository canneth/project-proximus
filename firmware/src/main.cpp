
#include <Arduino.h>

#include <Eigen.h>

#include "MatrixPrinter.h"

#include "GlobalConstants.h"

#include "Config.h"
#include "Robot.h"

#include "GaitConfig.h"

using namespace project_namespace; // my namespace!
using namespace Eigen;

const int led_pin = 13;

MatrixPrinter matrix_printer;

float Config::dt = 0.01;
Robot robot;

void setup() {
    pinMode(led_pin, OUTPUT);
    Serial.begin(600000000);
}

void loop() {


    delay(1000);
}