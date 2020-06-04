
#include <Arduino.h>

#include <Eigen.h>

#include "MatrixPrinter.h"

#include "Leg.h"

using namespace project_namespace; // my namespace!
using namespace Eigen;

const uint8_t led_pin = 13;

MatrixPrinter matrix_printer;


void setup() {
    pinMode(led_pin, OUTPUT);
    Serial.begin(600000000);
   
}

void loop() {
    
    delay(1000);
  
}