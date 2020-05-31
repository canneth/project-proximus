
// Ignore deprecation warnings from Eigen library
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs" 
#include <Eigen30.h>
#pragma GCC diagnostic pop

#include <Arduino.h>

#include "GlobalConstants.h"
#include "ConvenienceFunctions.h"
#include "TestClass.h"

using namespace project_namespace;
using namespace Eigen;

// Global constants
const uint8_t led_pin = 13;

void setup() {
  Serial.begin(600000000);
  pinMode(led_pin, OUTPUT);
}

void loop() {
  digitalWrite(led_pin, LOW);

  MatrixXd a(2, 2);
  a << 1, 2,
       3, 4;
  MatrixXd b(2,2);
  b << 2, 3,
       1, 4;
  a += b;

  serialPrintMatrix(a);

  digitalWrite(led_pin, HIGH);

  delay(9999);
}