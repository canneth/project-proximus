
#include <arm_math.h>

#include <Arduino.h>

#include "ConvenienceFunctions.h"
// #include "TestClass.h"

#include "Matrix.h"

#include "Robot.h"

using namespace project_namespace;
// Global constants
const uint8_t led_pin = 13;


void setup() {
  Serial.begin(600000000);
  pinMode(led_pin, OUTPUT);
}

void loop() {

  Robot robot;
  // digitalWrite(led_pin, LOW);
  Serial.println("BEFORE:");
  serialPrintMatrix(robot.test_mat);
  robot.test_mat.src.pData[2] = 999;
  Serial.println("AFTER: ");
  serialPrintMatrix(robot.test_mat);
  digitalWrite(led_pin, HIGH);

  delay(1000);
}