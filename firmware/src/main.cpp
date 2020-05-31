#include <Arduino.h>

#include "GlobalConstants.h"

using namespace project_namespace;

const uint8_t led_pin = 13;

void setup() {
  Serial.begin(600000000);
  pinMode(led_pin, OUTPUT);
}


void loop() {
  
}
