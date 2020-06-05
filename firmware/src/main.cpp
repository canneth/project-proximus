
// Arduino functionality libraries
#include <Arduino.h>
#include <Wire.h>

// Hardware libraries
#include <SparkFun_BNO080_Arduino_Library.h>

// Math libraries
#include <Eigen.h>

// My libraries
#include "MatrixPrinter.h"

#include "GlobalConstants.h"

#include "Config.h"
#include "Robot.h"
#include "Command.h"
#include "IMU.h"

///////////////////////
// Test includes

///////////////////////

using namespace project_namespace; // my namespace!
using namespace Eigen;

const int led_pin = 13;

MatrixPrinter matrix_printer;

Robot robot;
Command command;

BNO080 imu_hardware;
IMU imu(imu_hardware);

/////////////////////
// Test classes

/////////////////////

void setup() {
    pinMode(led_pin, OUTPUT);
    Serial.begin(600000000);
    
    // IMU SETUP
    Wire.begin();
    if (imu.imu_hardware.begin() == false)
    {
        Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1);
    }
    Wire.setClock(400000); // Increase I2C data rate to 400kHz
    // imu.imu_hardware.enableRotationVector(50); // For quaternion; send data update every 50ms
    // imu.imu_hardware.enableAccelerometer(50); // Send data update every 50ms
    imu.imu_hardware.enableGyro(50); // Send data update every 50ms
}

void loop() {
    delay(1000);
}