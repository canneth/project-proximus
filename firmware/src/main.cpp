
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
#include "MasterController.h"

///////////////////////
// Test includes

///////////////////////

using namespace project_namespace;

const int led_pin = 13;

Robot robot;
Command command;
BNO080 imu_hardware;
IMU imu(imu_hardware);
MasterController master_controller(imu);

long last_time = millis();

char sprintf_buffer[255];

/////////////////////
// Test classes

/////////////////////

void setup() {
    while(!Serial);
    pinMode(led_pin, OUTPUT);
    Serial.begin(600000000);
    
    // IMU SETUP
    Wire1.begin();
    Wire1.setClock(400000); // Increase I2C data rate to 400kHz
    
    if (imu.imu_hardware.begin(0x4B, Wire1) == false)
    {
        Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1);
    }
    imu.imu_hardware.enableRotationVector(Config::dt*1000); // For quaternion; send data update every 50ms
    imu.imu_hardware.enableLinearAccelerometer(Config::dt*1000); // Send data update every 50ms
    imu.imu_hardware.enableGyro(Config::dt*1000); // Send data update every 50ms
}

void loop() {
    long current_time = millis();
    long delta_time = current_time - last_time;
    if (delta_time < (long(Config::dt*1000))) {
        return;
    }
    last_time = current_time;

    command.setMode(Mode::TROT);
    command.setSwingHeight(0.1);
    command.setBodyVelocity(Eigen::Vector3f(0.5, 0, 0));
    master_controller.stepOnce(robot, command);

    // // For serial logging purposes only
    // Eigen::Matrix<float, 3, 4> all_foot_positions;
    // all_foot_positions = robot.getFootPositionsWrtBodyTrue();
    // float FL_x = all_foot_positions.col(0)(0);
    // float FL_y = all_foot_positions.col(0)(1);
    // float FL_z = all_foot_positions.col(0)(2);
    // float FR_x = all_foot_positions.col(1)(0);
    // float FR_y = all_foot_positions.col(1)(1);
    // float FR_z = all_foot_positions.col(1)(2);
    // float BL_x = all_foot_positions.col(2)(0);
    // float BL_y = all_foot_positions.col(2)(1);
    // float BL_z = all_foot_positions.col(2)(2);
    // float BR_x = all_foot_positions.col(3)(0);
    // float BR_y = all_foot_positions.col(3)(1);
    // float BR_z = all_foot_positions.col(3)(2);
    // sprintf(
    //     sprintf_buffer,
    //     "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
    //     FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, BL_x, BL_y, BL_z, BR_x, BR_y, BR_z
    // );
    // Serial.println(sprintf_buffer);

}