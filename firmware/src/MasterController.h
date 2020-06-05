
#ifndef MASTER_CONTROLLER_H
#define MASTER_CONTROLLER_H

#include "GlobalConstants.h"
#include "GaitController.h"
#include "Robot.h"
#include "Command.h"

namespace project_namespace {
    class MasterController {
        private:
            IMU& imu;
            GaitController gait_controller;
            int ticks;
        protected:
        public:
            // CONSTRUCTORS
            MasterController(IMU& imu_init);
            // METHODS
            void stepOnce(Robot& robot, Command& command);
    };
}
#endif