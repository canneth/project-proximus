
#ifndef LEG_SWING_CONTROLLER_H
#define LEG_SWING_CONTROLLER_H

#include <Arduino.h>
#include <Eigen.h>

#include "GaitConfig.h"
#include "IMU.h"
#include "Robot.h"
#include "Command.h"
#include "Parameters.h"
#include "MyMath.h"

namespace project_namespace {
    class LegSwingController {
        private:
            GaitConfig& gait_config;
            IMU& imu;
        protected:
        public:
            // CONSTRUCTORS
            LegSwingController(GaitConfig& gait_config_init, IMU& imu_init);

            // METHODS
            Eigen::Vector3f calculateRaibertTouchdownLocation(Robot& robot, Command& command, int leg_index);
            Eigen::Vector3f calculateRollPitchCapturePoint(Command& command);
            Eigen::Vector3f calculateRollPitchRateCapturePoint();
            Eigen::Vector3f calculateNewFootPosition(
                Robot& robot,
                Command& command,
                int leg_index,
                float swing_proportion_completed,
                FootTrajectory trajectory_shape
            );

    };

}
#endif