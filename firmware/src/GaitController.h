
#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Eigen.h>

#include "GlobalConstants.h"

#include "GaitConfig.h"
#include "LegStanceController.h"
#include "LegSwingController.h"
#include "IMU.h"

namespace project_namespace {

    class GaitController {
        private:
            Gait gait;
            FootTrajectory trajectory_shape;
            IMU& imu;
            GaitConfig gait_config;
        protected:
        public:
            // PUBLIC MEMBERS
            LegStanceController leg_stance_controller;
            LegSwingController leg_swing_controller;
            // CONSTRUCTORS
            GaitController(Gait gait_init, FootTrajectory trajectory_shape_init, IMU& imu_init);
            // METHODS
            int calculateGaitPhaseIndex(int ticks);
            int calculateTicksIntoCurrentLegPhase(int ticks, int leg_index);
            int calculateTicksIntoCurrentGaitPhase(int ticks);
            Eigen::Vector4i calculateContactPattern(int ticks);
            Eigen::Matrix<float, 3, 4> calculateAllNewFootPositions(Robot& robot, Command& command, int ticks);
    };

}
#endif