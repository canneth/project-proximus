
#ifndef LEG_STANCE_CONTROLLER_H
#define LEG_STANCE_CONTROLLER_H

#include <Eigen.h>

#include "GaitConfig.h"
#include "Robot.h"
#include "Command.h"
#include "MyMath.h"

namespace project_namespace {
    class LegStanceController {
        private:
            GaitConfig gait_config;
        protected:
        public:
            // CONSTRUCTORS
            LegStanceController(GaitConfig gait_config_init);
            
            // METHODS
            Eigen::Vector3f calculateNewFootLocation(Robot& robot, Command& command, int leg_index);
    };
}
#endif