
#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Eigen.h>

#include "GlobalConstants.h"

#include "GaitConfig.h"

namespace project_namespace {

    class GaitController {
        private:
            Gait gait;
            
            GaitConfig gait_config;
        protected:
        public:
            // CONSTRUCTORS
            GaitController(Gait gait_init);
            // METHODS
            int calculateGaitPhaseIndex(int ticks);
            int calculateTicksIntoCurrentLegPhase(int ticks, int leg_index);
            int calculateTicksIntoCurrentGaitPhase(int ticks);
            Eigen::Vector4i calculateContactPattern(int ticks);
    };

}
#endif