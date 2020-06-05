
#ifndef GAIT_CONFIG_H
#define GAIT_CONFIG_H

#include <Eigen.h>

#include "GlobalConstants.h"

#include "Config.h"

namespace project_namespace {
    class GaitConfig {
        private:
            Gait gait;
            FootTrajectory foot_trajectory;

            float gait_stance_duration;
            float gait_swing_duration;
            Eigen::Matrix<int, 4, 4> contact_schedule;
            int gait_number_of_phases;
            int gait_stance_duration_in_ticks;
            int gait_swing_duration_in_ticks;
            int gait_cycle_duration_in_ticks;
            Eigen::Vector4i gait_phase_durations_in_ticks;

            int leg_stance_duration_in_ticks;
            int leg_swing_duration_in_ticks;

            float swing_height;
        protected:
        public:
            // CONSTRUCTORS
            GaitConfig(Gait gait_init, FootTrajectory foot_trajectory_init);
            // GETTERS
            Eigen::Matrix<int, 4, 4> getContactSchedule();
            int getGaitNumberOfPhases();
            int getGaitStanceDurationInTicks();
            int getGaitSwingDurationInTicks();
            int getGaitCycleDurationInTicks();
            Eigen::Vector4i getGaitPhaseDurationsInTicks();
            int getLegStanceDurationInTicks();
            int getLegSwingDurationInTicks();
            float getSwingHeight();
            // SETTERS
            void setSwingHeight(float swing_height_arg);
    };
}
#endif