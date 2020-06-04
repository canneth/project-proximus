
#include "GaitConfig.h"

using namespace project_namespace;

// CONSTRUCTORS
GaitConfig::GaitConfig(
    Gait gait_init
):
    gait(gait_init),
    // The following are initialised conditional upon gait_init
    gait_stance_duration(0.0),
    gait_swing_duration(0.0),
    contact_schedule(Eigen::Matrix<int, 4, 4>::Zero()),
    gait_number_of_phases(0),
    gait_stance_duration_in_ticks(0),
    gait_swing_duration_in_ticks(0),
    gait_cycle_duration_in_ticks(0),
    gait_phase_durations_in_ticks(Eigen::Vector4i::Zero()),
    leg_stance_duration_in_ticks(0),
    leg_swing_duration_in_ticks(0),
    swing_height(0.0)
{
    if (gait_init == Gait::TROT) {
        gait_stance_duration = 0.02;
        gait_swing_duration = 0.15;
        contact_schedule << (
            Eigen::Matrix<int, 4, 4>()
            <<  1, 1, 1, 0,
                1, 0, 1, 1,
                1, 0, 1, 1,
                1, 1, 1, 0
        ).finished();
        gait_number_of_phases = 4;
        gait_stance_duration_in_ticks = int(gait_stance_duration/Config::dt);
        gait_swing_duration_in_ticks = int(gait_swing_duration/Config::dt);
        gait_cycle_duration_in_ticks = 2*gait_stance_duration_in_ticks + 2*gait_swing_duration_in_ticks;
        gait_phase_durations_in_ticks << (
            Eigen::Vector4i()
            <<  gait_stance_duration_in_ticks,
                gait_swing_duration_in_ticks,
                gait_stance_duration_in_ticks,
                gait_swing_duration_in_ticks
        ).finished();
        leg_stance_duration_in_ticks = 2*gait_stance_duration_in_ticks + gait_swing_duration_in_ticks;
        leg_swing_duration_in_ticks = gait_swing_duration_in_ticks;
        swing_height = 0.0;
    }
}

// GETTERS
Eigen::Matrix<int, 4, 4> GaitConfig::getContactSchedule() {
    return contact_schedule;
}
int GaitConfig::getGaitNumberOfPhases() {
    return gait_number_of_phases;
}
int GaitConfig::getGaitStanceDurationInTicks() {
    return gait_stance_duration_in_ticks;
}
int GaitConfig::getGaitSwingDurationInTicks() {
    return gait_swing_duration_in_ticks;
}
int GaitConfig::getGaitCycleDurationInTicks() {
    return gait_cycle_duration_in_ticks;
}
Eigen::Vector4i GaitConfig::getGaitPhaseDurationsInTicks() {
    return gait_phase_durations_in_ticks;
}
int GaitConfig::getLegStanceDurationInTicks() {
    return leg_stance_duration_in_ticks;
}
int GaitConfig::getLegSwingDurationInTicks() {
    return leg_swing_duration_in_ticks;
}
float GaitConfig::getSwingHeight() {
    return swing_height;
}

