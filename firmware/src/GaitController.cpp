
#include "GaitController.h"

using namespace project_namespace;

// CONSTRUCTORS
GaitController::GaitController(Gait gait_init):
    gait(gait_init),
    gait_config(GaitConfig(gait_init))
{}

// METHODS
int GaitController::calculateGaitPhaseIndex(int ticks) {
    /*
    DESCRIPTION:
    Calculates and returns the phase index of the gait that the robot is in
    given elapsed time in number of ticks.

     ARGUMENTS:
    + ticks: The elapsed time into the current gait in ticks, where 1 tick is 1s/dt.
    
    RETURNS:
    + gait_phase_index: The phase index of the current phase of the gait.
    */

    int ticks_into_current_gait_cycle = ticks % gait_config.getGaitCycleDurationInTicks();
    int tick_sum = 0;
    int gait_phase_index = 99; // Initialise with impossible value, caught by assert later
    for (int i = 0; i < gait_config.getGaitNumberOfPhases(); i++) {
        tick_sum += gait_config.getGaitPhaseDurationsInTicks()(i);
        if (ticks_into_current_gait_cycle < tick_sum) {
            gait_phase_index = i;
        }
    }
    assert ((gait_phase_index >= 0) && (gait_phase_index <= gait_config.getGaitNumberOfPhases()));
    return gait_phase_index;
}

int GaitController::calculateTicksIntoCurrentGaitPhase(int ticks) {
    /*
    DESCRIPTION:
    Calculates and returns the elapsed time, in ticks, from the beginning of the current phase
    of the gait (NOT LEG PHASE!!!).

    ARGUMENTS:
    + ticks: The elapsed time into the current gait in ticks, where 1 tick is 1s/dt.

    RETURNS:
    + ticks_into_current_phase: The elapsed time, in ticks, from the start of the current phase of the gait.
    */

    int ticks_into_current_gait_cycle = ticks % gait_config.getGaitCycleDurationInTicks();
    int tick_sum = 0;
    int ticks_into_current_phase = 999; // Initialise with impossible value, caught by assert later
    for (int i = 0; i < gait_config.getGaitNumberOfPhases(); i++) {
        tick_sum += gait_config.getGaitPhaseDurationsInTicks()(i);
        if (ticks_into_current_gait_cycle < tick_sum) {
            ticks_into_current_phase =
                ticks_into_current_gait_cycle
                - tick_sum
                + gait_config.getGaitPhaseDurationsInTicks()(i)
            ;
        }
    }
    assert (
        (ticks_into_current_phase >= 0)
        && (
            (ticks_into_current_phase <= gait_config.getGaitStanceDurationInTicks())
            || (ticks_into_current_phase <= gait_config.getGaitSwingDurationInTicks())
        )
    );
    return ticks_into_current_phase;
}

int GaitController::calculateTicksIntoCurrentLegPhase(int ticks, int leg_index) {
    /*
    DESCRIPTION:
    Calculates and returns the number of ticks into the current leg phase (NOT GAIT PHASE!!!)
    for the given leg specified by leg_index.

    ARGUMENTS:
    + ticks: The elapsed time since the beginning of the gait in ticks, where 1 tick is 1s/dt.
    + leg_index: The index of the leg to calculate for. {FL: 0, FR: 1, BL: 2, BR: 3}

    RETURNS:
    + ticks_into_current_leg_phase: The number of ticks into the leg's own phase (NOT GAIT PHASE!!!).
    */

    int current_gait_phase_index = calculateGaitPhaseIndex(ticks);
    int ticks_into_current_gait_phase = calculateTicksIntoCurrentGaitPhase(ticks);
    int ticks_into_current_leg_phase = 999; // Initialise with impossible value, caught by assert later

    if (gait == Gait::TROT) {
        if (leg_index == 0) {
            // Front-left leg
            if (current_gait_phase_index == 0) {
                // FL is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 1) {
                // FL is in the middle of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 2) {
                // FL is approaching end of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + gait_config.getGaitSwingDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 3) {
                // FL is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            }
        }
        if (leg_index == 1) {
            // Front-right leg
            if (current_gait_phase_index == 0) {
                // FR is approaching end of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + gait_config.getGaitSwingDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 1) {
                // FR is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 2) {
                // FR is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 3) {
                // FR is in the middle of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            }
        }
        if (leg_index == 2) {
            // Back-left leg
            if (current_gait_phase_index == 0) {
                // BL is approaching end of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + gait_config.getGaitSwingDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 1) {
                // BL is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 2) {
                // BL is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 3) {
                // BL is in the middle of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            }
        }
        if (leg_index == 3) {
            // Back-right leg
            if (current_gait_phase_index == 0) {
                // BR is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            } else if (current_gait_phase_index == 1) {
                // BR is in the middle of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 2) {
                // BR is approaching end of its leg stance phase
                ticks_into_current_leg_phase =
                    gait_config.getGaitStanceDurationInTicks()
                    + gait_config.getGaitSwingDurationInTicks()
                    + ticks_into_current_gait_phase
                ;
            } else if (current_gait_phase_index == 3) {
                // BR is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase;
            }
        }
    }
    assert(
        (ticks_into_current_leg_phase >= 0)
        && (ticks_into_current_leg_phase <= gait_config.getLegStanceDurationInTicks())
    ); // leg stance duration is always longer than swing stance duration
    
    return ticks_into_current_leg_phase;
}

Eigen::Vector4i GaitController::calculateContactPattern(int ticks) {
    /*
    DESCRIPTION:
    Calculates and returns the contact pattern of the feet given the elapsed time since
    gait start in ticks.

    ARGUMENTS:
    + ticks: The elapsed time since the beginning of the gait in ticks, where 1 tick is 1s/dt.

    RETURNS:
    + contact_pattern: The contact pattern of the legs at the time given in ticks.
    0 = swing, 1 = stance, [FL, FR, BL, BR].
    */
    
    int current_gait_phase_index = calculateGaitPhaseIndex(ticks);
    Eigen::Vector4i contact_pattern(0, 0, 0, 0);
    contact_pattern = gait_config.getContactSchedule().col(current_gait_phase_index);

    return contact_pattern;
}
