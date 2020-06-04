
#include "LegStanceController.h"

using namespace project_namespace;

// CONSTRUCTORS
LegStanceController::LegStanceController(GaitConfig gait_config_init):
    gait_config(gait_config_init)
{}

// METHODS
Eigen::Vector3f calculateNewFootLocation(Robot& robot, Command& command, int leg_index) {
    /*
    DESCRIPTION:
    Calculates and returns the new foot position (wrt body) after a single tick.

    ARGUMENTS:
    + robot: The robot to control and command.
    + command: It contains the input commands into the robot.
    + leg_index: The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

    RETURNS:
    + new_foot_position: The x, y, z coordinates of the new foot position (wrt body) after a single tick.
    */
    Eigen::Vector3f new_foot_position(0.0, 0.0, 0.0); // Initialise

    Eigen::Vector3f current_foot_location_assuming_no_body_rpy;
    current_foot_location_assuming_no_body_rpy = robot.getFootPositionsWrtBodyAssumingNoBodyRPY().col(leg_index);
    Eigen::Vector3f foot_velocity(0.0, 0.0, 0.0); // Initialise
    foot_velocity = (
        Eigen::Vector3f()
        <<  -command.getBodyVelocity()(0),
            -command.getBodyVelocity()(1),
            0.0
    ).finished();
    Eigen::Vector3f foot_delta_p(0.0, 0.0, 0.0); // Initialise
    foot_delta_p = foot_velocity * Config::dt;
    Eigen::Matrix<float, 3, 3> foot_delta_R; // Initialise
    foot_delta_R = MyMath::eulerToMatrix(0, 0, -command.getGaitYawSpeed()*Config::dt);
    new_foot_position = foot_delta_R*current_foot_location_assuming_no_body_rpy + foot_delta_p;
    new_foot_position(2) = -command.getStanceHeight();

    return new_foot_position;
}