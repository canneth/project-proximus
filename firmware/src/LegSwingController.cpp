
#include "LegSwingController.h"
#include "MatrixPrinter.h"

using namespace project_namespace;

// CONSTRUCTORS

LegSwingController::LegSwingController(GaitConfig& gait_config_init, IMU& imu_init):
    gait_config(gait_config_init),
    imu(imu_init)
{ }

// METHODS

Eigen::Vector3f LegSwingController::calculateRaibertTouchdownLocation(Robot& robot, Command& command, int leg_index) {
    /*
    DESCRIPTION:
    Calculates the touchdown location of the foot in swing using the Raibert heuristic.

    ARGUMENTS:
    + robot: The robot to control and command.
    + command: Contains the input commands into the robot.
    + leg_index: The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

    RETURNS:
    + touchdown_location: The x, y, z coordinates of the touchdown location wrt body, in the body frame.
    */

    Eigen::Vector3f touchdown_location(0.0, 0.0, 0.0); // Initialise

    // Ratios between touchdown distance from neutral foot position and total xy planar body movement
    float alpha = 0.0; // For gait translation.
    float beta = 0.0; // For gait yaw.

    // If gait translation involves both antero-posterior and lateral movement,
    // alpha will be a vector sum of respective alphas in both directions, clipped within 0 and 1.
    if (command.getBodyVelocity()(0) > 0) { // If moving forwards
        alpha = constrain(sqrt(pow(Parameters::lateral_alpha, 2.0) + pow(Parameters::forwards_alpha, 2.0)), 0.0, 1.0);
    } else if (command.getBodyVelocity()(0) < 0) {
        alpha = constrain(sqrt(pow(Parameters::lateral_alpha, 2.0) + pow(Parameters::backwards_alpha, 2.0)), 0.0, 1.0);
    } else {
        alpha = Parameters::lateral_alpha;
    }
    // Calculate touchdown location displacement due to linear movement
    Eigen::Vector3f touchdown_displacement_from_neutral_location(0.0, 0.0, 0.0);
    touchdown_displacement_from_neutral_location =
        alpha
        *gait_config.getLegStanceDurationInTicks()
        *Config::dt
        *command.getBodyVelocity()
    ;
    touchdown_displacement_from_neutral_location(2) = 0; // z-coordinate not important for touchdown location
    // Calculate touchdown location displacement due to gait yaw
    float projected_total_gait_yaw_during_stance_phase =
        beta
        *gait_config.getLegStanceDurationInTicks()
        *Config::dt
        *command.getGaitYawSpeed()
    ;
    Eigen::Matrix<float, 3, 3> projected_total_gait_yaw_rotation_matrix;
    projected_total_gait_yaw_rotation_matrix = MyMath::eulerToMatrix(0, 0, projected_total_gait_yaw_during_stance_phase);
    
    // Linear combination of effects of both linear and yaw movement
    touchdown_location =
        projected_total_gait_yaw_rotation_matrix*robot.getFootPositionsWrtBodyAtRest().col(leg_index)
        + touchdown_displacement_from_neutral_location
    ;

    return touchdown_location;
}

Eigen::Vector3f LegSwingController::calculateRollPitchCapturePoint(Command& command) {
    /*
    DESCRIPTION:
    Calculates a "capture point" based on the roll and pitch angle errors of the body of the robot.

    ARGUMENTS:
    + command: The commanded body roll and pitch angles are drawn from this.
    + roll_gain: The P-gain on the roll angle.
    + pitch_gain: The P-gain on the pitch angle.

    RETURNS:
    + rp_capture_point: The offsets from the touchdown location dependent on
    the roll and pitch angle errors of the robot body.
    */

    Eigen::Vector3f rp_capture_point(0.0, 0.0, 0.0); // Initialise

    Eigen::Vector3f rpy_angles;
    rpy_angles = imu.getHeadingQuaternion().normalized().toRotationMatrix().eulerAngles(0, 1, 2);
    float roll_error =  command.getBodyRoll() - rpy_angles(0);
    float pitch_error =  command.getBodyPitch() - rpy_angles(1);
    // When roll error is -ve, it means that the robot is rolling towards the right
    // The correction saggital-axis displacement of the feet should thus be towards the body's right (-ve y-direction)
    // When pitch error is -ve, it means the robot is pitching forwards
    // The correction frontal-axis displacement of the feet should thus be towards the body's front (+ve x-direction)
    // The z-coordinate is irrelevant here.
    rp_capture_point = (
        Eigen::Vector3f()
        <<  -1.0*Parameters::pitch_gain*pitch_error,
            Parameters::roll_gain*roll_error,
            0.0
    ).finished();

    return rp_capture_point;
}

Eigen::Vector3f LegSwingController::calculateRollPitchRateCapturePoint() {
    /*
    DESCRIPTION:
    Calculates a "capture point" based on the roll and pitch rates of the body of the robot.

    ARGUMENTS:
    + roll_rate_gain: The P-gain on the roll angle rate.
    + pitch_rate_gain: The P-gain on the pitch angle rate.

    RETURNS:
    + rp_rate_capture_point: The offsets from the touchdown location dependent on
    the roll and pitch angle rates of the robot body.
    */
    Eigen::Vector3f rp_rate_capture_point(0.0, 0.0, 0.0); // Initialise
    Eigen::Vector3f gyro_vals(0.0, 0.0, 0.0);
    gyro_vals = imu.getGyro();
    float roll_rate = gyro_vals(0);
    float pitch_rate = gyro_vals(1);
    // When roll rate is +ve, it means that the robot is rolling towards the right
    // The correction saggital-axis displacement of the feet should thus be towards the body's right (-ve y-direction)
    // When pitch rate is +ve, it means the robot is pitching forwards
    // The correction frontal-axis displacement of the feet should thus be towards the body's front (+ve x-direction)
    rp_rate_capture_point = (
        Eigen::Vector3f()
        <<  Parameters::pitch_rate_gain*pitch_rate,
            -1.0*Parameters::roll_rate_gain*roll_rate,
            0.0
    ).finished();

    return rp_rate_capture_point;
}

Eigen::Vector3f LegSwingController::calculateNewFootPosition(
    Robot& robot,
    Command& command,
    int leg_index,
    float swing_proportion_completed,
    FootTrajectory trajectory_shape
) {
    /*
    DESCRIPTION:
    Calculates and returns the new foot position (wrt body) after a single tick.

    ARGUMENTS:
    + robot: The robot to control and command.
    + command: It contains the input commands into the robot.
    + leg_index: The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).
    + swing_proportion_completed: The proportion of the swing phase completed by the foot.
    + trajectory_shape: The shape of the trajectory of the foot.

    RETURNS:
    + new_foot_position:The x, y, z coordinates of the new foot position (wrt body, in body frame)
    after a single tick.
    */

    Eigen::Vector3f new_foot_position(0.0, 0.0, 0.0); // Initialise

    assert(swing_proportion_completed >= 0 && swing_proportion_completed <= 1);
    Eigen::Vector3f current_foot_location_assuming_no_body_rpy(0.0, 0.0, 0.0);
    current_foot_location_assuming_no_body_rpy = robot.getFootPositionsWrtBodyAssumingNoBodyRPY().col(leg_index);
    Eigen::Vector3f raibert_touchdown_location(0.0, 0.0, 0.0);
    raibert_touchdown_location = calculateRaibertTouchdownLocation(robot, command, leg_index);

    Eigen::Vector3f touchdown_location(0.0, 0.0, 0.0);
    touchdown_location = raibert_touchdown_location;

    // Applying rp_capture_point
    Eigen::Vector3f rp_capture_point(0.0, 0.0, 0.0);
    rp_capture_point = calculateRollPitchCapturePoint(command);
    touchdown_location(0) = touchdown_location(0) + rp_capture_point(0); // Apply x-offset to all legs
    // Apply y-offset conditionally to the legs as follows:
    if ((rp_capture_point(1) > 0) and (leg_index == 0 or leg_index == 2)) {
        // Robot is moving with an undesired velocity in the +ve y-direction (the robot's left)
        // Along the y-direction, move only the legs on the left of the robot to counteract this
        touchdown_location(1) = touchdown_location(1) + rp_capture_point(1);
    } else if ((rp_capture_point(1) < 0) and (leg_index == 1 or leg_index == 3)) {
        // Robot is moving with an undesired velocity in the -ve y-direction (the robot's right)
        // Along the y-direction, move only the legs on the right of the robot to counteract this
        touchdown_location(1) = touchdown_location(1) + rp_capture_point(1);
    }

    // Using rp_rate_capture_point
    Eigen::Vector3f rp_rate_capture_point(0.0, 0.0, 0.0);
    rp_rate_capture_point = calculateRollPitchRateCapturePoint();
    touchdown_location(0) = touchdown_location(0) + rp_rate_capture_point(0); // Apply x-offset to all legs
    // Apply y-offset conditionally to the legs as follows
    if ((rp_rate_capture_point[1] > 0) and (leg_index == 0 or leg_index == 2)) {
        // Robot is moving with an undesired velocity in the +ve y-direction (the robot's left)
        // Along the y-direction, move only the legs on the left of the robot to counteract this
        touchdown_location(1) = touchdown_location(1) + rp_rate_capture_point(1);
    } else if ((rp_rate_capture_point[1] < 0) and (leg_index == 1 or leg_index == 3)) {
        // Robot is moving with an undesired velocity in the -ve y-direction (the robot's right)
        // Along the y-direction, move only the legs on the right of the robot to counteract this
        touchdown_location(1) = touchdown_location(1) + rp_rate_capture_point(1);
    }

    float time_to_touchdown = Config::dt * gait_config.getLegSwingDurationInTicks() * (1.0 - swing_proportion_completed);
    Eigen::Vector3f foot_delta_p(0.0, 0.0, 0.0);
    foot_delta_p = (touchdown_location - current_foot_location_assuming_no_body_rpy)/(time_to_touchdown / Config::dt);
    foot_delta_p(2) = 0;
    float z_from_ground = 0.0;
    if (trajectory_shape == FootTrajectory::SEMICIRCULAR) {
        z_from_ground = gait_config.getSwingHeight()*sin(swing_proportion_completed*PI);
    } else if (trajectory_shape == FootTrajectory::TRIANGULAR) {
        if (swing_proportion_completed <= 0.5) {
            z_from_ground = gait_config.getSwingHeight()*swing_proportion_completed;
        } else {
            z_from_ground = gait_config.getSwingHeight()*(1.0 - swing_proportion_completed);
        }
    }
    new_foot_position = current_foot_location_assuming_no_body_rpy + foot_delta_p;
    new_foot_position(2) = -command.getStanceHeight() + z_from_ground;

    return new_foot_position;
}