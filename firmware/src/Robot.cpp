
#include "Robot.h"

using namespace project_namespace;

// CONSTRUCTORS
Robot::Robot():
    front_left_leg(
        Leg(
            0.155, // d_x
            0.04, // d_y
            0.038, // d_j2_j1_bx
            0.071, // d_j2_j1_by
            0.160, // l_2
            0.160, // l_3
            Eigen::Vector3i(1, 1, 1) // TODO: To be set after assembling and testing
        )
    ),
    front_right_leg(
        Leg(
            0.155, // d_x
            -0.04, // d_y
            0.038, // d_j2_j1_bx
            -0.071, // d_j2_j1_by
            0.160, // l_2
            0.160, // l_3
            Eigen::Vector3i(1, 1, 1) // TODO: To be set after assembling and testing
        )
    ),
    back_left_leg(
        Leg(
            -0.155, // d_x
            0.04, // d_y
            -0.038, // d_j2_j1_bx
            0.071, // d_j2_j1_by
            0.160, // l_2
            0.160, // l_3
            Eigen::Vector3i(1, 1, 1) // TODO: To be set after assembling and testing
        )
    ),
    back_right_leg(
        Leg(
            -0.155, // d_x
            -0.04, // d_y
            -0.038, // d_j2_j1_bx
            -0.071, // d_j2_j1_by
            0.160, // l_2
            0.160, // l_3
            Eigen::Vector3i(1, 1, 1) // TODO: To be set after assembling and testing
        )
    ),

    stance_polygon_length(0.0),
    stance_polygon_width(0.0),
    stance_height(0.0),

    body_roll(0.0),
    body_pitch(0.0),
    body_yaw(0.0),

    foot_positions_wrt_body_true(Eigen::Matrix<float, 3, 4>::Zero()),
    foot_positions_wrt_body_assuming_no_body_rpy(Eigen::Matrix<float, 3, 4>::Zero()),
    foot_positions_wrt_body_at_rest(Eigen::Matrix<float, 3, 4>::Zero()),

    joint_angles(Eigen::Matrix<float, 3, 4>::Zero()),
    contact_pattern(Eigen::Vector4i::Zero())
{}

// GETTERS
float Robot::getStancePolygonLength() {
    return stance_polygon_length;
}
float Robot::getStancePolygonWidth() {
    return stance_polygon_width;
}
float Robot::getStanceHeight() {
    return stance_height;
}
float Robot::getBodyRoll() {
    return body_roll;
}
float Robot::getBodyPitch() {
    return body_pitch;
}
float Robot::getBodyYaw() {
    return body_yaw;
}
Eigen::Matrix<float, 3, 4> Robot::getFootPositionsWrtBodyTrue() {
    return foot_positions_wrt_body_true;
}
Eigen::Matrix<float, 3, 4> Robot::getFootPositionsWrtBodyAssumingNoBodyRPY() {
    return foot_positions_wrt_body_assuming_no_body_rpy;
}
Eigen::Matrix<float, 3, 4> Robot::getFootPositionsWrtBodyAtRest() {
    return foot_positions_wrt_body_at_rest;
}
Eigen::Matrix<float, 3, 4> Robot::getJointAngles() {
    return joint_angles;
}
Eigen::Vector4i Robot::getContactPattern() {
    return contact_pattern;
}

// SETTERS
void Robot::setStancePolygonLength(float stance_polygon_length_arg) {
    stance_polygon_length = stance_polygon_length_arg;
}
void Robot::setStancePolygonWidth(float stance_polygon_width_arg) {
    stance_polygon_width = stance_polygon_width_arg;
}
void Robot::setStanceHeight(float stance_height_arg) {
    stance_height = stance_height_arg;
}
void Robot::setBodyRoll(float body_roll_arg) {
    body_roll = body_roll_arg;
}
void Robot::setBodyPitch(float body_pitch_arg) {
    body_pitch = body_pitch_arg;
}
void Robot::setBodyYaw(float body_yaw_arg) {
    body_yaw = body_yaw_arg;
}
void Robot::setFootPositionsWrtBodyTrue(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_true_arg) {
    foot_positions_wrt_body_true = foot_positions_wrt_body_true_arg;
}
void Robot::setFootPositionsWrtBodyAssumingNoBodyRPY(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_assuming_no_body_rpy_arg) {
    foot_positions_wrt_body_assuming_no_body_rpy = foot_positions_wrt_body_assuming_no_body_rpy_arg;
}
void Robot::setFootPositionsWrtBodyAtRest(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_at_rest_arg) {
    foot_positions_wrt_body_at_rest = foot_positions_wrt_body_at_rest_arg;
}
void Robot::setJointAngles(Eigen::Matrix<float, 3, 4> joint_angles_arg) {
    joint_angles = joint_angles_arg;
}
void Robot::setContactPattern(Eigen::Vector4i contact_pattern_arg) {
    contact_pattern = contact_pattern_arg;
}

// METHODS
void Robot::moveAllFeet(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_cmd) {
    /*
    DESCRIPTION:
    Moves all feet to their respective positions as specified in foot_positions_wrt_body, where
    each column represents the x, y, z coordinates of each foot.

    ARUGMENTS:
    + foot_positions_wrt_body_cmd: The commanded foot positions.
    of the foot, [FL, FR, BL, BR].
    */

    front_left_leg.moveFoot(foot_positions_wrt_body_cmd.col(0));
    front_right_leg.moveFoot(foot_positions_wrt_body_cmd.col(1));
    back_left_leg.moveFoot(foot_positions_wrt_body_cmd.col(2));
    back_right_leg.moveFoot(foot_positions_wrt_body_cmd.col(3));
    foot_positions_wrt_body_true = foot_positions_wrt_body_cmd;
}