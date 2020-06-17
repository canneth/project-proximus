
#include "Leg.h"

// This include fixes the output value for theta_1 in calcualteIKFoot() for some reason.
// Without it, theta_1 always gets set to 1.57080, but with it it behaves as expected.
// No idea why...
#include <Arduino.h>

using namespace project_namespace;

// CONSTRUCTORS
Leg::Leg(
    float d_x_init,
    float d_y_init,
    float d_j2_j1_bx_init,
    float d_j2_j1_by_init,
    float l_2_init,
    float l_3_init,
    Eigen::Vector3i joint_servo_directions_init
):
    d_x(d_x_init),
    d_y(d_y_init),
    d_j2_j1_bx(d_j2_j1_bx_init),
    d_j2_j1_by(d_j2_j1_by_init),
    l_2(l_2_init),
    l_3(l_3_init),
    joint_servo_directions(joint_servo_directions_init),

    joint_angles(Eigen::Vector3f::Zero()),
    foot_position_wrt_body(Eigen::Vector3f::Zero())
{}

// GETTERS
Eigen::Vector3i Leg::getJointServoDirections() {
    return joint_servo_directions;
}
Eigen::Vector3f Leg::getJointAngles() {
    return joint_angles;
}
Eigen::Vector3f Leg::getFootPositionWrtBody() {
    return foot_position_wrt_body;
}

// SETTERS
void Leg::setJointAngles(Eigen::Vector3f joint_angles_arg) {
    joint_angles = joint_angles_arg;
}
void Leg::setFootPositionWrtBody(Eigen::Vector3f foot_position_wrt_body_arg) {
    foot_position_wrt_body = foot_position_wrt_body_arg;
}

// METHODS
Eigen::Vector3f Leg::calculateIKFoot(Eigen::Vector3f foot_pos) {
    /*
    DESCRIPTION:
    Calculates the IK of the leg, yielding the corresponding joint_angles for the given foot_pos.
    joint_angles and foot_position_wrt_body are updated after calculation.

    ARGUMENTS:
    + foot_pos: The position of the foot wrt the body in the body frame.

    RETURNS:
    + joint_angles: The joint angles calculated.
    */

    float theta_1 = 0.0;
    float theta_2 = 0.0;
    float theta_3 = 0.0;

    float p_x = foot_pos(0);
    float p_y = foot_pos(1);
    float p_z = foot_pos(2);
    // float p_yz = sqrt(pow(p_y, 2.0) + pow(p_z, 2.0));

    // Part 1: Finding theta_1
    float l_a = pow((pow((p_y - d_y), 2.0) + pow(p_z, 2.0)), 0.5);
    float rho = asin(abs(p_z)/l_a);
    float beta = acos(abs(d_j2_j1_by)/l_a);
    theta_1 = beta - rho;
    
    // Part 2: theta_3
    float l_b = sqrt(pow(l_a, 2.0) - pow(d_j2_j1_by, 2.0));
    float l_eff = sqrt(pow(l_b, 2.0) + pow((p_x - d_x - d_j2_j1_bx), 2.0));
    theta_3 = acos((pow(l_eff, 2.0) - pow(l_2, 2.0) - pow(l_3, 2.0))/(-2.0*l_2*l_3)) - EIGEN_PI/2.0;

    // Part 3: theta_2
    float gamma = acos((pow(l_3, 2.0) - pow(l_eff, 2.0) - pow(l_2, 2.0))/(-2.0*l_eff*l_2));
    theta_2 = atan2(p_x - d_x - d_j2_j1_bx, l_b) - gamma + EIGEN_PI/2.0;

    joint_angles(0) = theta_1;
    joint_angles(1) = theta_2;
    joint_angles(2) = theta_3;

    return joint_angles;
}

void Leg::moveLegToJointAngles(Eigen::Vector3f joint_angles_cmd) {
    /*
    DESCRIPTION:
    Moves all joints in the leg to the angles specified in joint_angles.

    ARGUMENTS:
    + joint_angles_cmd: The commanded joint angles. {coxa: theta_1, femur: theta_2, tibia: theta_3}.
    */

    joint_angles = joint_angles_cmd; // Update attribute
    coxa_servo.write(degrees(joint_angles(0)));
    femur_servo.write(degrees(joint_angles(1)));
    tibia_servo.write(degrees(joint_angles(2)));
}

void Leg::moveFoot(Eigen::Vector3f foot_pos) {
    /*
    DESCRIPTION:
    Runs calculateIKFoot on given dest and sends a command to the simulator to
    move the foot to the calculated joint angles. Updates own joint angle and
    foot location attributes.

    ARGUMENTS:
    + foot_pos: The destination coordinates [x, y, z] of the foot.
    */

    calculateIKFoot(foot_pos);
    setFootPositionWrtBody(foot_pos);
    moveLegToJointAngles(joint_angles);
}