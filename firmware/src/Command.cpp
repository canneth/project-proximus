
#include "Command.h"

using namespace project_namespace;


// CONSTRUCTORS
Command::Command():
    stance_polygon_length(0.2),
    stance_polygon_width(0.09),
    stance_height(0.2),

    body_roll(0.0),
    body_pitch(0.0),
    body_yaw(0.0),

    body_velocity(Eigen::Vector3f::Zero()),
    gait_yaw_speed(0.0),
    swing_height(0.1),

    mode(Mode::REST)
{ }

// GETTERS
float Command::getStancePolygonLength() {
    return stance_polygon_length;
}
float Command::getStancePolygonWidth() {
    return stance_polygon_width;
}
float Command::getStanceHeight() {
    return stance_height;
}
float Command::getBodyRoll() {
    return body_roll;
}
float Command::getBodyPitch() {
    return body_pitch;
}
float Command::getBodyYaw() {
    return body_yaw;
}
Eigen::Vector3f Command::getBodyVelocity() {
    return body_velocity;
}
float Command::getGaitYawSpeed() {
    return gait_yaw_speed;
}
float Command::getSwingHeight() {
    return swing_height;
}
Mode Command::getMode() {
    return mode;
}

// SETTERS
void Command::setStancePolygonLength(float stance_polygon_length_arg) {
    stance_polygon_length = stance_polygon_length_arg;
}
void Command::setStancePolygonWidth(float stance_polygon_width_arg) {
    stance_polygon_width = stance_polygon_width_arg;
}
void Command::setStanceHeight(float stance_height_arg) {
    stance_height = stance_height_arg;
}
void Command::setBodyRoll(float body_roll_arg) {
    body_roll = body_roll_arg;
}
void Command::setBodyPitch(float body_pitch_arg) {
    body_pitch = body_pitch_arg;
}
void Command::setBodyYaw(float body_yaw_arg) {
    body_yaw = body_yaw_arg;
}
void Command::setBodyVelocity(Eigen::Vector3f body_velocity_arg) {
    body_velocity = body_velocity_arg;
}
void Command::setGaitYawSpeed(float gait_yaw_speed_arg) {
    gait_yaw_speed = gait_yaw_speed_arg;
}
void Command::setSwingHeight(float swing_height_arg) {
    swing_height = swing_height_arg;
}
void Command::setMode(Mode mode_arg) {
    mode = mode_arg;
}

