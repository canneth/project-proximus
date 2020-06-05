
#include "MasterController.h"

#include "MatrixPrinter.h"

using namespace project_namespace;

// CONSTRUCTORS
MasterController::MasterController(IMU& imu_init):
    imu(imu_init),
    gait_controller(GaitController(Gait::TROT, FootTrajectory::SEMICIRCULAR, imu)),
    ticks(0)
{ }

// METHODS
void MasterController::stepOnce(Robot& robot, Command& command) {
    /*
    DESCRIPTION:
    Takes in a Robot object and a Command object and steps the state of the robot one tick forward based
    on inputs from command.
    NOTE: THIS IS THE ONLY FUNCTION WHERE ROBOT ATTRIBUTES ARE CHANGED! It is merely accessed everywhere else.

    ARGUMENTS:
    + robot: The robot to control and command.
    + command: It contains the input commands into the robot.
    */

    if (command.getMode() == Mode::REST) {
        ticks = 0;
        Eigen::Matrix<float, 3, 4> new_foot_positions_wrt_body; // Initialise
        // Calculate neutral stance positions for each leg
        new_foot_positions_wrt_body.col(0) = (
            Eigen::Vector3f()
            <<  command.getStancePolygonLength()/2.0,
                command.getStancePolygonWidth()/2.0,
                -1.0*command.getStanceHeight()
        ).finished();
        new_foot_positions_wrt_body.col(1) = (
            Eigen::Vector3f()
            <<  command.getStancePolygonLength()/2.0,
                -1.0*command.getStancePolygonWidth()/2.0,
                -1.0*command.getStanceHeight()
        ).finished();
        new_foot_positions_wrt_body.col(2) = (
            Eigen::Vector3f()
            <<  -1.0*command.getStancePolygonLength()/2.0,
                command.getStancePolygonWidth()/2.0,
                -1.0*command.getStanceHeight()
        ).finished();
        new_foot_positions_wrt_body.col(3) = (
            Eigen::Vector3f()
            <<  -1.0*command.getStancePolygonLength()/2.0,
                -1.0*command.getStancePolygonWidth()/2.0,
                -1.0*command.getStanceHeight()
        ).finished();
        // Track foot trajectory without body rpy
        robot.setFootPositionsWrtBodyAssumingNoBodyRPY(new_foot_positions_wrt_body);
        // Desired body orientation matrix
        Eigen::Matrix<float, 3, 3> body_rpy_matrix;
        body_rpy_matrix =
            MyMath::eulerToMatrix(
                command.getBodyRoll(),
                command.getBodyPitch(),
                command.getBodyYaw()
            );
        // Apply body rpy
        new_foot_positions_wrt_body = body_rpy_matrix.transpose()*new_foot_positions_wrt_body;
        // Move feet to calculated positions
        robot.moveAllFeet(new_foot_positions_wrt_body); // Foot positions, joint angles in Legs and Robot are internally updated.
        // Update robot attributes
        robot.setStancePolygonLength(command.getStancePolygonLength()); // Resting foot locations are updated internally
        robot.setStancePolygonWidth(command.getStancePolygonWidth()); // Resting foot locations are updated internally
        robot.setStanceHeight(command.getStanceHeight()); // Resting foot locations are updated internally
        robot.setBodyRoll(command.getBodyRoll());
        robot.setBodyPitch(command.getBodyPitch());
        robot.setBodyYaw(command.getBodyYaw());

    } else if (command.getMode() == Mode::TROT) {
        Eigen::Matrix<float, 3, 4> new_foot_positions_wrt_body; // Initialise
        new_foot_positions_wrt_body = gait_controller.calculateAllNewFootPositions(robot, command, ticks);
        // Update robot contact pattern
        robot.setContactPattern(gait_controller.calculateContactPattern(ticks));
        // Track foot trajectory without body rpy
        robot.setFootPositionsWrtBodyAssumingNoBodyRPY(new_foot_positions_wrt_body);
        // Desired body orientation matrix
        Eigen::Matrix<float, 3, 3> body_rpy_matrix;
        body_rpy_matrix =
            MyMath::eulerToMatrix(
                command.getBodyRoll(),
                command.getBodyPitch(),
                command.getBodyYaw()
            );
        // Apply body rpy
        new_foot_positions_wrt_body = body_rpy_matrix.transpose()*new_foot_positions_wrt_body;
        // Move feet to calculated positions
        robot.moveAllFeet(new_foot_positions_wrt_body); // Foot positions (true), joint angles in Legs and Robot are internally updated.
        // Update robot attributes
        robot.setStancePolygonLength(command.getStancePolygonLength()); // Resting foot locations are updated internally
        robot.setStancePolygonWidth(command.getStancePolygonWidth()); // Resting foot locations are updated internally
        robot.setStanceHeight(command.getStanceHeight()); // Resting foot locations are updated internally
        robot.setBodyRoll(command.getBodyRoll());
        robot.setBodyPitch(command.getBodyPitch());
        robot.setBodyYaw(command.getBodyYaw());

        ticks += 1;
    }
}