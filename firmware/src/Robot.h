
#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen.h>

#include "Leg.h"

namespace project_namespace {

    class Robot {
        private:
            float stance_polygon_length;
            float stance_polygon_width;
            float stance_height;
            
            float body_roll;
            float body_pitch;
            float body_yaw;

            Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_true;
            Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_assuming_no_body_rpy;
            Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_at_rest;

            Eigen::Matrix<float, 3, 4> joint_angles;
            Eigen::Vector4i contact_pattern;

        protected:
        public:
            // PUBLIC ATTRIBUTES
            Leg front_left_leg;
            Leg front_right_leg;
            Leg back_left_leg;
            Leg back_right_leg;
            
            // CONSTRUCTORS
            Robot();

            // GETTERS
            float getStancePolygonLength();
            float getStancePolygonWidth();
            float getStanceHeight();
            float getBodyRoll();
            float getBodyPitch();
            float getBodyYaw();
            Eigen::Matrix<float, 3, 4> getFootPositionsWrtBodyTrue();
            Eigen::Matrix<float, 3, 4> getFootPositionsWrtBodyAssumingNoBodyRPY();
            Eigen::Matrix<float, 3, 4> getFootPositionsWrtBodyAtRest();
            Eigen::Matrix<float, 3, 4> getJointAngles();
            Eigen::Vector4i getContactPattern();

            // SETTERS
            void setStancePolygonLength(float stance_polygon_length_arg);
            void setStancePolygonWidth(float stance_polygon_width_arg);
            void setStanceHeight(float stance_height_arg);
            void setBodyRoll(float body_roll_arg);
            void setBodyPitch(float body_pitch_arg);
            void setBodyYaw(float body_yaw_arg);
            void setFootPositionsWrtBodyTrue(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_true_arg);
            void setFootPositionsWrtBodyAssumingNoBodyRPY(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_assuming_no_body_rpy_arg);
            void setFootPositionsWrtBodyAtRest(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body_at_rest_arg);
            void setJointAngles(Eigen::Matrix<float, 3, 4> joint_angles_arg);
            void setContactPattern(Eigen::Vector4i contact_pattern_arg);

            // METHODS
            void moveAllFeet(Eigen::Matrix<float, 3, 4> foot_positions_wrt_body);



    };

}
#endif