
#ifndef LEG_H
#define LEG_H

#include <Eigen.h>
#include <PWMServo.h>

namespace project_namespace {
    
    class Leg {
        private:
            // PHYSICAL PARAMS
            float d_x;
            float d_y;
            float d_j2_j1_bx;
            float d_j2_j1_by;
            float l_2;
            float l_3;
            Eigen::Vector3i joint_servo_directions;

            // STATES
            Eigen::Vector3f joint_angles;
            Eigen::Vector3f foot_position_wrt_body;
        protected:
        public:
            // PUBLIC MEMBERS
            PWMServo coxa_servo;
            PWMServo femur_servo;
            PWMServo tibia_servo;
            // CONSTRUCTORS
            Leg(
                float d_x_init,
                float d_y_init,
                float d_j2_j1_bx_init,
                float d_j2_j1_by_init,
                float l_2_init,
                float l_3_init,
                Eigen::Vector3i joint_servo_directions_init
            );
            // GETTERS
            Eigen::Vector3i getJointServoDirections();
            Eigen::Vector3f getJointAngles();
            Eigen::Vector3f getFootPositionWrtBody();
            // SETTERS
            void setJointAngles(Eigen::Vector3f joint_angles_arg);
            void setFootPositionWrtBody(Eigen::Vector3f foot_position_wrt_body_arg);
            // METHODS
            Eigen::Vector3f calculateIKFoot(Eigen::Vector3f foot_pos);
            void moveLegToJointAngles(Eigen::Vector3f joint_angles_cmd);
            void moveFoot(Eigen::Vector3f foot_pos);
    };

}
#endif