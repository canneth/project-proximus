
#ifndef COMMAND_H
#define COMMAND_H

#include <Eigen.h>

#include "GlobalConstants.h"

namespace project_namespace {

    class Command {
        private:
            float stance_polygon_length;
            float stance_polygon_width;
            float stance_height;

            float body_roll;
            float body_pitch;
            float body_yaw;

            Eigen::Vector3f body_velocity;
            float gait_yaw_speed;
            float swing_height;

            Mode mode;
        protected:
        public:
            // CONSTRUCTORS
            Command();

            // GETTERS
            float getStancePolygonLength();
            float getStancePolygonWidth();
            float getStanceHeight();

            float getBodyRoll();
            float getBodyPitch();
            float getBodyYaw();

            Eigen::Vector3f getBodyVelocity();
            float getGaitYawSpeed();
            float getSwingHeight();

            Mode getMode();

            // SETTERS
            void setStancePolygonLength(float stance_polygon_length_arg);
            void setStancePolygonWidth(float stance_polygon_width_arg);
            void setStanceHeight(float stance_height_arg);

            void setBodyRoll(float body_roll_arg);
            void setBodyPitch(float body_pitch_arg);
            void setBodyYaw(float body_yaw_arg);

            void setBodyVelocity(Eigen::Vector3f body_velocity_arg);
            void setGaitYawSpeed(float gait_yaw_speed_arg);
            void setSwingHeight(float swing_height_arg);

            void setMode(Mode mode_arg);
            
    };
}
#endif