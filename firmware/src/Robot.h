
#ifndef ROBOT_H
#define ROBOT_H

namespace project_namespace {
    class Robot {
        private:
            float stance_polygon_length;
            float stance_polygon_width;
            float stance_height;

            float body_velocity;
            float body_roll;
            float body_pitch;
            float body_yaw;
            foot_locations_wrt_body_true; // np.zeros((3, 4)), each column represents a foot
            foot_locations_wrt_body_assuming_no_body_rpy; // np.zeros((3, 4))
            foot_velocities_wrt_body; // np.zeros((3, 4))
            joint_angles; // np.zeros((3, 4))
            contact_pattern; // np.ones((4)), by default, all legs in stance

            // p_b_vpsp = np.zeros((3))
            // virtual_points = np.zeros((4, 2, 3))
            // vpsp_vertices = np.zeros((3, 4))
        protected:
        public:

    }
}
#endif