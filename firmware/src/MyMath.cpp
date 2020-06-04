
#include "MyMath.h"

using namespace project_namespace;


Eigen::Matrix<float, 3, 3> MyMath::eulerToMatrix(float x, float y, float z) {
    /*
    DESCRIPTION:
    Calculates the rotation matrix from roll, pitch and yaw angles x, y, z.
    This follows the convention of sxyz == rzyx, (as opposed to rxyz).
    s == extrinsic, r == intrinsic.
    In other words, this calculates the rotation matrix from R(z)R(y)R(x).

    ARGUMENTS:
    + x: The intrinsic roll angle.
    + y: The intrinsic pitch angle.
    + z: The intrinsic yaw angle.

    RETURNS:
    + rot_mat: The rotation matrix.
    */

    Eigen::Matrix<float, 3, 3> rot_mat;
    rot_mat =
        Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(y, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX())
    ;
    return rot_mat;
}