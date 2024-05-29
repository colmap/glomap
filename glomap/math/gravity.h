#ifndef GLOMAP_SCENE_GRAVITY_H_
#define GLOMAP_SCENE_GRAVITY_H_

#include <Eigen/Core>

namespace glomap {

// Get the aligment rotation matrix by QR decomposition
// The second col of output is gravity direction
Eigen::Matrix3d GetAlignRot(const Eigen::Vector3d& gravity);

// Get the rotation angle for an upright rotation matrix
double RotUpToAngle(const Eigen::Matrix3d& R_up);

// Get the upright rotation matrix from a rotation angle
Eigen::Matrix3d AngleToRotUp(double angle);

}; // glomap

#endif // GLOMAP_SCENE_GRAVITY_H_
