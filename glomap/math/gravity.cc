#include "gravity.h"

#include "glomap/scene/types_sfm.h"

#include <Eigen/QR>

namespace glomap {

// The second col of R_align is gravity direction
Eigen::Matrix3d GetAlignRot(const Eigen::Vector3d& gravity) {
  Eigen::Matrix3d R;
  Eigen::Vector3d v = gravity.normalized();
  R.col(1) = v;

  Eigen::Matrix3d Q = v.householderQr().householderQ();
  Eigen::Matrix<double, 3, 2> N = Q.rightCols(2);
  R.col(0) = N.col(0);
  R.col(2) = N.col(1);
  if (R.determinant() < 0) {
    R.col(2) = -R.col(2);
  }
  return R;
}

double RotUpToAngle(const Eigen::Matrix3d& R_up) {
  Eigen::Quaternion<double> q_pitch(R_up);
  double theta_half = std::atan2(q_pitch.y(), q_pitch.w());
  return theta_half * 2;
}

Eigen::Matrix3d AngleToRotUp(double angle) {
  Eigen::Quaternion<double> q_pitch;
  q_pitch.coeffs() << 0, std::sin(angle / 2), 0, std::cos(angle / 2);
  return q_pitch.toRotationMatrix();
}
}  // namespace glomap