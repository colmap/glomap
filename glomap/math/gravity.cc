#include "gravity.h"

#include "glomap/scene/types_sfm.h"
#include "glomap/math/rigid3d.h"

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
  return RotationToAngleAxis(R_up)[1];
}

Eigen::Matrix3d AngleToRotUp(double angle) {
  Eigen::Vector3d aa(0, angle, 0);
  return AngleAxisToRotation(aa);
}
}  // namespace glomap