#pragma once

#include "glomap/math/gravity.h"
#include "glomap/scene/types.h"
#include "glomap/types.h"

#include <colmap/scene/frame.h>

namespace glomap {

struct GravityInfo {
 public:
  // Whether the gravity information is available
  bool has_gravity = false;

  const Eigen::Matrix3d& GetRAlign() const { return R_align_; }

  inline void SetGravity(const Eigen::Vector3d& g);
  inline Eigen::Vector3d GetGravity() const { return gravity_; };

 private:
  // Direction of the gravity
  Eigen::Vector3d gravity_;

  // Alignment matrix, the second column is the gravity direction
  Eigen::Matrix3d R_align_;
};

struct Frame : public colmap::Frame {
  Frame() : colmap::Frame() {}
  Frame(const colmap::Frame& frame) : colmap::Frame(frame) {}

  // Gravity information
  GravityInfo gravity_info;

  // Easy way to check if the image has gravity information
  inline bool HasGravity() const;
};

bool Frame::HasGravity() const { return gravity_info.has_gravity; }

void GravityInfo::SetGravity(const Eigen::Vector3d& g) {
  gravity_ = g;
  R_align_ = GetAlignRot(g);
  has_gravity = true;
}
}  // namespace glomap