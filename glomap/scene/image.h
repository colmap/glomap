#pragma once

#include "glomap/math/gravity.h"
#include "glomap/scene/types.h"
#include "glomap/types.h"

namespace glomap {

struct GravityInfo {
 public:
  // Whether the gravity information is available
  bool has_gravity = false;

  const Eigen::Matrix3d& GetRAlign() const { return R_align; }

  inline void SetGravity(const Eigen::Vector3d& g);
  inline Eigen::Vector3d GetGravity() const { return gravity; };

 private:
  // Direction of the gravity
  Eigen::Vector3d gravity;

  // Alignment matrix, the second column is the gravity direction
  Eigen::Matrix3d R_align;
};

struct Image {
  Image() : image_id(-1), file_name("") {}
  Image(image_t img_id, camera_t cam_id, std::string file_name)
      : image_id(img_id), file_name(file_name), camera_id(cam_id) {}

  // Basic information
  // image_id, file_name need to be specified at construction time
  const image_t image_id;
  const std::string file_name;

  // The id of the camera
  camera_t camera_id;

  // whether the image is within the largest connected component
  bool is_registered = false;
  int cluster_id = -1;

  // The pose of the image, defined as the transformation from world to camera.
  Rigid3d cam_from_world;

  // Gravity information
  GravityInfo gravity_info;

  // Distorted feature points in pixels.
  std::vector<Eigen::Vector2d> features;
  // Normalized feature rays, can be obtained by calling UndistortImages.
  std::vector<Eigen::Vector3d> features_undist;

  // Methods
  inline Eigen::Vector3d Center() const;
};

Eigen::Vector3d Image::Center() const {
  return cam_from_world.rotation.inverse() * -cam_from_world.translation;
}

void GravityInfo::SetGravity(const Eigen::Vector3d& g) {
  gravity = g;
  R_align = GetAlignRot(g);
  has_gravity = true;
}

}  // namespace glomap
