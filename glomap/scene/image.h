#pragma once

#include "glomap/math/gravity.h"
#include "glomap/scene/rigid3d.h"
#include "glomap/scene/types.h"
#include "glomap/types.h"

namespace glomap {

struct GravityInfo {
  // Whether the gravity information is available
  bool has_gravity = false;

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

  // The pose of the image, defined as the transformation from world to camera.
  Rigid3d cam_from_world;

  // Gravity information
  GravityInfo gravity_info;

  // Features
  std::vector<Eigen::Vector2d> features;
  std::vector<Eigen::Vector3d>
      features_undist;  // store the normalized features, can be obtained by
                        // calling UndistortImages

  // Methods
  inline Eigen::Vector3d Center() const;
  inline void SetGravityInfo(const Eigen::Vector3d& g);
};

Eigen::Vector3d Image::Center() const {
  return cam_from_world.rotation.inverse() * -cam_from_world.translation;
}

void Image::SetGravityInfo(const Eigen::Vector3d& g) {
  gravity_info.gravity = g;
  gravity_info.R_align = GetAlignRot(g);
  gravity_info.has_gravity = true;
}
}  // namespace glomap
