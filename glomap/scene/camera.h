#pragma once

#include "glomap/types.h"

#include <colmap/scene/camera.h>
#include <colmap/sensor/models.h>
#include <colmap/util/logging.h>

#include <PoseLib/misc/colmap_models.h>

namespace glomap {

struct Camera : public colmap::Camera {
  Camera() : colmap::Camera() {}
  Camera(const colmap::Camera& camera) : colmap::Camera(camera) {}

  Camera& operator=(const colmap::Camera& camera) {
    *this = Camera(camera);
    return *this;
  }

  bool has_refined_focal_length = false;

  // Returns true if this is a spherical camera model (e.g., EQUIRECTANGULAR)
  // Spherical cameras do not have meaningful focal length or principal point
  inline bool IsSpherical() const;

  inline double Focal() const;
  inline Eigen::Vector2d PrincipalPoint() const;
  inline Eigen::Matrix3d GetK() const;
};

bool Camera::IsSpherical() const {
  // Check model_id for EQUIRECTANGULAR (most robust)
  // Also check model name as fallback for compatibility
  return model_id == colmap::CameraModelId::kEquirectangular ||
         ModelName() == "EQUIRECTANGULAR" || ModelName() == "SPHERICAL";
}

double Camera::Focal() const {
  // Spherical cameras don't have a meaningful focal length
  // Return a dummy value based on image dimensions for compatibility
  if (IsSpherical()) {
    constexpr double kPi = 3.141592653589793238462643383279502884L;
    return static_cast<double>(std::max(width, height)) / kPi;
  }
  return (FocalLengthX() + FocalLengthY()) / 2.0;
}

Eigen::Vector2d Camera::PrincipalPoint() const {
  // Spherical cameras don't have a principal point in the traditional sense
  // Return the image center for compatibility
  if (IsSpherical()) {
    return Eigen::Vector2d(width / 2.0, height / 2.0);
  }
  return Eigen::Vector2d(PrincipalPointX(), PrincipalPointY());
}

Eigen::Matrix3d Camera::GetK() const {
  // Spherical cameras don't have a 3x3 intrinsic matrix
  // Return identity-like matrix for compatibility (should not be used)
  if (IsSpherical()) {
    LOG(WARNING) << "GetK() called on spherical camera - intrinsic matrix is "
                    "not meaningful for equirectangular cameras";
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 2) = width / 2.0;
    K(1, 2) = height / 2.0;
    return K;
  }
  Eigen::Matrix3d K;
  K << FocalLengthX(), 0, PrincipalPointX(), 0, FocalLengthY(),
      PrincipalPointY(), 0, 0, 1;
  return K;
}

inline poselib::Camera ColmapCameraToPoseLibCamera(const Camera& camera) {
  poselib::Camera pose_lib_camera(
      camera.ModelName(), camera.params, camera.width, camera.height);
  return pose_lib_camera;
}

}  // namespace glomap
