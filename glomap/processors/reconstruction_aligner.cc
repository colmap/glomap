#include "reconstruction_aligner.h"

#include <colmap/estimators/similarity_transform.h>
#include <colmap/geometry/pose.h>
#include <colmap/geometry/sim3.h>

namespace glomap {
bool AlignReconstructionToPosePositionPriors(
    const std::unordered_map<image_t, colmap::PosePrior>& pose_priors,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks,
    double max_error) {
  const int num_images = images.size();

  // Robust estimate transformation from image center to position prior
  std::vector<Eigen::Vector3d> src_locations;
  std::vector<Eigen::Vector3d> tgt_locations;
  src_locations.reserve(num_images);
  tgt_locations.reserve(num_images);
  for (const auto& [image_id, image] : images) {
    src_locations.push_back(image.Center());
    const colmap::PosePrior& pose_prior = pose_priors.at(image_id);
    if (pose_prior.coordinate_system !=
        colmap::PosePrior::CoordinateSystem::CARTESIAN) {
      LOG(WARNING) << "Could not align to non-Cartesian pose priors.";
      return false;
    }
    tgt_locations.emplace_back(pose_prior.position);
  }

  colmap::Sim3d tform;
  bool success = false;
  if (max_error > 0) {
    colmap::RANSACOptions options;
    options.max_error = max_error;
    auto report = colmap::EstimateSim3dRobust(
        src_locations, tgt_locations, options, tform);
    success = report.success;
  } else {
    success = colmap::EstimateSim3d(src_locations, tgt_locations, tform);
  }

  // Apply similarity transformation
  if (success) {
    for (auto& [image_id, image] : images) {
      if (image.is_registered) {
        image.cam_from_world =
            TransformCameraWorld(tform, image.cam_from_world);
      }
    }

    for (auto& [_, track] : tracks) {
      track.xyz = tform * track.xyz;
    }
  } else {
    LOG(WARNING) << "Could not allign reconstruction to pose priors.";
  }
  return success;
}
}  // namespace glomap