#include "reconstruction_aligner.h"

#include <colmap/estimators/similarity_transform.h>
#include <colmap/geometry/pose.h>
#include <colmap/geometry/sim3.h>

namespace glomap {
bool AlignReconstructionToPosePositionPriors(
    const std::unordered_map<image_t, colmap::PosePrior>& pose_priors,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks,
    const colmap::RANSACOptions& ransac_options) {
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
  if (ransac_options.max_error > 0) {
    auto report = colmap::EstimateSim3dRobust(
        src_locations, tgt_locations, ransac_options, tform);
    success = report.success;
  } else {
    success = colmap::EstimateSim3d(src_locations, tgt_locations, tform);
  }

  // Apply similarity transformation
  if (success) {
    for (auto& [image_id, image] : images) {
      if (image.frame_ptr->is_registered) {
        image.frame_ptr->RigFromWorld() =
            TransformCameraWorld(tform, image.frame_ptr->RigFromWorld());
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