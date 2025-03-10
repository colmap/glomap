#include "camera_rig.h"

#include <colmap/util/misc.h>

namespace glomap {

// double CameraRig::ComputeRigFromWorldScale(
//     const std::unordered_map<image_t, Image>& images) {

//   THROW_CHECK_GT(NumSnapshots(), 0);
//   const size_t num_cameras = NumCameras();
//   THROW_CHECK_GT(num_cameras, 0);

//   double rig_from_world_scale = 0;
//   size_t num_dists = 0;
//   std::vector<Eigen::Vector3d> proj_centers_in_rig(num_cameras);
//   std::vector<Eigen::Vector3d> proj_centers_in_world(num_cameras);
//   for (const auto& snapshot : snapshots_) {
//     for (size_t i = 0; i < num_cameras; ++i) {
//         proj_centers_in_rig[i] =
//         colmap::Inverse(CamFromRig(images[snapshot[i]].cam_from_world)).translation;
//         proj_centers_in_world[i] = images[snapshot[i]].Center();
//     }

//     for (size_t i = 0; i < num_cameras; ++i) {
//       for (size_t j = 0; j < i; ++j) {
//         const double rig_dist =
//             (proj_centers_in_rig[i] - proj_centers_in_rig[j]).norm();
//         const double world_dist =
//             (proj_centers_in_world[i] - proj_centers_in_world[j]).norm();
//         const double kMinDist = 1e-6;
//         if (rig_dist > kMinDist && world_dist > kMinDist) {
//           rig_from_world_scale += rig_dist / world_dist;
//           num_dists += 1;
//         }
//       }
//     }
//   }

//   if (num_dists == 0) {
//     return std::numeric_limits<double>::quiet_NaN();
//   }

//   return rig_from_world_scale / num_dists;
// }

// bool CameraRig::ComputeCamsFromRigs(const std::unordered_map<image_t, Image>&
// images) {
//   THROW_CHECK_GT(NumSnapshots(), 0);
//   THROW_CHECK_NE(RefCameraId(), kInvalidCameraId);

//   for (auto& cam_from_rig : cams_from_rigs_) {
//     cam_from_rig.second.translation = Eigen::Vector3d::Zero();
//   }

//   std::unordered_map<camera_t, std::vector<Eigen::Quaterniond>>
//       cam_from_ref_cam_rotations;
//   for (const auto& snapshot : snapshots_) {
//     // Find the image of the reference camera in the current snapshot.
//     const Image* ref_image = nullptr;
//     for (const auto image_id : snapshot) {
//     //   const auto& image = reconstruction.Image(image_id);
//       auto image = images[image_id];
//       if (image.camera_id == RefCameraId()) {
//         ref_image = &image;
//         break;
//       }
//     }

//     const Rigid3d world_from_ref_cam =
//         Inverse(THROW_CHECK_NOTNULL(ref_image)->cam_from_world);

//     // Compute the relative poses from all cameras in the current snapshot to
//     // the reference camera.
//     for (const auto image_id : snapshot) {
//       const auto& image = reconstruction.Image(image_id);
//       if (image.CameraId() != RefCameraId()) {
//         const Rigid3d cam_from_ref_cam =
//             image.CamFromWorld() * world_from_ref_cam;
//         cam_from_ref_cam_rotations[image.CameraId()].push_back(
//             cam_from_ref_cam.rotation);
//         CamFromRig(image.CameraId()).translation +=
//             cam_from_ref_cam.translation;
//       }
//     }
//   }

//   cams_from_rigs_.at(RefCameraId()) = Rigid3d();

//   // Compute the average relative poses.
//   for (auto& cam_from_rig : cams_from_rigs_) {
//     if (cam_from_rig.first != RefCameraId()) {
//       if (cam_from_ref_cam_rotations.count(cam_from_rig.first) == 0) {
//         LOG(INFO) << "Need at least one snapshot with an image of camera "
//                   << cam_from_rig.first << " and the reference camera "
//                   << RefCameraId()
//                   << " to compute its relative pose in the camera rig";
//         return false;
//       }
//       const std::vector<Eigen::Quaterniond>& cam_from_rig_rotations =
//           cam_from_ref_cam_rotations.at(cam_from_rig.first);
//       const std::vector<double> weights(cam_from_rig_rotations.size(), 1.0);
//       cam_from_rig.second.rotation =
//           colmap::AverageQuaternions(cam_from_rig_rotations, weights);
//       cam_from_rig.second.translation /= cam_from_rig_rotations.size();
//     }
//   }
//   return true;
// }

Rigid3d CameraRig::ComputeRigFromWorld(
    size_t snapshot_idx,
    const std::unordered_map<image_t, Image>& images) const {
  // const auto& snapshot = snapshots_.at(snapshot_idx);
  const auto& snapshot = Snapshots()[snapshot_idx];

  std::vector<Eigen::Quaterniond> rig_from_world_rotations;
  rig_from_world_rotations.reserve(snapshot.size());
  Eigen::Vector3d rig_from_world_translations = Eigen::Vector3d::Zero();
  for (const auto image_id : snapshot) {
    const auto& image = images.at(image_id);
    const Rigid3d rig_from_world =
        colmap::Inverse(CamFromRig(image.camera_id)) * image.cam_from_world;
    rig_from_world_rotations.push_back(rig_from_world.rotation);
    rig_from_world_translations += rig_from_world.translation;
  }

  const std::vector<double> rotation_weights(snapshot.size(), 1);
  return Rigid3d(
      colmap::AverageQuaternions(rig_from_world_rotations, rotation_weights),
      rig_from_world_translations /= snapshot.size());
}
}  // namespace glomap