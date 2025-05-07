#include "rig_global_rotation_averaging.h"
// #include "global_rotation_averaging.h"

#include "glomap/math/l1_solver.h"
#include "glomap/math/rigid3d.h"
#include "glomap/math/tree.h"

#include <iostream>
#include <queue>

namespace glomap {

bool RigRotationEstimator::EstimateRotations(
    const ViewGraph& view_graph,
    const std::vector<CameraRig>& camera_rigs,
    std::unordered_map<image_t, Image>& images) {
  // Initialize the rotation from maximum spanning tree
  if (!options_.skip_initialization && !options_.use_gravity) {
    InitializeFromMaximumSpanningTree(view_graph, images);
  }

  // Set up the linear system
  SetupLinearSystem(view_graph, camera_rigs, images);

  // Solve the linear system for L1 norm optimization
  if (options_.max_num_l1_iterations > 0) {
    if (!SolveL1Regression(view_graph, images)) {
      return false;
    }
  }

  // Solve the linear system for IRLS optimization
  if (options_.max_num_irls_iterations > 0) {
    if (!SolveIRLS(view_graph, images)) {
      return false;
    }
  }

  ConvertResults(camera_rigs, images);

  return true;
}

// TODO: add the gravity aligned version
// TODO: refine the code
void RigRotationEstimator::SetupLinearSystem(
    const ViewGraph& view_graph,
    const std::vector<CameraRig>& camera_rigs,
    std::unordered_map<image_t, Image>& images) {
  // Clear all the structures
  sparse_matrix_.resize(0, 0);
  tangent_space_step_.resize(0);
  tangent_space_residual_.resize(0);
  rotation_estimated_.resize(0);
  image_id_to_idx_.clear();
  rel_temp_info_.clear();

  // Initialize the structures for estimated rotation
  image_id_to_idx_.reserve(images.size());
  rotation_estimated_.resize(
      3 * images.size());  // allocate more memory than needed
  image_t num_dof = 0;
  rig_is_registered_.reserve(camera_rigs.size());
  for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); ++idx_rig) {
    const auto& camera_rig = camera_rigs.at(idx_rig);
    rig_is_registered_.emplace_back();
    auto& rig_is_registered = rig_is_registered_.back();
    const size_t num_snapshots = camera_rig.NumSnapshots();

    for (size_t idx_snapshot = 0; idx_snapshot < num_snapshots;
         ++idx_snapshot) {
      bool is_registered = false;
      const auto& snapshot = camera_rig.Snapshots()[idx_snapshot];
      for (const auto image_id : snapshot) {
        const auto& image = images.at(image_id);
        if (images.find(image_id) == images.end()) continue;
        if (!images[image_id].is_registered) continue;
        image_id_to_camera_rig_index_.emplace(image_id, idx_rig);
        image_id_to_idx_[image_id] = num_dof;
        is_registered = true;
      }
      rig_is_registered.emplace_back(is_registered);

      camera_rig.ComputeRigFromWorld(idx_snapshot, images);
      if (is_registered) {
        num_dof += 3;
      }
    }
  }

  // If no cameras are set to be fixed, then take the first camera
  if (fixed_camera_id_ == -1) {
    for (auto& [image_id, image] : images) {
      if (!image.is_registered) continue;
      fixed_camera_id_ = image_id;
      // fixed_camera_rotation_ = Rigid3dToAngleAxis(image.cam_from_world);

      camera_t camera_id = images[image_id].camera_id;
      if (image_id_to_camera_rig_index_.find(image_id) ==
          image_id_to_camera_rig_index_.end())
        fixed_camera_rotation_ = Rigid3dToAngleAxis(image.cam_from_world);
      else
        fixed_camera_rotation_ = Rigid3dToAngleAxis(
            colmap::Inverse(
                camera_rigs[image_id_to_camera_rig_index_[image_id]].CamFromRig(
                    camera_id)) *
            image.cam_from_world);
      break;
    }
  }

  rotation_estimated_.conservativeResize(num_dof);

  // Prepare the relative information
  int counter = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;

    image_t image_id1 = image_pair.image_id1;
    image_t image_id2 = image_pair.image_id2;

    Rigid3d cam1_from_rig1, cam2_from_rig2;
    int idx_rig1 = (image_id_to_camera_rig_index_.find(image_id1) !=
                    image_id_to_camera_rig_index_.end())
                       ? image_id_to_camera_rig_index_[image_id1]
                       : -1;
    int idx_rig2 = (image_id_to_camera_rig_index_.find(image_id2) !=
                    image_id_to_camera_rig_index_.end())
                       ? image_id_to_camera_rig_index_[image_id2]
                       : -1;

    int vector_idx1 = image_id_to_idx_[image_id1];
    int vector_idx2 = image_id_to_idx_[image_id2];

    if (vector_idx1 == vector_idx2) {
      // Skip the self loop
      continue;
    }

    if (idx_rig1 != -1) {
      camera_t camera_id = images[image_id1].camera_id;
      cam1_from_rig1 = camera_rigs[idx_rig1].CamFromRig(camera_id);
    }
    if (idx_rig2 != -1) {
      camera_t camera_id = images[image_id2].camera_id;
      cam2_from_rig2 = camera_rigs[idx_rig2].CamFromRig(camera_id);
    }

    rel_temp_info_[pair_id].R_rel =
        (cam2_from_rig2.rotation.inverse() *
         image_pair.cam2_from_cam1.rotation * cam1_from_rig1.rotation)
            .toRotationMatrix();

    // Align the relative rotation to the gravity
    // TODO: version with gravity is not debugged
    if (options_.use_gravity) {
      if (images[image_id1].gravity_info.has_gravity) {
        rel_temp_info_[pair_id].R_rel =
            rel_temp_info_[pair_id].R_rel *
            images[image_id1].gravity_info.GetRAlign();
      }

      if (images[image_id2].gravity_info.has_gravity) {
        rel_temp_info_[pair_id].R_rel =
            images[image_id2].gravity_info.GetRAlign().transpose() *
            rel_temp_info_[pair_id].R_rel;
      }
    }

    if (options_.use_gravity && images[image_id1].gravity_info.has_gravity &&
        images[image_id2].gravity_info.has_gravity) {
      counter++;
      Eigen::Vector3d aa = RotationToAngleAxis(rel_temp_info_[pair_id].R_rel);
      double error = aa[0] * aa[0] + aa[2] * aa[2];

      // Keep track of the error for x and z axis for gravity-aligned relative
      // pose
      rel_temp_info_[pair_id].xz_error = error;
      rel_temp_info_[pair_id].has_gravity = true;
      rel_temp_info_[pair_id].angle_rel = aa[1];
    } else {
      rel_temp_info_[pair_id].has_gravity = false;
    }
  }

  VLOG(2) << counter << " image pairs are gravity aligned" << std::endl;

  std::vector<Eigen::Triplet<double>> coeffs;
  coeffs.reserve(rel_temp_info_.size() * 6 + 3);

  // Establish linear systems
  size_t curr_pos = 0;
  std::vector<double> weights;
  weights.reserve(3 * view_graph.image_pairs.size());
  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;
    if (rel_temp_info_.find(pair_id) == rel_temp_info_.end()) continue;

    int image_id1 = image_pair.image_id1;
    int image_id2 = image_pair.image_id2;

    int vector_idx1 = image_id_to_idx_[image_id1];
    int vector_idx2 = image_id_to_idx_[image_id2];

    rel_temp_info_[pair_id].index = curr_pos;

    if (rel_temp_info_[pair_id].has_gravity) {
      coeffs.emplace_back(Eigen::Triplet<double>(curr_pos, vector_idx1, -1));
      coeffs.emplace_back(Eigen::Triplet<double>(curr_pos, vector_idx2, 1));
      if (image_pair.weight >= 0)
        weights.emplace_back(image_pair.weight);
      else
        weights.emplace_back(1);
      curr_pos++;
    } else {
      // If it is not gravity aligned, then we need to consider 3 dof
      if (!options_.use_gravity ||
          !images[image_id1].gravity_info.has_gravity) {
        for (int i = 0; i < 3; i++) {
          coeffs.emplace_back(
              Eigen::Triplet<double>(curr_pos + i, vector_idx1 + i, -1));
        }
      } else
        // else, other components are zero, and can be safely ignored
        coeffs.emplace_back(
            Eigen::Triplet<double>(curr_pos + 1, vector_idx1, -1));

      // Similarly for the second componenet
      if (!options_.use_gravity ||
          !images[image_id2].gravity_info.has_gravity) {
        for (int i = 0; i < 3; i++) {
          coeffs.emplace_back(
              Eigen::Triplet<double>(curr_pos + i, vector_idx2 + i, 1));
        }
      } else
        coeffs.emplace_back(
            Eigen::Triplet<double>(curr_pos + 1, vector_idx2, 1));
      for (int i = 0; i < 3; i++) {
        if (image_pair.weight >= 0)
          weights.emplace_back(image_pair.weight);
        else
          weights.emplace_back(1);
      }

      curr_pos += 3;
    }
  }

  // Set some cameras to be fixed
  // if some cameras have gravity, then add a single term constraint
  // Else, change to 3 constriants
  if (options_.use_gravity &&
      images[fixed_camera_id_].gravity_info.has_gravity) {
    coeffs.emplace_back(Eigen::Triplet<double>(
        curr_pos, image_id_to_idx_[fixed_camera_id_], 1));
    weights.emplace_back(1);
    curr_pos++;
  } else {
    for (int i = 0; i < 3; i++) {
      coeffs.emplace_back(Eigen::Triplet<double>(
          curr_pos + i, image_id_to_idx_[fixed_camera_id_] + i, 1));
      weights.emplace_back(1);
    }
    curr_pos += 3;
  }

  // For rig case, we only keep one representative of the rig, so set all other
  // images to be not registered
  for (auto& [image_id, image] : images) {
    image.is_registered = false;
  }

  for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); ++idx_rig) {
    const auto& camera_rig = camera_rigs.at(idx_rig);
    const size_t num_snapshots = camera_rig.NumSnapshots();
    for (size_t idx_snapshot = 0; idx_snapshot < num_snapshots;
         ++idx_snapshot) {
      if (!rig_is_registered_[idx_rig][idx_snapshot]) continue;
      // Set the first camera in the rig to be registered
      const auto& snapshot = camera_rig.Snapshots()[idx_snapshot];
      image_t image_id = snapshot[0];
      images[image_id].is_registered = true;
      // image_id_to_camera_rig_index_[snapshot[0]] = idx_rig;
      // const auto& snapshot = camera_rig.Snapshots()[idx_snapshot];
      // for (const auto image_id : snapshot) {
      //   images[image_id].is_registered = true;
      // }
    }
  }

  sparse_matrix_.resize(curr_pos, num_dof);
  sparse_matrix_.setFromTriplets(coeffs.begin(), coeffs.end());

  // Set up the weight matrix for the linear system
  if (!options_.use_weight) {
    weights_ = Eigen::ArrayXd::Ones(curr_pos);
  } else {
    weights_ = Eigen::ArrayXd(weights.size());
    for (size_t i = 0; i < weights.size(); i++) weights_[i] = weights[i];
  }

  // Initialize x and b
  tangent_space_step_.resize(num_dof);
  tangent_space_residual_.resize(curr_pos);
}

void RigRotationEstimator::ConvertResults(
    const std::vector<CameraRig>& camera_rigs,
    std::unordered_map<image_t, Image>& images) {
  // Convert the final results
  for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); ++idx_rig) {
    const auto& camera_rig = camera_rigs.at(idx_rig);
    const size_t num_snapshots = camera_rig.NumSnapshots();
    for (size_t idx_snapshot = 0; idx_snapshot < num_snapshots;
         ++idx_snapshot) {
      if (!rig_is_registered_[idx_rig][idx_snapshot]) continue;
      for (const auto image_id : camera_rig.Snapshots()[idx_snapshot]) {
        if (images.find(image_id) == images.end()) continue;
        images[image_id].is_registered = true;
        Rigid3d cam_from_rig =
            camera_rig.CamFromRig(images[image_id].camera_id);
        images[image_id].cam_from_world.rotation =
            cam_from_rig.rotation *
            Eigen::Quaterniond(AngleAxisToRotation(
                rotation_estimated_.segment(image_id_to_idx_[image_id], 3)));
      }
    }
  }

  for (auto& [image_id, image] : images) {
    if (image_id_to_idx_.find(image_id) == image_id_to_idx_.end()) {
      continue;
    }
    // If it belongs to some rig, then do not set the camera rotation
    if (image_id_to_camera_rig_index_.find(image_id) !=
        image_id_to_camera_rig_index_.end()) {
      continue;
    }

    if (options_.use_gravity && image.gravity_info.has_gravity) {
      image.cam_from_world.rotation = Eigen::Quaterniond(
          image.gravity_info.GetRAlign() *
          AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id]]));
    } else {
      image.cam_from_world.rotation = Eigen::Quaterniond(AngleAxisToRotation(
          rotation_estimated_.segment(image_id_to_idx_[image_id], 3)));
    }
    // Restore the prior position (t = -Rc = R * R_ori * t_ori = R * t_ori)
    image.cam_from_world.translation =
        (image.cam_from_world.rotation * image.cam_from_world.translation);
  }
}

}  // namespace glomap