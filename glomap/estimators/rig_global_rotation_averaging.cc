#include "rig_global_rotation_averaging.h"

#include "glomap/math/l1_solver.h"
#include "glomap/math/rigid3d.h"
#include "glomap/math/tree.h"

#include <iostream>
#include <queue>

#include "colmap/geometry/pose.h"

namespace glomap {
namespace {
double RelAngleError(double angle_12, double angle_1, double angle_2) {
  double est = (angle_2 - angle_1) - angle_12;

  while (est >= EIGEN_PI) est -= TWO_PI;

  while (est < -EIGEN_PI) est += TWO_PI;

  // Inject random noise if the angle is too close to the boundary to break the
  // possible balance at the local minima
  if (est > EIGEN_PI - 0.01 || est < -EIGEN_PI + 0.01) {
    if (est < 0)
      est += (rand() % 1000) / 1000.0 * 0.01;
    else
      est -= (rand() % 1000) / 1000.0 * 0.01;
  }

  return est;
}
}  // namespace

bool RigRotationEstimator::EstimateRotations(
    const ViewGraph& view_graph,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  // TODO: change this part as well
  // Initialize the rotation from maximum spanning tree
  if (!options_.skip_initialization && !options_.use_gravity) {
    InitializeFromMaximumSpanningTree(view_graph, rigs, frames, images);
    // return true;  // Skip the rest of the process
  }

  // Set up the linear system
  SetupLinearSystem(view_graph, rigs, frames, images);

  // Solve the linear system for L1 norm optimization
  if (options_.max_num_l1_iterations > 0) {
    if (!SolveL1Regression(view_graph, frames, images)) {
      return false;
    }
  }

  // Solve the linear system for IRLS optimization
  if (options_.max_num_irls_iterations > 0) {
    if (!SolveIRLS(view_graph, frames, images)) {
      return false;
    }
  }

  ConvertResults(rigs, frames, images);

  return true;
}

void RigRotationEstimator::InitializeFromMaximumSpanningTree(
    const ViewGraph& view_graph,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  // Here, we assume that largest connected component is already retrieved, so
  // we do not need to do that again compute maximum spanning tree.
  std::unordered_map<image_t, image_t> parents;
  image_t root = MaximumSpanningTree(view_graph, images, parents, INLIER_NUM);

  // Iterate through the tree to initialize the rotation
  // Establish child info
  std::unordered_map<image_t, std::vector<image_t>> children;
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    children.insert(std::make_pair(image_id, std::vector<image_t>()));
  }
  for (auto& [child, parent] : parents) {
    if (root == child) continue;
    children[parent].emplace_back(child);
  }

  std::queue<image_t> indexes;
  indexes.push(root);

  std::unordered_map<image_t, Rigid3d> cam_from_worlds;
  while (!indexes.empty()) {
    image_t curr = indexes.front();
    indexes.pop();

    // Add all children into the tree
    for (auto& child : children[curr]) indexes.push(child);
    // If it is root, then fix it to be the original estimation
    if (curr == root) continue;

    // Directly use the relative pose for estimation rotation
    const ImagePair& image_pair = view_graph.image_pairs.at(
        ImagePair::ImagePairToPairId(curr, parents[curr]));
    if (image_pair.image_id1 == curr) {
      // 1_R_w = 2_R_1^T * 2_R_w
      // cam_from_worlds[curr].rotation =
      //     image_pair.cam2_from_cam1.rotation.inverse() *
      //     cam_from_worlds[parents[curr]].rotation;
      cam_from_worlds[curr].rotation =
          (Inverse(image_pair.cam2_from_cam1) * cam_from_worlds[parents[curr]])
              .rotation;
    } else {
      // 2_R_w = 2_R_1 * 1_R_w
      // images[curr].cam_from_world.rotation =
      //     (image_pair.cam2_from_cam1 * images[parents[curr]].cam_from_world)
      //         .rotation;
      cam_from_worlds[curr].rotation =
          (image_pair.cam2_from_cam1 * cam_from_worlds[parents[curr]]).rotation;
    }
  }

  std::unordered_map<camera_t, rig_t> camera_id_to_rig_id;
  for (auto& [rig_id, rig] : rigs) {
    for (auto& [sensor_id, sensor] : rig.Sensors()) {
      if (sensor_id.type != SensorType::CAMERA) continue;
      camera_id_to_rig_id[sensor_id.id] = rig_id;
    }
  }

  std::unordered_map<camera_t, std::vector<Eigen::Quaterniond>>
      cam_from_ref_cam_rotations;

  std::unordered_map<frame_t, image_t> frame_to_ref_image_id;
  for (auto& [frame_id, frame] : frames) {
    // First, figure out the reference camera in the frame
    image_t ref_img_id = -1;
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      const auto& image = images.at(image_id);
      if (!image.is_registered) continue;

      if (image.camera_id == frame.RigPtr()->RefSensorId().id) {
        ref_img_id = image_id;
        frame_to_ref_image_id[frame_id] = ref_img_id;
        break;
      }
    }

    // If the reference image is not found, then skip the frame
    if (ref_img_id == -1) {
      continue;
    }

    // Then, collect the rotations from the cameras to the reference camera
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      const auto& image = images.at(image_id);
      if (!image.is_registered) continue;

      Rig* rig_ptr = frame.RigPtr();

      // If the camera is a reference camera, then skip it
      if (image.camera_id == rig_ptr->RefSensorId().id) continue;

      if (rig_ptr
              ->MaybeSensorFromRig(
                  sensor_t(SensorType::CAMERA, image.camera_id))
              .has_value())
        continue;

      if (cam_from_ref_cam_rotations.find(image.camera_id) ==
          cam_from_ref_cam_rotations.end())
        cam_from_ref_cam_rotations[image.camera_id] =
            std::vector<Eigen::Quaterniond>();

      // Set the rotation from the camera to the world
      cam_from_ref_cam_rotations[image.camera_id].push_back(
          cam_from_worlds[image_id].rotation *
          cam_from_worlds[ref_img_id].rotation.inverse());
    }
  }

  Eigen::Vector3d nan_translation;
  nan_translation.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Use the average of the rotations to set the rotation from the camera
  // std::unordered_map<camera_t, Eigen::Quaterniond> cam_from_ref_cam_rigs;
  for (auto& [camera_id, cam_from_ref_cam_rotations_i] :
       cam_from_ref_cam_rotations) {
    const std::vector<double> weights(cam_from_ref_cam_rotations_i.size(), 1.0);
    Eigen::Quaterniond cam_from_ref_cam_rotation =
        colmap::AverageQuaternions(cam_from_ref_cam_rotations_i, weights);

    rigs[camera_id_to_rig_id[camera_id]].SetSensorFromRig(
        sensor_t(SensorType::CAMERA, camera_id),
        Rigid3d(cam_from_ref_cam_rotation, nan_translation));
  }

  // Then, collect the rotations into frames and rigs
  for (auto& [frame_id, frame] : frames) {
    // Then, collect the rotations from the cameras to the reference camera
    std::vector<Eigen::Quaterniond> rig_from_world_rotations;
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      const auto& image = images.at(image_id);
      if (!image.is_registered) continue;

      if (image_id == frame_to_ref_image_id[frame_id]) {
        rig_from_world_rotations.push_back(cam_from_worlds[image_id].rotation);
      } else {
        auto cam_from_rig_opt =
            rigs[camera_id_to_rig_id[image.camera_id]].MaybeSensorFromRig(
                sensor_t(SensorType::CAMERA, image.camera_id));
        if (!cam_from_rig_opt.has_value()) continue;
        rig_from_world_rotations.push_back(
            cam_from_rig_opt.value().rotation.inverse() *
            cam_from_worlds[image_id].rotation);
      }

      const std::vector<double> rotation_weights(
          rig_from_world_rotations.size(), 1);
      Eigen::Quaterniond rig_from_world_rotation = colmap::AverageQuaternions(
          rig_from_world_rotations, rotation_weights);
      frame.SetRigFromWorld(Rigid3d(rig_from_world_rotation, nan_translation));
    }
  }
}

// TODO: add the gravity aligned version
// TODO: refine the code
void RigRotationEstimator::SetupLinearSystem(
    const ViewGraph& view_graph,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  // Clear all the structures
  sparse_matrix_.resize(0, 0);
  tangent_space_step_.resize(0);
  tangent_space_residual_.resize(0);
  rotation_estimated_.resize(0);
  image_id_to_idx_.clear();
  camera_id_to_idx_.clear();
  rel_temp_info_.clear();

  // Initialize the structures for estimated rotation
  image_id_to_idx_.reserve(images.size());
  camera_id_to_idx_.reserve(images.size());
  rotation_estimated_.resize(
      6 * images.size());  // allocate more memory than needed
  image_t num_dof = 0;
  frame_is_registered_.clear();
  frame_has_gravity_.clear();
  std::unordered_map<camera_t, rig_t> camera_id_to_rig_id;
  for (auto& [frame_id, frame] : frames) {
    frame_is_registered_[frame_id] = false;
    frame_has_gravity_[frame_id] =
        (frame.DataIds().size() == 1) &&
        (images[frame.DataIds().begin()->id].gravity_info.has_gravity);
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      const auto& image = images.at(image_id);
      if (!image.is_registered) continue;
      frame_is_registered_[frame_id] = true;
      // frame_has_gravity_[frame_id] =
      //     frame_has_gravity_[frame_id] || image.gravity_info.has_gravity;
      // camera_ids.insert(image.camera_id);
      camera_id_to_rig_id[image.camera_id] = frame.RigId();
    }
  }
  // for (auto& [image_id, image] : images) {
  //   if (!image.is_registered) continue;
  //   image_id_to_idx_[image_id] = num_dof;
  //   if (options_.use_gravity && image.gravity_info.has_gravity) {
  //     rotation_estimated_[num_dof] =
  //         RotUpToAngle(image.gravity_info.GetRAlign().transpose() *
  //                      image.cam_from_world.rotation.toRotationMatrix());
  //     num_dof++;

  //     if (fixed_camera_id_ == -1) {
  //       fixed_camera_rotation_ =
  //           Eigen::Vector3d(0, rotation_estimated_[num_dof - 1], 0);
  //       fixed_camera_id_ = image_id;
  //     }
  //   } else {
  //     rotation_estimated_.segment(num_dof, 3) =
  //         Rigid3dToAngleAxis(image.cam_from_world);
  //     num_dof += 3;
  //   }
  // }

  // First, we need to determine which cameras need to be estimated
  std::unordered_map<camera_t, Eigen::Vector3d> cam_from_rig_rotations;
  for (auto& [camera_id, rig_id] : camera_id_to_rig_id) {
    sensor_t sensor_id(SensorType::CAMERA, camera_id);
    if (rigs[rig_id].IsRefSensor(sensor_id)) continue;

    auto cam_from_rig = rigs[rig_id].MaybeSensorFromRig(sensor_id);
    if (!cam_from_rig.has_value() ||
        cam_from_rig.value().translation.hasNaN()) {
      if (camera_id_to_idx_.find(camera_id) == camera_id_to_idx_.end()) {
        camera_id_to_idx_[camera_id] = -1;
        if (cam_from_rig.has_value()) {
          // If the camera is not part of a rig, then we can use the first image
          // to initialize the rotation
          cam_from_rig_rotations[camera_id] =
              Rigid3dToAngleAxis(cam_from_rig.value());
        }
      }
    }
  }

  for (auto& [frame_id, frame] : frames) {
    // Skip the unregistered frames
    if (frame_is_registered_[frame_id] == false) continue;
    frame_id_to_idx_[frame_id] = num_dof;
    for (auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      image_id_to_idx_[image_id] = num_dof;  // point to the first element
    }
    // if (!frame.is_registered) continue;
    // for (const auto& image_id : frame.Snapshot()) {
    // for (const auto& image_id : frame.ImageIds()) {
    //   if (images.find(image_id) == images.end()) continue;

    // const auto& image = images.at(image_id);
    // if (!image.is_registered) continue;
    // image_id_to_idx_[image_id] = num_dof;
    // TODO: only support per image gravity info. Might need to change to have
    // a per frame gravity info
    image_t image_id_begin = frame.DataIds().begin()->id;
    if (options_.use_gravity &&
        images[image_id_begin].gravity_info.has_gravity) {
      rotation_estimated_[num_dof] = RotUpToAngle(
          images[image_id_begin].gravity_info.GetRAlign().transpose() *
          images[image_id_begin].CamFromWorld().rotation.toRotationMatrix());
      num_dof++;

      if (fixed_camera_id_ == -1) {
        fixed_camera_rotation_ =
            Eigen::Vector3d(0, rotation_estimated_[num_dof - 1], 0);
        fixed_camera_id_ = image_id_begin;
      }
    } else {
      rotation_estimated_.segment(num_dof, 3) =
          Rigid3dToAngleAxis(frame.RigFromWorld());
      num_dof += 3;
    }
  }

  // Set the camera id to index mapping for cameras that need to be
  // estimated.
  for (auto& [camera_id, camera_idx] : camera_id_to_idx_) {
    // If the camera is not part of a rig, then we can use the first image
    // to initialize the rotation
    camera_id_to_idx_[camera_id] = num_dof;
    if (cam_from_rig_rotations.find(camera_id) !=
        cam_from_rig_rotations.end()) {
      rotation_estimated_.segment(num_dof, 3) =
          cam_from_rig_rotations[camera_id];
    } else {
      // If the camera is part of a rig, then we can use the rig rotation
      // to initialize the rotation
      rotation_estimated_.segment(num_dof, 3) = Eigen::Vector3d::Zero();
    }
    num_dof += 3;
  }

  // // rig_is_registered_.reserve(rigs.size());
  // // frame_is_registered_
  // for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); ++idx_rig) {
  //   const auto& camera_rig = camera_rigs.at(idx_rig);
  //   rig_is_registered_.emplace_back();
  //   auto& rig_is_registered = rig_is_registered_.back();
  //   const size_t num_snapshots = camera_rig.NumSnapshots();

  //   for (size_t idx_snapshot = 0; idx_snapshot < num_snapshots;
  //        ++idx_snapshot) {
  //     bool is_registered = false;
  //     const auto& snapshot = camera_rig.Snapshots()[idx_snapshot];
  //     for (const auto image_id : snapshot) {
  //       const auto& image = images.at(image_id);
  //       if (images.find(image_id) == images.end()) continue;
  //       if (!images[image_id].is_registered) continue;
  //       image_id_to_camera_rig_index_.emplace(image_id, idx_rig);
  //       image_id_to_idx_[image_id] = num_dof;
  //       is_registered = true;
  //     }
  //     rig_is_registered.emplace_back(is_registered);

  //     if (is_registered) {
  //       Rigid3d rig_from_world =
  //           camera_rig.ComputeRigFromWorld(idx_snapshot, images);
  //       rotation_estimated_.segment(num_dof, 3) =
  //           Rigid3dToAngleAxis(rig_from_world);
  //       num_dof += 3;
  //     }
  //   }
  // }

  // If no cameras are set to be fixed, then take the first camera
  if (fixed_camera_id_ == -1) {
    // for (auto& [image_id, image] : images) {
    //   if (!image.is_registered) continue;
    for (auto& [frame_id, frame] : frames) {
      if (frame_is_registered_[frame_id] == false) continue;

      // fixed_camera_id_ = image_id;
      fixed_camera_id_ = frame.DataIds().begin()->id;
      fixed_camera_rotation_ = Rigid3dToAngleAxis(frame.RigFromWorld());

      // camera_t camera_id = images[image_id].camera_id;
      // if (image_id_to_camera_rig_index_.find(image_id) ==
      //     image_id_to_camera_rig_index_.end())
      //   fixed_camera_rotation_ = Rigid3dToAngleAxis(image.cam_from_world);
      // else
      //   fixed_camera_rotation_ = Rigid3dToAngleAxis(
      //       colmap::Inverse(
      //           camera_rigs[image_id_to_camera_rig_index_[image_id]].CamFromRig(
      //               camera_id)) *
      //       image.cam_from_world);
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

    camera_t camera_id1 = images[image_id1].camera_id;
    camera_t camera_id2 = images[image_id2].camera_id;

    int vector_idx1 = image_id_to_idx_[image_id1];
    int vector_idx2 = image_id_to_idx_[image_id2];

    // bool invalid_pair = false;
    // invalid_pair = (vector_idx1 == vector_idx2) && (camera_id1
    // if (vector_idx1 == vector_idx2) {
    //   // Skip the self loop
    //   continue;
    // }

    Rigid3d cam1_from_rig1, cam2_from_rig2;
    int idx_rig1 = frames[images[image_id1].frame_id].RigId();
    int idx_rig2 = frames[images[image_id2].frame_id].RigId();

    // int idx_camera1 = -1, idx_camera2 = -1;
    bool has_sensor_from_rig1 = false;
    bool has_sensor_from_rig2 = false;
    if (!images[image_id1].HasTrivialFrame()) {
      // camera_t camera_id = images[image_id1].camera_id;
      // cam1_from_rig1 =
      //     rigs[idx_rig1].SensorFromRig(sensor_t(SensorType::CAMERA,
      //     camera_id));
      auto cam1_from_rig1_opt = rigs[idx_rig1].MaybeSensorFromRig(
          sensor_t(SensorType::CAMERA, camera_id1));
      if (camera_id_to_idx_.find(camera_id1) == camera_id_to_idx_.end()) {
        cam1_from_rig1 = cam1_from_rig1_opt.value();
        has_sensor_from_rig1 = true;
      }
    }
    if (!images[image_id2].HasTrivialFrame()) {
      auto cam2_from_rig2_opt = rigs[idx_rig2].MaybeSensorFromRig(
          sensor_t(SensorType::CAMERA, camera_id2));
      // if (cam2_from_rig2_opt.has_value()) {
      if (camera_id_to_idx_.find(camera_id2) == camera_id_to_idx_.end()) {
        cam2_from_rig2 = cam2_from_rig2_opt.value();
        has_sensor_from_rig2 = true;
      }
      // cam2_from_rig2 =
      //     rigs[idx_rig2].MaybeSensorFromRig(sensor_t(SensorType::CAMERA,
      //     camera_id2));
    }

    // If both images are from the same rig and there is no need to estimate
    // the cam_from_rig, skip the estimation
    if (has_sensor_from_rig1 && has_sensor_from_rig2 &&
        vector_idx1 == vector_idx2) {
      continue;  // Skip the self loop
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

    image_t image_id1 = image_pair.image_id1;
    image_t image_id2 = image_pair.image_id2;

    camera_t camera_id1 = images[image_id1].camera_id;
    camera_t camera_id2 = images[image_id2].camera_id;

    frame_t frame_id1 = images[image_id1].frame_id;
    frame_t frame_id2 = images[image_id2].frame_id;

    if (frame_is_registered_[frame_id1] == false ||
        frame_is_registered_[frame_id2] == false) {
      continue;  // skip unregistered frames
    }

    int vector_idx1 = image_id_to_idx_[image_id1];
    int vector_idx2 = image_id_to_idx_[image_id2];

    int vector_idx_cam1 = -1;
    int vector_idx_cam2 = -1;
    if (camera_id_to_idx_.find(camera_id1) != camera_id_to_idx_.end()) {
      vector_idx_cam1 = camera_id_to_idx_[camera_id1];
    }
    if (camera_id_to_idx_.find(camera_id2) != camera_id_to_idx_.end()) {
      vector_idx_cam2 = camera_id_to_idx_[camera_id2];
    }

    rel_temp_info_[pair_id].index = curr_pos;
    rel_temp_info_[pair_id].idx_cam1 = vector_idx_cam1;
    rel_temp_info_[pair_id].idx_cam2 = vector_idx_cam2;

    // TODO: figure out the logic for the gravity aligned case
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
      if (!options_.use_gravity || !frame_has_gravity_[frame_id1]) {
        for (int i = 0; i < 3; i++) {
          coeffs.emplace_back(
              Eigen::Triplet<double>(curr_pos + i, vector_idx1 + i, -1));
        }
      } else
        // else, other components are zero, and can be safely ignored
        coeffs.emplace_back(
            Eigen::Triplet<double>(curr_pos + 1, vector_idx1, -1));

      // Similarly for the second componenet
      if (!options_.use_gravity || !frame_has_gravity_[frame_id2]) {
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

      // If both camera share the same rig, the terms in the linear system would
      // be cancelled
      if (!(vector_idx_cam1 == -1 && vector_idx_cam2 == -1)) {
        if (vector_idx_cam1 != -1) {
          // If the camera is not part of a rig, then we can use the first image
          // to initialize the rotation
          for (int i = 0; i < 3; i++) {
            coeffs.emplace_back(
                Eigen::Triplet<double>(curr_pos + i, vector_idx_cam1 + i, -1));
          }
        }
        if (vector_idx_cam2 != -1) {
          for (int i = 0; i < 3; i++) {
            coeffs.emplace_back(
                Eigen::Triplet<double>(curr_pos + i, vector_idx_cam2 + i, 1));
          }
        }
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

  // For rig case, we only keep one representative of the rig, so set all
  // other images to be not registered
  for (auto& [image_id, image] : images) {
    image.is_registered = false;
  }

  for (auto& [frame_id, frame] : frames) {
    if (frame_is_registered_[frame_id]) {
      // Set the first image in the frame to be registered
      image_t image_id_begin = frame.DataIds().begin()->id;
      images[image_id_begin].is_registered = true;
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

bool RigRotationEstimator::SolveL1Regression(
    const ViewGraph& view_graph,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  L1SolverOptions opt_l1_solver;
  opt_l1_solver.max_num_iterations = 10;

  L1Solver<Eigen::SparseMatrix<double>> l1_solver(
      opt_l1_solver, weights_.matrix().asDiagonal() * sparse_matrix_);
  double last_norm = 0;
  double curr_norm = 0;

  ComputeResiduals(view_graph, images);
  VLOG(2) << "ComputeResiduals done";

  int iteration = 0;
  for (iteration = 0; iteration < options_.max_num_l1_iterations; iteration++) {
    VLOG(2) << "L1 ADMM iteration: " << iteration;

    last_norm = curr_norm;
    // use the current residual as b (Ax - b)

    tangent_space_step_.setZero();
    l1_solver.Solve(weights_.matrix().asDiagonal() * tangent_space_residual_,
                    &tangent_space_step_);
    if (tangent_space_step_.array().isNaN().any()) {
      LOG(ERROR) << "nan error";
      iteration++;
      return false;
    }

    if (VLOG_IS_ON(2))
      LOG(INFO) << "residual:"
                << (sparse_matrix_ * tangent_space_step_ -
                    tangent_space_residual_)
                       .array()
                       .abs()
                       .sum();

    curr_norm = tangent_space_step_.norm();
    UpdateGlobalRotations(view_graph, frames, images);
    ComputeResiduals(view_graph, images);

    // Check the residual. If it is small, stop
    // TODO: strange bug for the L1 solver: update norm state constant
    if (ComputeAverageStepSize(images) <
            options_.l1_step_convergence_threshold ||
        std::abs(last_norm - curr_norm) < EPS) {
      if (std::abs(last_norm - curr_norm) < EPS)
        LOG(INFO) << "std::abs(last_norm - curr_norm) < EPS";
      iteration++;
      break;
    }
    opt_l1_solver.max_num_iterations =
        std::min(opt_l1_solver.max_num_iterations * 2, 100);
  }
  VLOG(2) << "L1 ADMM total iteration: " << iteration;
  return true;
}

bool RigRotationEstimator::SolveIRLS(
    const ViewGraph& view_graph,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  // TODO: Determine what is the best solver for this part
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> llt;

  // weight_matrix.setIdentity();
  // sparse_matrix_ = A_ori;

  llt.analyzePattern(sparse_matrix_.transpose() * sparse_matrix_);

  const double sigma = DegToRad(options_.irls_loss_parameter_sigma);
  VLOG(2) << "sigma: " << options_.irls_loss_parameter_sigma;

  Eigen::ArrayXd weights_irls(sparse_matrix_.rows());
  Eigen::SparseMatrix<double> at_weight;

  if (options_.use_gravity && images[fixed_camera_id_].gravity_info.has_gravity)
    weights_irls[sparse_matrix_.rows() - 1] = 1;
  else
    weights_irls.segment(sparse_matrix_.rows() - 3, 3).setConstant(1);

  ComputeResiduals(view_graph, images);
  int iteration = 0;
  for (iteration = 0; iteration < options_.max_num_irls_iterations;
       iteration++) {
    VLOG(2) << "IRLS iteration: " << iteration;

    // Compute the weights for IRLS
    for (auto& [pair_id, pair_info] : rel_temp_info_) {
      image_pair_t image_pair_pos = pair_info.index;
      double err_squared = 0;
      double w = 0;
      // If both cameras have gravity, then we only consider the y-axis
      if (pair_info.has_gravity)
        err_squared = std::pow(tangent_space_residual_[image_pair_pos], 2) +
                      pair_info.xz_error;
      // Otherwise, we consider all 3 dof
      else
        err_squared =
            tangent_space_residual_.segment<3>(image_pair_pos).squaredNorm();

      // Compute the weight
      if (options_.weight_type == RigRotationEstimatorOptions::GEMAN_MCCLURE) {
        double tmp = err_squared + sigma * sigma;
        w = sigma * sigma / (tmp * tmp);
      } else if (options_.weight_type ==
                 RigRotationEstimatorOptions::HALF_NORM) {
        w = std::pow(err_squared, (0.5 - 2) / 2);
      }

      if (std::isnan(w)) {
        LOG(ERROR) << "nan weight!";
        return false;
      }

      // If both cameras have gravity, then only 1 equation
      if (pair_info.has_gravity) weights_irls[image_pair_pos] = w;
      // Otherwise, 3 equations
      else
        weights_irls.segment<3>(image_pair_pos).setConstant(w);
    }

    // Update the factorization for the weighted values.
    at_weight = sparse_matrix_.transpose() *
                weights_irls.matrix().asDiagonal() *
                weights_.matrix().asDiagonal();

    llt.factorize(at_weight * sparse_matrix_);

    // Solve the least squares problem..
    tangent_space_step_.setZero();
    tangent_space_step_ = llt.solve(at_weight * tangent_space_residual_);
    UpdateGlobalRotations(view_graph, frames, images);
    ComputeResiduals(view_graph, images);

    // Check the residual. If it is small, stop
    if (ComputeAverageStepSize(images) <
        options_.irls_step_convergence_threshold) {
      iteration++;
      break;
    }
  }
  VLOG(2) << "IRLS total iteration: " << iteration;

  return true;
}

void RigRotationEstimator::UpdateGlobalRotations(
    const ViewGraph& view_graph,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    image_t vector_idx = image_id_to_idx_[image_id];
    if (!(options_.use_gravity && image.gravity_info.has_gravity)) {
      Eigen::Matrix3d R_ori =
          AngleAxisToRotation(rotation_estimated_.segment(vector_idx, 3));

      rotation_estimated_.segment(vector_idx, 3) = RotationToAngleAxis(
          R_ori *
          AngleAxisToRotation(-tangent_space_step_.segment(vector_idx, 3)));
      // if (camera_id_to_idx_.find(image.camera_id) != camera_id_to_idx_.end())
      // {
      //   cam_from_rigs[image.camera_id].push_back(R_ori);
      // }
    } else {
      rotation_estimated_[vector_idx] -= tangent_space_step_[vector_idx];
    }
  }

  std::unordered_map<camera_t, std::vector<Eigen::Matrix3d>> cam_from_rigs;
  for (auto& [camera_id, camera_idx] : camera_id_to_idx_) {
    cam_from_rigs[camera_id] = std::vector<Eigen::Matrix3d>();
  }
  for (auto& [frame_id, frame] : frames) {
    if (frame_is_registered_[frame_id] == false) continue;
    // Update the rig from world for the frame
    Eigen::Matrix3d R_ori = AngleAxisToRotation(
        rotation_estimated_.segment(frame_id_to_idx_[frame_id], 3));
    // frame.SetRigFromWorld(Rigid3d(R_ori, nan_translation));

    // Update the cam_from_rig for the cameras in the frame
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      const auto& image = images.at(image_id);
      if (camera_id_to_idx_.find(image.camera_id) != camera_id_to_idx_.end()) {
        cam_from_rigs[image.camera_id].push_back(R_ori);
      }
    }
  }

  // Update the global rotations for cam_from_rig cameras
  // Note: the update is non trivial, and we need to average the rotations from
  // all the frames
  for (auto& [camera_id, camera_idx] : camera_id_to_idx_) {
    Eigen::Matrix3d R_ori =
        AngleAxisToRotation(rotation_estimated_.segment(camera_idx, 3));

    std::vector<Eigen::Quaterniond> rig_rotations;
    Eigen::Matrix3d R_update =
        AngleAxisToRotation(-tangent_space_step_.segment(camera_idx, 3));
    for (const auto& R : cam_from_rigs[camera_id]) {
      // Update the rotation for the camera
      rig_rotations.push_back(
          Eigen::Quaterniond(R_ori * R * R_update * R.transpose()));
    }
    // Average the rotations for the rig
    Eigen::Quaterniond R_ave = colmap::AverageQuaternions(
        rig_rotations, std::vector<double>(rig_rotations.size(), 1));

    rotation_estimated_.segment(camera_idx, 3) =
        RotationToAngleAxis(R_ave.toRotationMatrix());
  }
}

void RigRotationEstimator::ComputeResiduals(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  int curr_pos = 0;
  for (auto& [pair_id, pair_info] : rel_temp_info_) {
    image_t image_id1 = view_graph.image_pairs.at(pair_id).image_id1;
    image_t image_id2 = view_graph.image_pairs.at(pair_id).image_id2;

    int idx1 = image_id_to_idx_[image_id1];
    int idx2 = image_id_to_idx_[image_id2];

    int idx_cam1 = pair_info.idx_cam1;
    int idx_cam2 = pair_info.idx_cam2;

    // TODO: figure out what to do with the gravity aligned case
    if (pair_info.has_gravity) {
      tangent_space_residual_[pair_info.index] =
          (RelAngleError(pair_info.angle_rel,
                         rotation_estimated_[image_id_to_idx_[image_id1]],
                         rotation_estimated_[image_id_to_idx_[image_id2]]));
    } else {
      Eigen::Matrix3d R_1, R_2;
      if (options_.use_gravity && images[image_id1].gravity_info.has_gravity) {
        R_1 = AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id1]]);
      } else {
        R_1 = AngleAxisToRotation(
            rotation_estimated_.segment(image_id_to_idx_[image_id1], 3));
      }

      if (options_.use_gravity && images[image_id2].gravity_info.has_gravity) {
        R_2 = AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id2]]);
      } else {
        R_2 = AngleAxisToRotation(
            rotation_estimated_.segment(image_id_to_idx_[image_id2], 3));
      }

      if (idx_cam1 != -1) {
        // If the camera is not part of a rig, then we can use the first image
        // to initialize the rotation
        R_1 =
            AngleAxisToRotation(rotation_estimated_.segment(idx_cam1, 3)) * R_1;
      }
      if (idx_cam2 != -1) {
        R_2 =
            AngleAxisToRotation(rotation_estimated_.segment(idx_cam2, 3)) * R_2;
      }

      tangent_space_residual_.segment(pair_info.index, 3) =
          -RotationToAngleAxis(R_2.transpose() * pair_info.R_rel * R_1);
    }
  }

  if (options_.use_gravity && images[fixed_camera_id_].gravity_info.has_gravity)
    tangent_space_residual_[tangent_space_residual_.size() - 1] =
        rotation_estimated_[image_id_to_idx_[fixed_camera_id_]] -
        fixed_camera_rotation_[1];
  else
    tangent_space_residual_.segment(tangent_space_residual_.size() - 3, 3) =
        RotationToAngleAxis(
            AngleAxisToRotation(fixed_camera_rotation_).transpose() *
            AngleAxisToRotation(rotation_estimated_.segment(
                image_id_to_idx_[fixed_camera_id_], 3)));
}

double RigRotationEstimator::ComputeAverageStepSize(
    const std::unordered_map<image_t, Image>& images) {
  double total_update = 0;
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;

    if (options_.use_gravity && image.gravity_info.has_gravity) {
      total_update += std::abs(tangent_space_step_[image_id_to_idx_[image_id]]);
    } else {
      total_update +=
          tangent_space_step_.segment(image_id_to_idx_[image_id], 3).norm();
    }
  }
  return total_update / image_id_to_idx_.size();
}

void RigRotationEstimator::ConvertResults(
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  for (auto& [frame_id, frame] : frames) {
    if (frame_is_registered_[frame_id] == false) continue;

    image_t image_id_begin = frame.DataIds().begin()->id;

    // Set the rig from world rotation
    // If the frame has gravity, then use the first image's gravity
    bool use_gravity = options_.use_gravity && frame_has_gravity_[frame_id];

    if (use_gravity) {
      frame.SetRigFromWorld(Rigid3d(
          Eigen::Quaterniond(
              images[image_id_begin].gravity_info.GetRAlign().transpose() *
              AngleToRotUp(
                  rotation_estimated_[image_id_to_idx_[image_id_begin]])),
          Eigen::Vector3d::Zero()));
    } else {
      frame.SetRigFromWorld(Rigid3d(
          Eigen::Quaterniond(AngleAxisToRotation(rotation_estimated_.segment(
              image_id_to_idx_[image_id_begin], 3))),
          Eigen::Vector3d::Zero()));
    }

    // frame.SetRigFromWorld(
    //     Eigen::Quaterniond(AngleAxisToRotation(
    //         rotation_estimated_.segment(image_id_to_idx_[frame.DataIds().begin()->sensor_id],
    //         3))));

    // for (auto& [image_id, image] : images) {
    //   if (image_id_to_idx_.find(image_id) == image_id_to_idx_.end()) {
    //     continue;
    //   }
    //   // If it belongs to some rig, then do not set the camera rotation
    //   if (image_id_to_camera_rig_index_.find(image_id) !=
    //       image_id_to_camera_rig_index_.end()) {
    //     continue;
    //   }

    //   if (options_.use_gravity && image.gravity_info.has_gravity) {
    //     image.cam_from_world.rotation = Eigen::Quaterniond(
    //         image.gravity_info.GetRAlign() *
    //         AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id]]));
    //   } else {
    //     image.cam_from_world.rotation =
    //     Eigen::Quaterniond(AngleAxisToRotation(
    //         rotation_estimated_.segment(image_id_to_idx_[image_id], 3)));
    //   }
    //   // Restore the prior position (t = -Rc = R * R_ori * t_ori = R *
    //   t_ori) image.cam_from_world.translation =
    //       (image.cam_from_world.rotation *
    //       image.cam_from_world.translation);
    // }
  }

  // add the estimated
  for (auto& [rig_id, rig] : rigs) {
    for (auto& [sensor_id, sensor] : rig.Sensors()) {
      if (camera_id_to_idx_.find(sensor_id.id) == camera_id_to_idx_.end()) {
        continue;  // Skip cameras that are not estimated
      }
      Rigid3d cam_from_rig;
      cam_from_rig.rotation = AngleAxisToRotation(
          rotation_estimated_.segment(camera_id_to_idx_[sensor_id.id], 3));
      cam_from_rig.translation.setConstant(
          std::numeric_limits<double>::quiet_NaN());  // No translation yet
      rig.SetSensorFromRig(sensor_id, cam_from_rig);
    }
  }
}

}  // namespace glomap