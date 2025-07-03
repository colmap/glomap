#include "glomap/controllers/rotation_averager.h"

#include "glomap/estimators/rotation_initializer.h"
#include "glomap/io/colmap_converter.h"

namespace glomap {

bool SolveRotationAveraging(ViewGraph& view_graph,
                            std::unordered_map<rig_t, Rig>& rigs,
                            std::unordered_map<frame_t, Frame>& frames,
                            std::unordered_map<image_t, Image>& images,
                            const RotationAveragerOptions& options) {
  view_graph.KeepLargestConnectedComponents(frames, images);

  bool solve_1dof_system = options.use_gravity && options.use_stratified;

  ViewGraph view_graph_grav;
  image_pair_t total_pairs = 0;
  image_pair_t grav_pairs = 0;
  if (solve_1dof_system) {
    // Prepare two sets: ones all with gravity, and one does not have gravity.
    // Solve them separately first, then solve them in a single system
    for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
      if (!image_pair.is_valid) continue;

      image_t image_id1 = image_pair.image_id1;
      image_t image_id2 = image_pair.image_id2;

      Image& image1 = images[image_id1];
      Image& image2 = images[image_id2];

      if (!image1.is_registered || !image2.is_registered) continue;

      total_pairs++;

      if (image1.HasGravity() && image2.HasGravity()) {
        view_graph_grav.image_pairs.emplace(
            pair_id,
            ImagePair(image_id1, image_id2, image_pair.cam2_from_cam1));
        grav_pairs++;
      }
    }
  }

  // If there is no image pairs with gravity or most image pairs are with
  // gravity, then just run the 3dof version
  bool status = (grav_pairs == 0) || (grav_pairs > total_pairs * 0.95);
  solve_1dof_system = solve_1dof_system && (!status);

  if (solve_1dof_system) {
    // Run the 1dof optimization
    LOG(INFO) << "Solving subset 1DoF rotation averaging problem in the mixed "
                 "prior system";
    int num_img_grv =
        view_graph_grav.KeepLargestConnectedComponents(frames, images);
    RotationEstimator rotation_estimator_grav(options);
    if (!rotation_estimator_grav.EstimateRotations(
            view_graph_grav, rigs, frames, images)) {
      return false;
    }
    view_graph.KeepLargestConnectedComponents(frames, images);
  }

  // By default, run trivial rotation averaging for rigged cameras if some
  // cam_from_rig are not estimated Check if there are rigs with non-trivial
  // cam_from_rig
  // bool run_trivial_ra = false;
  std::unordered_set<camera_t> camera_without_rig;
  rig_t max_rig_id = 0;
  for (const auto& [rig_id, rig] : rigs) {
    max_rig_id = std::max(max_rig_id, rig_id);
    for (auto& [sensor_id, sensor] : rig.Sensors()) {
      if (sensor_id.type != SensorType::CAMERA) continue;
      if (!rig.MaybeSensorFromRig(sensor_id).has_value()) {
        camera_without_rig.insert(sensor_id.id);
      }
    }
  }

  bool status_ra = false;
  // If the trivial rotation averaging is enabled, run it
  if (camera_without_rig.size() > 0 && !options.skip_initialization) {
    LOG(INFO) << "Running trivial rotation averaging for rigged cameras";
    // Create a rig for each camera
    std::unordered_map<rig_t, Rig> rigs_trivial;
    std::unordered_map<frame_t, Frame> frames_trivial;
    std::unordered_map<image_t, Image> images_trivial;

    // For cameras without rigs, create a trivial rig
    std::unordered_map<camera_t, rig_t> camera_id_to_rig_id;
    for (const auto& [rig_id, rig] : rigs) {
      Rig rig_trivial;
      rig_trivial.SetRigId(rig_id);
      rig_trivial.AddRefSensor(rig.RefSensorId());
      camera_id_to_rig_id[rig.RefSensorId().id] = rig_id;

      for (auto& [sensor_id, sensor] : rig.Sensors()) {
        if (sensor_id.type != SensorType::CAMERA) continue;
        if (rig.MaybeSensorFromRig(sensor_id).has_value()) {
          rig_trivial.AddSensor(sensor_id, sensor);
          camera_id_to_rig_id[sensor_id.id] = rig_id;
        }
      }
      rigs_trivial[rig_trivial.RigId()] = rig_trivial;
    }

    // Then, for each camera without rig, create a trivial rig
    for (const auto& camera_id : camera_without_rig) {
      Rig rig_trivial;
      rig_trivial.SetRigId(++max_rig_id);
      rig_trivial.AddRefSensor(sensor_t(SensorType::CAMERA, camera_id));
      rigs_trivial[rig_trivial.RigId()] = rig_trivial;
      camera_id_to_rig_id[camera_id] = rig_trivial.RigId();
    }

    frame_t max_frame_id = 0;
    for (const auto& [frame_id, frame] : frames) {
      if (frame_id == colmap::kInvalidFrameId) continue;
      max_frame_id = std::max(max_frame_id, frame_id);
    }
    max_frame_id++;

    for (auto& [frame_id, frame] : frames) {
      Frame frame_trivial = Frame();
      frame_trivial.SetFrameId(frame_id);
      frame_trivial.SetRigId(frame.RigId());
      frame_trivial.SetRigPtr(rigs_trivial.find(frame.RigId()) !=
                                      rigs_trivial.end()
                                  ? &rigs_trivial[frame.RigId()]
                                  : nullptr);
      frames_trivial[frame_id] = frame_trivial;

      for (const auto& data_id : frame.DataIds()) {
        image_t image_id = data_id.id;
        if (images.find(image_id) == images.end()) continue;
        const auto& image = images.at(image_id);
        if (!image.is_registered) continue;
        images_trivial.insert(std::make_pair(
            image_id, Image(image_id, image.camera_id, image.file_name)));
        images_trivial[image_id].is_registered = true;

        if (camera_without_rig.find(images_trivial[image_id].camera_id) ==
            camera_without_rig.end()) {
          // images_trivial_to_frame_id[image_id] = frame_id;

          frames_trivial[frame_id].AddDataId(images_trivial[image_id].DataId());
          images_trivial[image_id].frame_id = frame_id;
          images_trivial[image_id].frame_ptr = &frames_trivial[frame_id];
        } else {
          // If the camera is not in any rig, then create a trivial frame
          // for it
          CreateFrameForImage(Rigid3d(),
                              images_trivial[image_id],
                              rigs_trivial,
                              frames_trivial,
                              camera_id_to_rig_id[image.camera_id],
                              max_frame_id);
          max_frame_id++;
        }
      }
    }

    // Run the trivial rotation averaging
    RotationEstimatorOptions options_trivial = options;
    options_trivial.skip_initialization = options.skip_initialization;
    RotationEstimator rotation_estimator_trivial(options_trivial);
    rotation_estimator_trivial.EstimateRotations(
        view_graph, rigs_trivial, frames_trivial, images_trivial);

    // Collect the results
    std::unordered_map<image_t, Rigid3d> cam_from_worlds;
    for (const auto& [image_id, image] : images_trivial) {
      if (!image.is_registered) continue;
      cam_from_worlds[image_id] = image.CamFromWorld();
    }

    ConvertRotationsFromImageToRig(cam_from_worlds, images, rigs, frames);

    RotationEstimatorOptions options_ra = options;
    options_ra.skip_initialization = true;
    RotationEstimator rotation_estimator(options_ra);
    status_ra =
        rotation_estimator.EstimateRotations(view_graph, rigs, frames, images);
    view_graph.KeepLargestConnectedComponents(frames, images);
  } else {
    RotationAveragerOptions options_ra = options;
    // For cases where there are some cameras without known cam_from_rig
    // transformation, we need to run the rotation averaging with the
    // skip_initialization flag set to false for convergence
    if (camera_without_rig.size() > 0) {
      options_ra.skip_initialization = false;
    }

    RotationEstimator rotation_estimator(options_ra);
    status_ra =
        rotation_estimator.EstimateRotations(view_graph, rigs, frames, images);
    view_graph.KeepLargestConnectedComponents(frames, images);
  }
  return status_ra;
}

}  // namespace glomap