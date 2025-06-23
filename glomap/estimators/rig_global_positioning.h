#pragma once

// #include "glomap/estimators/global_positioning.h"
#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

namespace glomap {

// struct RigGlobalPositionerOptions : public GlobalPositionerOptions {
//   //   // Whether initialize the reconstruction randomly
//   //   bool generate_random_positions = true;
//   //   bool generate_random_points = true;
//   //   bool generate_scales = true;  // Now using fixed 1 as initializaiton

//   //   // Flags for which parameters to optimize
//   //   bool optimize_positions = true;
//   //   bool optimize_points = true;
//   //   bool optimize_scales = true;

//   //   // Constrain the minimum number of views per track
//   //   int min_num_view_per_track = 3;

//   RigGlobalPositionerOptions() : GlobalPositionerOptions() {}
// };

struct RigGlobalPositionerOptions : public OptimizationBaseOptions {
  // ONLY_POINTS is recommended
  enum ConstraintType {
    // only include camera to point constraints
    ONLY_POINTS,
    // only include camera to camera constraints
    ONLY_CAMERAS,
    // the points and cameras are reweighted to have similar total contribution
    POINTS_AND_CAMERAS_BALANCED,
    // treat each contribution from camera to point and camera to camera equally
    POINTS_AND_CAMERAS,
  };

  // Whether initialize the reconstruction randomly
  bool generate_random_positions = true;
  bool generate_random_points = true;
  bool generate_scales = true;  // Now using fixed 1 as initializaiton

  // Flags for which parameters to optimize
  bool optimize_positions = true;
  bool optimize_points = true;
  bool optimize_scales = true;

  bool use_gpu = true;
  std::string gpu_index = "-1";
  int min_num_images_gpu_solver = 50;

  // Constrain the minimum number of views per track
  int min_num_view_per_track = 3;

  // Random seed
  unsigned seed = 1;

  // the type of global positioning
  ConstraintType constraint_type = ONLY_POINTS;
  double constraint_reweight_scale =
      1.0;  // only relevant for POINTS_AND_CAMERAS_BALANCED

  RigGlobalPositionerOptions() : OptimizationBaseOptions() {
    thres_loss_function = 1e-1;
  }

  std::shared_ptr<ceres::LossFunction> CreateLossFunction() {
    return std::make_shared<ceres::HuberLoss>(thres_loss_function);
  }
};

class RigGlobalPositioner {
 public:
  RigGlobalPositioner(const RigGlobalPositionerOptions& options);

  // Returns true if the optimization was a success, false if there was a
  // failure.
  // Assume tracks here are already filtered
  bool Solve(const ViewGraph& view_graph,
             std::unordered_map<rig_t, Rig>& rigs,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<frame_t, Frame>& frames,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

  RigGlobalPositionerOptions& GetOptions() { return options_; }

 protected:
  void SetupProblem(const ViewGraph& view_graph,
                    const std::unordered_map<rig_t, Rig>& rigs,
                    const std::unordered_map<track_t, Track>& tracks);

  // void ExtractRigsFromWorld(const std::unordered_map<rig_t, Rig>& rigs,
  //                           const std::unordered_map<frame_t, Frame>& frames,
  //                           const std::unordered_map<image_t, Image>&
  //                           images);

  // Initialize all cameras to be random.
  void InitializeRandomPositions(const ViewGraph& view_graph,
                                 std::unordered_map<frame_t, Frame>& frames,
                                 std::unordered_map<image_t, Image>& images,
                                 std::unordered_map<track_t, Track>& tracks);

  // // Creates camera to camera constraints from relative translations. (3D)
  // void AddCameraToCameraConstraints(const ViewGraph& view_graph,
  //                                   std::unordered_map<image_t, Image>&
  //                                   images);

  // Add tracks to the problem
  void AddPointToCameraConstraints(
      std::unordered_map<rig_t, Rig>& rigs,
      std::unordered_map<camera_t, Camera>& cameras,
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<image_t, Image>& images,
      std::unordered_map<track_t, Track>& tracks);

  // Add a single track to the problem
  void AddTrackToProblem(track_t track_id,
                         std::unordered_map<rig_t, Rig>& rigs,
                         std::unordered_map<camera_t, Camera>& cameras,
                         std::unordered_map<frame_t, Frame>& frames,
                         std::unordered_map<image_t, Image>& images,
                         std::unordered_map<track_t, Track>& tracks);

  // Set the parameter groups
  void AddCamerasAndPointsToParameterGroups(
      std::unordered_map<rig_t, Rig>& rigs,
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<track_t, Track>& tracks);

  // Parameterize the variables, set some variables to be constant if desired
  void ParameterizeVariables(std::unordered_map<rig_t, Rig>& rigs,
                             std::unordered_map<frame_t, Frame>& frames,
                             std::unordered_map<track_t, Track>& tracks);

  // During the optimization, the camera translation is set to be the camera
  // center Convert the results back to camera poses
  void ConvertResults(std::unordered_map<rig_t, Rig>& rigs,
                      std::unordered_map<frame_t, Frame>& frames);

  RigGlobalPositionerOptions options_;

  std::mt19937 random_generator_;
  std::unique_ptr<ceres::Problem> problem_;

  // Loss functions for reweighted terms.
  std::shared_ptr<ceres::LossFunction> loss_function_;
  std::shared_ptr<ceres::LossFunction> loss_function_ptcam_uncalibrated_;
  std::shared_ptr<ceres::LossFunction> loss_function_ptcam_calibrated_;

  // Auxiliary scale variables.
  std::vector<double> scales_;

  // Reconstruction& reconstruction_;

  // std::shared_ptr<ceres::Problem> problem_;
  // std::unique_ptr<ceres::LossFunction> loss_function_;

  // std::unordered_set<camera_t> camera_ids_;
  // std::unordered_map<point3D_t, size_t> point3D_num_observations_;

  // Mapping from images to camera rigs.
  // std::unordered_map<image_t, CameraRig*> image_id_to_camera_rig_;
  // std::unordered_map<image_t, int> image_id_to_camera_rig_index_;
  // std::unordered_map<image_t, Rigid3d*> image_id_to_rig_from_world_;

  // For each camera rig, the absolute camera rig poses for all snapshots.
  // std::vector<std::vector<Rigid3d>> rigs_from_world_;

  // // The Quaternions added to the problem, used to set the local
  // // parameterization once after setting up the problem.
  // std::unordered_set<double*> parameterized_cams_from_rig_rotations_;

  std::unordered_map<rig_t, double> rig_scales_;

  // colmap::Reconstruction reconstruction_;
};

}  // namespace glomap
