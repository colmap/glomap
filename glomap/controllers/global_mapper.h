#pragma once

#include "glomap/controllers/track_establishment.h"
#include "glomap/controllers/track_retriangulation.h"
#include "glomap/estimators/bundle_adjustment.h"
#include "glomap/estimators/global_positioning.h"
#include "glomap/estimators/global_rotation_averaging.h"
#include "glomap/estimators/relpose_estimation.h"
#include "glomap/estimators/view_graph_calibration.h"
#include "glomap/types.h"

#include <colmap/scene/database.h>

namespace glomap {

struct GlobalMapperOptions {
  // Options for each component
  ViewGraphCalibratorOptions opt_vgcalib;
  RelativePoseEstimationOptions opt_relpose;
  RotationEstimatorOptions opt_ra;
  TrackEstablishmentOptions opt_track;
  GlobalPositionerOptions opt_gp;
  BundleAdjusterOptions opt_ba;
  TriangulatorOptions opt_triangulator;

  // Inlier thresholds for each component
  InlierThresholdOptions inlier_thresholds;

  // Control the number of iterations for each component
  int num_iteration_bundle_adjustment = 3;
  int num_iteration_retriangulation = 1;

  // Control the flow of the global sfm
  bool skip_preprocessing = false;
  bool skip_view_graph_calibration = false;
  bool skip_relative_pose_estimation = false;
  bool skip_rotation_averaging = false;
  bool skip_track_establishment = false;
  bool skip_global_positioning = false;
  bool skip_bundle_adjustment = false;
  bool skip_retriangulation = false;
  bool skip_pruning = true;
};

class GlobalMapper {
 public:
  GlobalMapper(const GlobalMapperOptions& options) : options_(options) {}

  bool Solve(const colmap::Database& database,
             ViewGraph& view_graph,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

 private:
  const GlobalMapperOptions options_;
};

}  // namespace glomap
