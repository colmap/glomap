#include "glomap/controllers/global_mapper.h"

#include "glomap/io/colmap_io.h"
#include "glomap/types.h"

#include <colmap/estimators/alignment.h>
#include <colmap/scene/synthetic.h>
#include <colmap/util/testing.h>

#include <gtest/gtest.h>

namespace glomap {
namespace {

void ExpectEqualReconstructions(const colmap::Reconstruction& gt,
                                const colmap::Reconstruction& computed,
                                const double max_rotation_error_deg,
                                const double max_proj_center_error,
                                const double num_obs_tolerance) {
  EXPECT_EQ(computed.NumCameras(), gt.NumCameras());
  EXPECT_EQ(computed.NumImages(), gt.NumImages());
  EXPECT_EQ(computed.NumRegImages(), gt.NumRegImages());
  EXPECT_GE(computed.ComputeNumObservations(),
            (1 - num_obs_tolerance) * gt.ComputeNumObservations());

  colmap::Sim3d gtFromComputed;
  colmap::AlignReconstructionsViaProjCenters(computed,
                                             gt,
                                             /*max_proj_center_error=*/0.1,
                                             &gtFromComputed);

  const std::vector<colmap::ImageAlignmentError> errors =
      colmap::ComputeImageAlignmentError(computed, gt, gtFromComputed);
  EXPECT_EQ(errors.size(), gt.NumImages());
  for (const auto& error : errors) {
    EXPECT_LT(error.rotation_error_deg, max_rotation_error_deg);
    EXPECT_LT(error.proj_center_error, max_proj_center_error);
  }
}

GlobalMapperOptions CreateTestOptions() {
  GlobalMapperOptions options;
  options.skip_view_graph_calibration = false;
  options.skip_relative_pose_estimation = false;
  options.skip_rotation_averaging = false;
  options.skip_track_establishment = false;
  options.skip_global_positioning = false;
  options.skip_bundle_adjustment = false;
  options.skip_retriangulation = false;
  return options;
}

TEST(GlobalMapper, WithoutNoise) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 7;
  synthetic_dataset_options.num_points3D = 50;
  synthetic_dataset_options.point2D_stddev = 0;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  GlobalMapper global_mapper(CreateTestOptions());
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  colmap::Reconstruction reconstruction;
  ConvertGlomapToColmap(cameras, images, tracks, reconstruction);

  ExpectEqualReconstructions(gt_reconstruction,
                             reconstruction,
                             /*max_rotation_error_deg=*/1e-2,
                             /*max_proj_center_error=*/1e-4,
                             /*num_obs_tolerance=*/0);
}

TEST(GlobalMapper, WithNoiseAndOutliers) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 7;
  synthetic_dataset_options.num_points3D = 100;
  synthetic_dataset_options.point2D_stddev = 0.5;
  synthetic_dataset_options.inlier_match_ratio = 0.6;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  GlobalMapper global_mapper(CreateTestOptions());
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  colmap::Reconstruction reconstruction;
  ConvertGlomapToColmap(cameras, images, tracks, reconstruction);

  ExpectEqualReconstructions(gt_reconstruction,
                             reconstruction,
                             /*max_rotation_error_deg=*/1e-1,
                             /*max_proj_center_error=*/1e-1,
                             /*num_obs_tolerance=*/0.02);
}

}  // namespace
}  // namespace glomap
