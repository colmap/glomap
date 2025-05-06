#include "glomap/controllers/rotation_averager.h"

#include "glomap/controllers/global_mapper.h"
#include "glomap/estimators/gravity_refinement.h"
#include "glomap/io/colmap_io.h"
#include "glomap/math/rigid3d.h"
#include "glomap/types.h"

#include <colmap/estimators/alignment.h>
#include <colmap/scene/synthetic.h>
#include <colmap/util/testing.h>

#include <gtest/gtest.h>

namespace glomap {
namespace {

void CreateRandomRotation(const double stddev, Eigen::Quaterniond& q) {
  std::random_device rd{};
  std::mt19937 gen{rd()};

  // Construct a random axis
  double theta = double(rand()) / RAND_MAX * 2 * M_PI;
  double phi = double(rand()) / RAND_MAX * M_PI;
  Eigen::Vector3d axis(std::cos(theta) * std::sin(phi),
                       std::sin(theta) * std::sin(phi),
                       std::cos(phi));

  // Construct a random angle
  std::normal_distribution<double> d{0, stddev};
  double angle = d(gen);
  q = Eigen::AngleAxisd(angle, axis);
}

void PrepareGravity(const colmap::Reconstruction& gt,
                    std::unordered_map<image_t, Image>& images,
                    double stddev_gravity = 0.0,
                    double outlier_ratio = 0.0) {
  for (auto& image_id : gt.RegImageIds()) {
    Eigen::Vector3d gravity =
        gt.Image(image_id).CamFromWorld().rotation * Eigen::Vector3d(0, 1, 0);

    if (stddev_gravity > 0.0) {
      Eigen::Quaterniond q;
      CreateRandomRotation(DegToRad(stddev_gravity), q);
      gravity = q * gravity;
    }

    if (outlier_ratio > 0.0 && double(rand()) / RAND_MAX < outlier_ratio) {
      Eigen::Quaterniond q;
      CreateRandomRotation(1., q);
      gravity =
          Rigid3dToAngleAxis(Rigid3d(q, Eigen::Vector3d::Zero())).normalized();
    }
    images[image_id].gravity_info.SetGravity(gravity);
  }
}

GlobalMapperOptions CreateMapperTestOptions() {
  GlobalMapperOptions options;
  options.skip_view_graph_calibration = false;
  options.skip_relative_pose_estimation = false;
  options.skip_rotation_averaging = true;
  options.skip_track_establishment = true;
  options.skip_global_positioning = true;
  options.skip_bundle_adjustment = true;
  options.skip_retriangulation = true;

  return options;
}

RotationAveragerOptions CreateRATestOptions(bool use_gravity = false) {
  RotationAveragerOptions options;
  options.skip_initialization = true;
  options.use_gravity = use_gravity;
  return options;
}

void ExpectEqualRotations(const colmap::Reconstruction& gt,
                          const colmap::Reconstruction& computed,
                          const double max_rotation_error_deg) {
  const std::set<image_t> reg_image_ids_set = gt.RegImageIds();
  std::vector<image_t> reg_image_ids(reg_image_ids_set.begin(),
                                     reg_image_ids_set.end());
  for (size_t i = 0; i < reg_image_ids.size(); i++) {
    const image_t image_id1 = reg_image_ids[i];
    for (size_t j = 0; j < reg_image_ids.size(); j++) {
      if (i == j) continue;
      const image_t image_id2 = reg_image_ids[j];

      const Rigid3d cam2_from_cam1 =
          computed.Image(image_id2).CamFromWorld() *
          colmap::Inverse(computed.Image(image_id1).CamFromWorld());

      const Rigid3d cam2_from_cam1_gt =
          gt.Image(image_id2).CamFromWorld() *
          colmap::Inverse(gt.Image(image_id1).CamFromWorld());

      double rotation_error_deg = CalcAngle(cam2_from_cam1_gt, cam2_from_cam1);
      EXPECT_LT(rotation_error_deg, max_rotation_error_deg);
    }
  }
}

void ExpectEqualGravity(
    const colmap::Reconstruction& gt,
    const std::unordered_map<image_t, Image>& images_computed,
    const double max_gravity_error_deg) {
  for (const auto& image_id : gt.RegImageIds()) {
    const Eigen::Vector3d gravity_gt =
        gt.Image(image_id).CamFromWorld().rotation * Eigen::Vector3d(0, 1, 0);
    const Eigen::Vector3d gravity_computed =
        images_computed.at(image_id).gravity_info.GetGravity();

    double gravity_error_deg = CalcAngle(gravity_gt, gravity_computed);
    EXPECT_LT(gravity_error_deg, max_gravity_error_deg);
  }
}

TEST(RotationEstimator, WithoutNoise) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 9;
  synthetic_dataset_options.num_points3D = 50;
  synthetic_dataset_options.point2D_stddev = 0;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  // PrepareRelativeRotations(view_graph, images);
  PrepareGravity(gt_reconstruction, images);

  GlobalMapper global_mapper(CreateMapperTestOptions());
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  // Version with Gravity
  for (bool use_gravity : {true, false}) {
    SolveRotationAveraging(
        view_graph, images, CreateRATestOptions(use_gravity));

    colmap::Reconstruction reconstruction;
    ConvertGlomapToColmap(cameras, images, tracks, reconstruction);
    ExpectEqualRotations(
        gt_reconstruction, reconstruction, /*max_rotation_error_deg=*/1e-2);
  }
}

TEST(RotationEstimator, WithNoiseAndOutliers) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  // FLAGS_v = 1;
  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 7;
  synthetic_dataset_options.num_points3D = 100;
  synthetic_dataset_options.point2D_stddev = 1;
  synthetic_dataset_options.inlier_match_ratio = 0.6;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  PrepareGravity(gt_reconstruction, images, /*stddev_gravity=*/3e-1);

  GlobalMapper global_mapper(CreateMapperTestOptions());
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  for (bool use_gravity : {true, false}) {
    SolveRotationAveraging(
        view_graph, images, CreateRATestOptions(use_gravity));

    colmap::Reconstruction reconstruction;
    ConvertGlomapToColmap(cameras, images, tracks, reconstruction);
    if (use_gravity)
      ExpectEqualRotations(
          gt_reconstruction, reconstruction, /*max_rotation_error_deg=*/1.5);
    else
      ExpectEqualRotations(
          gt_reconstruction, reconstruction, /*max_rotation_error_deg=*/2.);
  }
}

TEST(RotationEstimator, RefineGravity) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  // FLAGS_v = 2;
  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 100;
  synthetic_dataset_options.num_points3D = 200;
  synthetic_dataset_options.point2D_stddev = 0;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  PrepareGravity(
      gt_reconstruction, images, /*stddev_gravity=*/0., /*outlier_ratio=*/0.3);

  GlobalMapper global_mapper(CreateMapperTestOptions());
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  GravityRefinerOptions opt_grav_refine;
  GravityRefiner grav_refiner(opt_grav_refine);
  grav_refiner.RefineGravity(view_graph, images);

  // Check whether the gravity does not have error after refinement
  ExpectEqualGravity(gt_reconstruction, images, /*max_gravity_error_deg=*/1e-2);
}

}  // namespace
}  // namespace glomap
