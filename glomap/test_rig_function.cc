

#include "glomap/io/colmap_converter.h"
#include "glomap/io/colmap_io.h"
#include "glomap/io/pose_io.h"
#include "glomap/controllers/rig_global_mapper.h"
#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/processors/relpose_filter.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/test/prepare_experiment.h"
#include "glomap/types.h"

#include <colmap/util/timer.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <string>

#include "glomap/json.h"

#include <fstream>

using json = nlohmann::json;

using namespace glomap;
int main(int argc, char** argv) {
  colmap::InitializeGlog(argv);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 2;

  // LOG(INFO) << "argc: " << argc << std::endl;

  // std::string database_path;
  // database_path = argv[1];
  std::string database_path;
  database_path = "../../prague/db.db";

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  // Load the database
  colmap::Database database(database_path);
  ConvertDatabaseToGlomap(database, view_graph, cameras, images);
  std::cout << "Loaded database" << std::endl;

  ReadRelPose("../../prague/relpoase_glomap.txt", images, view_graph);

  // // --------------------------------------------------------------
  // // For experiment, keep only 10 images for each sequence
  // int kept_img = 400;
  // std::unordered_map<camera_t, std::vector<image_t>> camera_id_to_image_id;
  // for (auto& [camera_id, camera] : cameras) {
  //   camera_id_to_image_id[camera_id] = std::vector<image_t>();
  // }

  // for (auto& [image_id, image] : images) {
  //   camera_id_to_image_id[image.camera_id].emplace_back(image_id);
  // }

  // std::unordered_set<image_t> erased_ids;
  // for (auto& [camera_id, camera] : cameras) {
  //   std::vector<image_t>& image_ids = camera_id_to_image_id[camera_id];
  //   std::sort(image_ids.begin(), image_ids.end());
  //   for (size_t i = kept_img; i < image_ids.size(); i++) {
  //     erased_ids.insert(image_ids[i]);
  //   }
  // }

  // std::unordered_set<image_pair_t> erased_pair_ids;
  // for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
  //   if (erased_ids.find(image_pair.image_id1) != erased_ids.end() ||
  //       erased_ids.find(image_pair.image_id2) != erased_ids.end())
  //     image_pair.is_valid = false;
  // }
  // // --------------------------------------------------------------

  int num_img = view_graph.KeepLargestConnectedComponents(images);
  std::cout << "KeepLargestConnectedComponents done" << std::endl;
  std::cout << "num_img: " << num_img << std::endl;

  // --------------------------------------------------------------
  // Set up camera rigs
  std::vector<CameraRig> camera_rigs;
  camera_rigs.emplace_back(CameraRig());
  CameraRig& camera_rig = camera_rigs[0];

  // Read rig info from the calib.json
  std::string calib_path = "../../prague/calib.json";
  std::ifstream calib_file(calib_path, std::ifstream::binary);
  json calib = json::parse(calib_file);
  Eigen::Matrix3d R_0 = Eigen::Matrix3d::Zero();
  R_0(0, 1) = 1;
  R_0(1, 0) = -1;
  R_0(2, 2) = 1;
  Rigid3d rig_0(Eigen::Quaterniond(R_0), Eigen::Vector3d::Zero());
  for (int idx = 0; idx < 6; idx++) {
    Eigen::Matrix3d R;
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        R(i, j) = calib["cams"]["cam" + std::to_string(idx)]["R"][i][j];
      }
    }
    Eigen::Vector3d t;
    for (size_t i = 0; i < 3; i++) {
      t[i] = calib["cams"]["cam" + std::to_string(idx)]["t"][i];
    }

    camera_rig.AddCamera(
        idx + 1, rig_0 * colmap::Inverse(Rigid3d(Eigen::Quaterniond(R), t)));
    // idx + 1,  colmap::Inverse(Rigid3d(Eigen::Quaterniond(R *
    // R_0.transpose()), t)));
  }
  std::cout << "AddCamera done" << std::endl;

  // Add snapshot to the CameraRig
  std::unordered_map<int, std::vector<image_t>> snapshot_idx_to_image_ids;
  for (auto& [image_id, image] : images) {
    int str_len = image.file_name.size();

    int sequence_idx = std::stoi(image.file_name.substr(str_len - 4 - 7, 7));
    snapshot_idx_to_image_ids[sequence_idx].emplace_back(image_id);
  }

  for (const auto& [snapshot_idx, image_ids] : snapshot_idx_to_image_ids) {
    camera_rig.AddSnapshot(image_ids);
  }

  // --------------------------------------------------------------
  // Establish rigs

  RigGlobalMapperOptions options;

  // Run the relative pose estimation and establish tracks
  options.skip_preprocessing = true;
  options.skip_view_graph_calibration = true;
  options.skip_relative_pose_estimation = true;
  options.skip_rotation_averaging = false;
  options.skip_track_establishment = false;

  options.skip_global_positioning = false;
  options.skip_bundle_adjustment = false;
  options.skip_retriangulation = true;
  options.skip_pruning = true;

  options.inlier_thresholds.min_inlier_num = 30;
  options.inlier_thresholds.max_epipolar_error_E = 1.;

  options.opt_ba.solver_options.max_num_iterations = 200;

  options.opt_track.min_num_tracks_per_view = 50;

  InlierThresholdOptions inlier_thresholds = options.inlier_thresholds;
  // Undistort the images and filter edges by inlier number
  UndistortImages(cameras, images, true);
  ImagePairsInlierCount(view_graph, cameras, images, inlier_thresholds, true);

  RelPoseFilter::FilterInlierNum(view_graph,
                                 options.inlier_thresholds.min_inlier_num);
  RelPoseFilter::FilterInlierRatio(view_graph,
                                   options.inlier_thresholds.min_inlier_ratio);

  options.opt_gp.use_gpu = false;

  options.opt_ba.use_gpu = false;

  RigGlobalMapper global_mapper(options);
  global_mapper.Solve(
      database, view_graph, camera_rigs, cameras, images, tracks);


  WriteGlomapReconstruction("test_4", cameras, images, tracks, "bin", "");
  // // -------------------------------------------------
  // std::ofstream file_rel;
  // file_rel.open("relpose_3dof_trans.txt");
  // std::unordered_map<image_pair_t, ImagePair>& image_pairs =
  // view_graph.image_pairs; std::vector<image_pair_t> image_pair_ids; for
  // (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
  //     if (!image_pair.is_valid) continue;
  //     image_pair_ids.push_back(image_pair_id);
  // }

  // std::cout << "image_pairs.size(): " << image_pairs.size() << std::endl;

  // for (image_pair_t pair = 0; pair < image_pair_ids.size(); pair++) {
  //     image_pair_t image_pair_id = image_pair_ids[pair];
  //     ImagePair& image_pair = image_pairs[image_pair_ids[pair]];
  //     image_t idx1 = image_pair.image_id1;
  //     image_t idx2 = image_pair.image_id2;

  //     // CameraPose pose_rel_calc = image_pair.pose_rel;
  //     std::string pair_name = images[idx1].file_name + "-" +
  //     images[idx2].file_name; file_rel << pair_name << " " <<
  //     image_pair.weight; for (int i = 0; i < 4; i++) {
  //         file_rel << " " << image_pair.cam2_from_cam1.rotation.coeffs()[i];
  //     }
  //     for (int i = 0; i < 3; i++) {
  //         file_rel << " " << image_pair.cam2_from_cam1.translation[i];
  //     }
  //     file_rel << "\n";

  // }
  // file_rel.close();
  // // -------------------------------------------------

  // WriteGlomapReconstruction(argv[2], cameras, images, tracks, "bin", "");

  return 0;
};
