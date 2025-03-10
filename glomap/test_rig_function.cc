

#include "glomap/io/colmap_converter.h"
// #include "glomap/io/theia_converter.h"
#include "glomap/io/colmap_io.h"
// #include "glomap/controllers/global_mapper_stochastic.h"
#include "glomap/controllers/global_mapper.h"
#include "glomap/estimators/callback_functions.h"
#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/test/prepare_experiment.h"
#include "glomap/types.h"

#include <colmap/util/timer.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <string>

// #include "glomap/io/theia_io.h"
// #include "glomap/test/theia_globalsfm.h"
#include "glomap/processors/image_pair_inliers.h"
#include "glomap/processors/image_undistorter.h"
// #include <theia/sfm/global_reconstruction_estimator.h>
// #include <theia/sfm/hybrid_reconstruction_estimator.h>
// #include <theia/sfm/incremental_reconstruction_estimator.h>
// #include <theia/sfm/reconstruction.h>
// #include <theia/sfm/view_graph/view_graph.h>
// #include <theia/io/write_colmap_files.h>

#include "glomap/estimators/rig_global_positioning.h"

// #include <json/value.h>
// #include <jsoncpp/json/json.h>
// #include <json/json.h>
#include "glomap/json.h"

#include <fstream>

using json = nlohmann::json;

using namespace glomap;
int main(int argc, char** argv) {
  colmap::InitializeGlog(argv);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 0;

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

//   // --------------------------------------------------------------
//   // For experiment, keep only 10 images for each sequence
//   int kept_img = 10;
//   std::unordered_map<camera_t, std::vector<image_t>> camera_id_to_image_id;
//   for (auto& [camera_id, camera] : cameras) {
//     camera_id_to_image_id[camera_id] = std::vector<image_t>();
//   }

//   for (auto& [image_id, image] : images) {
//     camera_id_to_image_id[image.camera_id].emplace_back(image_id);
//   }

//   std::unordered_set<image_t> erased_ids;
//   for (auto& [camera_id, camera] : cameras) {
//     std::vector<image_t>& image_ids = camera_id_to_image_id[camera_id];
//     std::sort(image_ids.begin(), image_ids.end());
//     for (size_t i = kept_img; i < image_ids.size(); i++) {
//       erased_ids.insert(image_ids[i]);
//     }
//   }

//   std::unordered_set<image_pair_t> erased_pair_ids;
//   for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
//     if (erased_ids.find(image_pair.image_id1) != erased_ids.end() ||
//         erased_ids.find(image_pair.image_id2) != erased_ids.end())
//       image_pair.is_valid = false;
//   }
//   // --------------------------------------------------------------

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
  // Json::Value calib;
  // calib_file >> calib;
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

    if (idx == 0) {
      // R_0 = R;
      // rig_0 = Rigid3d(Eigen::Quaterniond(R), t);
    }
    // std::cout << t << std::endl;

    camera_rig.AddCamera(
        idx + 1, rig_0 * colmap::Inverse(Rigid3d(Eigen::Quaterniond(R), t)));
    // idx + 1,  colmap::Inverse(Rigid3d(Eigen::Quaterniond(R *
    // R_0.transpose()), t)));
    std::cout << idx + 1 << ", "
              << camera_rig.CamFromRig(idx + 1).rotation.toRotationMatrix()
              << std::endl;
  }
  std::cout << "AddCamera done" << std::endl;

  // Add snapshot to the CameraRig
  // std::unordered_map<image_t, int> image_id_to_snapshot_idx;
  std::unordered_map<int, std::vector<image_t>> snapshot_idx_to_image_ids;
  for (auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    int str_len = image.file_name.size();

    int sequence_idx = std::stoi(image.file_name.substr(str_len - 4 - 7, 7));
    snapshot_idx_to_image_ids[sequence_idx].emplace_back(image_id);
  }

  for (const auto& [snapshot_idx, image_ids] : snapshot_idx_to_image_ids) {
    camera_rig.AddSnapshot(image_ids);
  }
  for (size_t i = 0; i < camera_rig.Snapshots().size(); i++) {
    std::cout << i;
    for (size_t j = 0; j < camera_rig.Snapshots()[i].size(); j++) {
      image_t image_id = camera_rig.Snapshots()[i][j];
      std::cout << ", " << image_id << ", " << images[image_id].file_name;
    }
    std::cout << std::endl;
  }

  // Cameras are added to the snapshots

  // std::cout << calib["cams"]["cam0"]["R"] << std::endl;

  // camera_rig.

  // --------------------------------------------------------------

  // Establish rigs

  GlobalMapperOptions options;

  // Run the relative pose estimation and establish tracks
  options.skip_preprocessing = false;
  options.skip_view_graph_calibration = false;
  options.skip_relative_pose_estimation = true;
  options.skip_rotation_averaging = false;
  options.skip_track_establishment = false;

  options.skip_global_positioning = true;
  options.skip_bundle_adjustment = true;
  options.skip_retriangulation = true;
  options.skip_pruning = true;

  options.inlier_thresholds.min_inlier_num = 30;
  options.inlier_thresholds.max_epipolar_error_E = 1.;

  options.opt_ba.solver_options.max_num_iterations = 200;


  options.opt_track.min_num_tracks_per_view = 50;

  // if (argc > 3)
  //     options.use_ = (std::stoi(argv[3]) > 0);
  // else
  //     options.use_ = true;

  // if (argc > 4)
  //     options.num_ite_gp_ = std::stoi(argv[4]);
  // else
  //     options.num_ite_gp_ = 3;

  // if (argc > 5)
  //     options.thres_gp_ = std::stod(argv[5]);
  // else
  //     options.thres_gp_ = 0.2;

  // if (argc > 6)
  //     options.opt_gp.solver_options.max_num_iterations = std::stoi(argv[6]);
  // else
  //     options.opt_gp.solver_options.max_num_iterations = 5;

  colmap::Timer run_timer;
  run_timer.Start();

  GlobalMapper global_mapper(options);
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  //   WriteGlomapReconstruction(
  //     "test_2", cameras, images, tracks, "bin", "");

  //   return 0;

  // TODO: solve the global rotation with the camera rig
  // Can easily do so by adding new cameras and using new view graph with new
  // image pairs (using the average rotation?)

  // Check whether the local rotation is consistent with the global rotation
  // Check the first snapshot
  for (int i = 0; i < camera_rig.NumSnapshots(); i++) {
    Rigid3d rig_from_world = camera_rig.ComputeRigFromWorld(i, images);
    for (int j = 0; j < camera_rig.Snapshots()[i].size(); j++) {
      image_t image_id = camera_rig.Snapshots()[i][j];
      Rigid3d cam_from_world = images[image_id].cam_from_world;
      Rigid3d cam_from_rig = camera_rig.CamFromRig(images[image_id].camera_id);

      images[image_id].cam_from_world = cam_from_rig * rig_from_world;
      std::cout << CalcAngle(cam_from_world, cam_from_rig * rig_from_world)
                << " ";
      // std::cout << "image_id: " << image_id << std::endl;
      // std::cout << "cam_from_rig (store): " <<
      // cam_from_rig.rotation.toRotationMatrix() << std::endl; std::cout <<
      // "cam_from_rig: " << (cam_from_world *
      // colmap::Inverse(rig_from_world)).rotation.toRotationMatrix() <<
      // std::endl;
    }
    std::cout << std::endl;

    // for (int j = 0; j < camera_rig.Snapshots()[i].size(); j++) {
    //     image_t image_id_1 = camera_rig.Snapshots()[i][j];
    //     Rigid3d cam_from_rig_1 =
    //     camera_rig.CamFromRig(images[image_id_1].camera_id); Rigid3d
    //     cam_from_world_1 = images[image_id_1].cam_from_world; for (int k = j
    //     + 1; k < camera_rig.Snapshots()[i].size(); k++) {
    //         image_t image_id_2 = camera_rig.Snapshots()[i][k];
    //         Rigid3d cam_from_rig_2 =
    //         camera_rig.CamFromRig(images[image_id_2].camera_id); Rigid3d
    //         cam_from_world_2 = images[image_id_2].cam_from_world; std::cout
    //         << "image_id1, image_id2: " << image_id_1 << ", " << image_id_2
    //         << std::endl; std::cout << "camera_id1, camera_id2: " <<
    //         images[image_id_1].camera_id << ", " <<
    //         images[image_id_2].camera_id << std::endl; std::cout <<
    //         "cam_from_rig_2 * cam_from_rig_1.T" << (cam_from_rig_2 *
    //         colmap::Inverse(cam_from_rig_1)).rotation.toRotationMatrix() <<
    //         std::endl; std::cout << "cam_from_world_2 * cam_from_world_1.T"
    //         << (cam_from_world_2 *
    //         colmap::Inverse(cam_from_world_1)).rotation.toRotationMatrix() <<
    //         std::endl;
    //     }
    // }
  }

  //   colmap::Reconstruction recontruction;
  //   recontruction.Read("test_2/0");

  //   ConvertColmapToGlomap(recontruction, cameras, images, tracks);
  //   UndistortImages(cameras, images);

  std::cout << "Camera Rotations solved" << std::endl;

  RigGlobalPositionerOptions options_rig;

  RigGlobalPositioner rig_global_positioner(options_rig);

  for (auto& [track_id, track] : tracks) {
    track.is_initialized = true;
  }
  // options_rig.verbose = true;

  rig_global_positioner.Solve(view_graph, camera_rigs, cameras, images, tracks);

  run_timer.Pause();

  options.skip_preprocessing = true;
  options.skip_view_graph_calibration = true;
  options.skip_relative_pose_estimation = true;
  options.skip_rotation_averaging = true;
  options.skip_track_establishment = true;

  options.skip_global_positioning = true;
  options.skip_bundle_adjustment = true;
  options.skip_retriangulation = true;
  options.skip_pruning = true;

  options.num_iteration_bundle_adjustment = 0;

  GlobalMapper global_mapper_new(options);
  global_mapper_new.Solve(database, view_graph, cameras, images, tracks);

  WriteGlomapReconstruction("test_3", cameras, images, tracks, "bin", "");
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

  WriteGlomapReconstruction(argv[2], cameras, images, tracks, "bin", "");

  return 0;
};
