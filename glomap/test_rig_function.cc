

#include "glomap/controllers/rig_global_mapper.h"
#include "glomap/io/colmap_converter.h"
#include "glomap/io/colmap_io.h"
#include "glomap/io/pose_io.h"
#include "glomap/io/utils.h"
#include "glomap/json.h"
#include "glomap/processors/image_pair_inliers.h"
#include "glomap/processors/image_undistorter.h"
#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/processors/relpose_filter.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <colmap/util/string.h>
#include <colmap/util/timer.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <string>

using json = nlohmann::json;

using namespace glomap;
int main(int argc, char** argv) {
  colmap::InitializeGlog(argv);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 2;

  // staring_pose:
  //    - 0: from relative pose
  //    - 1: from global rotation + established tracks
  //    - 2: from global positioning result
  // TODO: need to specify the resume_path if we start from the middle of the
  // process
  int staring_pos = 0;
  std::string resume_path = "../../prague/after_gp/0";

  // LOG(INFO) << "argc: " << argc << std::endl;

  // std::string database_path;
  // database_path = argv[1];
  std::string database_path;
  database_path = "../../prague/db_undistorted.db";

  std::string image_list_path = "../../prague/image_list.txt";
  std::unordered_set<std::string> image_filenames;

  ReadImageList(image_list_path, image_filenames);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;
  colmap::Database database(database_path);

  // Load the database
  if (staring_pos == 0) {
    ConvertDatabaseToGlomap(
        database, view_graph, cameras, images, &image_filenames);
    std::cout << "Loaded database" << std::endl;
    ReadRelPose(
        "../../prague/relpose_undistorted.txt", images, view_graph, false);
  } else {
    view_graph.image_pairs.clear();
    colmap::Reconstruction reconstruction;
    reconstruction.Read(resume_path);
    ConvertColmapToGlomap(reconstruction, cameras, images, tracks);
  }

  // // --------------------------------------------------------------
  // // For experiment, keep only 10 images for each sequence
  // int kept_img = 10;
  // std::unordered_map<camera_t, std::vector<image_t>> camera_id_to_image_id;
  // for (auto& [camera_id, camera] : cameras) {
  //   camera_id_to_image_id[camera_id] = std::vector<image_t>();
  // }

  // for (auto& [image_id, image] : images) {
  //   if (image.is_registered == false) continue;
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

  if (staring_pos == 0) {
    int num_img = view_graph.KeepLargestConnectedComponents(images);
    std::cout << "KeepLargestConnectedComponents done" << std::endl;
    std::cout << "num_img: " << num_img << std::endl;
  }

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
  std::unordered_map<std::string, std::array<image_t, 6>>
      snapshot_key_to_image_ids;
  for (auto& [image_id, image] : images) {
    const std::string& name = image.file_name;

    std::size_t cam_pos =
        name.rfind("_cam");  // we may have two times _cam in the name :(
    int cam_idx = name[cam_pos + 4] - '0';
    image.camera_id = cam_idx + 1;

    // handle image names like:
    // reel_0017_20240117_cam0_0000000.jpg
    // reel_0049_20231107-121638_courtyard_MX_XVN_warning_cam_is_180_cam3_0000000.jpg
    // and we want to get a key like:
    //"reel_0017_20240117_0000000"
    //"reel_0049_20231107-121638_courtyard_MX_XVN_warning_cam_is_180_0000000"

    std::size_t prefix_end = cam_pos;
    std::size_t suffix_start = name.find('_', cam_pos + 5);  // after "camN"
    std::string snapshot_key =
        name.substr(4, prefix_end) + name.substr(suffix_start);

    snapshot_key_to_image_ids[snapshot_key][cam_idx] = image_id;
  }

  std::unordered_map<std::string, std::vector<image_t>>
      snapshot_key_to_image_ids_vector;

  for (const auto& [snapshot_key, image_ids] : snapshot_key_to_image_ids) {
    snapshot_key_to_image_ids_vector[snapshot_key].clear();
    for (int i = 0; i < 6; i++) {
      if (image_ids[i] != 0) {
        snapshot_key_to_image_ids_vector[snapshot_key].push_back(image_ids[i]);
      }
    }
  }

  for (const auto& [snapshot_key, image_ids] :
       snapshot_key_to_image_ids_vector) {
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

  options.skip_global_positioning = true;
  options.skip_bundle_adjustment = true;
  options.skip_retriangulation = true;
  options.skip_pruning = true;

  options.inlier_thresholds.min_inlier_num = 30;
  options.inlier_thresholds.max_epipolar_error_E = 1.;

  options.opt_ba.solver_options.max_num_iterations = 200;

  options.opt_track.min_num_tracks_per_view = 300;

  // for (auto &[image_id, image] : images) {
  //   if (image.is_registered) continue;
  //   std::cout << image.camera_id << " " << image.file_name << std::endl;
  // }

  colmap::Timer run_timer;
  run_timer.Start();
  options.opt_gp.use_gpu = false;

  options.opt_ba.use_gpu = false;

  if (staring_pos == 0) {
    InlierThresholdOptions inlier_thresholds = options.inlier_thresholds;
    // Undistort the images and filter edges by inlier number
    UndistortImages(cameras, images, true);
    ImagePairsInlierCount(view_graph, cameras, images, inlier_thresholds, true);

    RelPoseFilter::FilterInlierNum(view_graph,
                                   options.inlier_thresholds.min_inlier_num);
    RelPoseFilter::FilterInlierRatio(
        view_graph, options.inlier_thresholds.min_inlier_ratio);

    // Run the RA and establish tracks
    RigGlobalMapper global_mapper_track(options);
    global_mapper_track.Solve(
        database, view_graph, camera_rigs, cameras, images, tracks);

    WriteGlomapReconstruction(
        "../../prague/after_track", cameras, images, tracks, "bin", "");
  }

  if (staring_pos <= 1) {
    options.skip_rotation_averaging = true;
    options.skip_track_establishment = true;
    options.skip_global_positioning = false;

    RigGlobalMapper global_mapper_gp(options);
    global_mapper_gp.Solve(
        database, view_graph, camera_rigs, cameras, images, tracks);

    WriteGlomapReconstruction(
        "../../prague/after_gp", cameras, images, tracks, "bin", "");
  }

  options.skip_rotation_averaging = true;
  options.skip_track_establishment = true;
  options.skip_global_positioning = true;
  options.skip_bundle_adjustment = false;

  RigGlobalMapper global_mapper_ba(options);
  global_mapper_ba.Solve(
      database, view_graph, camera_rigs, cameras, images, tracks);

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  WriteGlomapReconstruction(
      "../../prague/glomap_undistorted", cameras, images, tracks, "bin", "");
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
