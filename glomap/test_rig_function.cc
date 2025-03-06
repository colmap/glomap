
#include "glomap/io/colmap_converter.h"
// #include "glomap/io/theia_converter.h"
#include "glomap/io/colmap_io.h"
#include "glomap/controllers/global_mapper_stochastic.h"
#include "glomap/test/prepare_experiment.h"
#include "glomap/processors/reconstruction_pruning.h"

#include "glomap/estimators/callback_functions.h"

#include "glomap/types.h"

#include <colmap/util/timer.h>

#include <string> 

#include <chrono>
#include <ctime>
#include <fstream>

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

using namespace glomap;
int main(int argc, char** argv) {
    colmap::InitializeGlog(argv);
    FLAGS_alsologtostderr = true;
    FLAGS_v = 0;

    LOG(INFO) << "argc: " << argc << std::endl;

    std::string database_path;
    database_path = argv[1];


    ViewGraph view_graph;
    std::unordered_map<camera_t, Camera> cameras;
    std::unordered_map<image_t, Image> images;
    std::unordered_map<track_t, Track> tracks;

    // Load the database
    colmap::Database database(database_path);
    ConvertDatabaseToGlomap(database, view_graph, cameras, images);
    std::cout << "Loaded database" << std::endl;

    int num_img = view_graph.KeepLargestConnectedComponents(images);
    std::cout << "KeepLargestConnectedComponents done" << std::endl;
    std::cout << "num_img: " << num_img << std::endl;

    GlobalMapperStochasticOptions options;

    // Run the relative pose estimation and establish tracks
    options.skip_preprocessing = false;
    options.skip_view_graph_calibration = false;
    options.skip_relative_pose_estimation = false;
    options.skip_rotation_averaging = false;
    options.skip_track_establishment = false;
    options.skip_global_positioning = false;
    options.skip_bundle_adjustment = true;
    options.skip_retriangulation = true;
    options.skip_pruning = true;


    options.inlier_thresholds.min_inlier_num = 30;
    options.inlier_thresholds.max_epipolar_error_E = 1.;

    options.opt_ba.solver_options.max_num_iterations = 200;

    // if (argc > 3)
    //     options.use_stochastic = (std::stoi(argv[3]) > 0);
    // else
    //     options.use_stochastic = true;
    
    // if (argc > 4)
    //     options.num_ite_gp_stochastic = std::stoi(argv[4]);
    // else
    //     options.num_ite_gp_stochastic = 3;

    // if (argc > 5)
    //     options.thres_gp_stochastic = std::stod(argv[5]);
    // else
    //     options.thres_gp_stochastic = 0.2;

    // if (argc > 6)
    //     options.opt_gp.solver_options.max_num_iterations = std::stoi(argv[6]);
    // else
    //     options.opt_gp.solver_options.max_num_iterations = 5;
    
    colmap::Timer run_timer;
    run_timer.Start();

    // GlobalMapperStochastic global_mapper(options);
    // global_mapper.Solve(database, view_graph, cameras, images, tracks);

    run_timer.Pause();

    // // -------------------------------------------------
    // std::ofstream file_rel;
    // file_rel.open("relpose_3dof_trans.txt");
    // std::unordered_map<image_pair_t, ImagePair>& image_pairs = view_graph.image_pairs;
    // std::vector<image_pair_t> image_pair_ids;
    // for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
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
    //     std::string pair_name = images[idx1].file_name + "-" + images[idx2].file_name;
    //     file_rel << pair_name << " " << image_pair.weight;
    //     for (int i = 0; i < 4; i++) {
    //         file_rel << " " << image_pair.cam2_from_cam1.rotation.coeffs()[i];
    //     }
    //     for (int i = 0; i < 3; i++) {
    //         file_rel << " " << image_pair.cam2_from_cam1.translation[i];
    //     }
    //     file_rel << "\n";

    // }
    // file_rel.close();
    // // -------------------------------------------------

    
    WriteGlomapReconstruction(
      argv[2], cameras, images, tracks, "bin", "");


    return 0;
};
