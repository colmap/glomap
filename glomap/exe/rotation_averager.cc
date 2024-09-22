
#include "glomap/controllers/rotation_averager.h"

#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_io.h"
#include "glomap/io/gravity_io.h"
#include "glomap/io/pose_io.h"
#include "glomap/types.h"

#include <colmap/util/misc.h>
#include <colmap/util/timer.h>

namespace glomap {
// -------------------------------------
// Running Global Rotation Averager
// -------------------------------------
int RunRotationAverager(int argc, char** argv) {
  std::string relpose_path;
  std::string output_path;
  std::string gravity_path = "";

  bool use_stratified = true;

  OptionManager options;
  options.AddRequiredOption("relpose_path", &relpose_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("gravity_path", &gravity_path);
  options.AddDefaultOption("use_stratified", &use_stratified);

  options.Parse(argc, argv);

  if (!colmap::ExistsFile(relpose_path)) {
    LOG(ERROR) << "`relpose_path` is not a file";
    return EXIT_FAILURE;
  }

  if (gravity_path != "" && !colmap::ExistsFile(gravity_path)) {
    LOG(ERROR) << "`gravity_path` is not a file";
    return EXIT_FAILURE;
  }

  RotationAveragerOptions rotation_averager_options;
  rotation_averager_options.skip_initialization = true;
  rotation_averager_options.use_gravity = true;

  rotation_averager_options.use_stratified = use_stratified;

  // Load the database
  ViewGraph view_graph;
  std::unordered_map<image_t, Image> images;

  ReadRelPose(relpose_path, images, view_graph);

  if (gravity_path != "") {
    ReadGravity(gravity_path, images);
  }

  int num_img = view_graph.KeepLargestConnectedComponents(images);
  LOG(INFO) << num_img << " / " << images.size()
            << " are within the largest connected component";

  RotationAverager rotation_averager(rotation_averager_options);
  if (!rotation_averager.Solve(view_graph, images)) {
    LOG(ERROR) << "Failed to solve global rotation averaging";
    return EXIT_FAILURE;
  }

  // Write out the estimated rotation
  WriteGlobalRotation(output_path, images);
  LOG(INFO) << "Global rotation averaging done" << std::endl;

  return EXIT_SUCCESS;
}

}  // namespace glomap