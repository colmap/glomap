#include "glomap/io/colmap_io.h"

#include <colmap/util/misc.h>

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks,
    const std::string output_format) {
  // Check whether reconstruction pruning is applied.
  // If so, export seperate reconstruction
  int largest_component_num = -1;
  for (const auto& [image_id, image] : images) {
    if (image.cluster_id > largest_component_num)
      largest_component_num = image.cluster_id;
  }
  // If it is not seperated into several clusters, then output them as whole
  if (largest_component_num == -1) {
    colmap::Reconstruction reconstruction;
    ConvertGlomapToColmap(cameras, images, tracks, reconstruction);
    colmap::CreateDirIfNotExists(reconstruction_path + "/0");
    if (output_format == "txt") {
      reconstruction.WriteText(reconstruction_path + "/0");
    } else if (output_format == "bin") {
      reconstruction.WriteBinary(reconstruction_path + "/0");
    } else {
      LOG(ERROR) << "Unsupported output type" << std::endl;
    }
  } else {
    for (int comp = 0; comp <= largest_component_num; comp++) {
      std::cout << "\r Exporting reconstruction " << comp + 1 << " / "
                << largest_component_num + 1 << std::flush;
      colmap::Reconstruction reconstruction;
      ConvertGlomapToColmap(cameras, images, tracks, reconstruction, comp);
      colmap::CreateDirIfNotExists(reconstruction_path + "/" +
                                   std::to_string(comp));
      reconstruction.WriteText(reconstruction_path + "/" +
                               std::to_string(comp));
      if (output_format == "txt") {
        reconstruction.WriteText(reconstruction_path + "/" +
                                 std::to_string(comp));
      } else if (output_format == "bin") {
        reconstruction.WriteBinary(reconstruction_path + "/" +
                                 std::to_string(comp));
      } else {
        LOG(ERROR) << "Unsupported output type" << std::endl;
      }
    }
    std::cout << std::endl;
  }
}

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction,
                               const std::string output_format) {
  colmap::CreateDirIfNotExists(reconstruction_path);
  if (output_format == "txt") {
    reconstruction.WriteText(reconstruction_path);
  } else if (output_format == "bin") {
    reconstruction.WriteBinary(reconstruction_path);
  } else {
    LOG(ERROR) << "Unsupported output type" << std::endl;
  }
}

}  // namespace glomap
