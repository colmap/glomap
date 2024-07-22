#include "glomap/io/colmap_io.h"

#include <colmap/util/misc.h>

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks) {
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
    reconstruction.WriteText(reconstruction_path + "/0");
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
    }
    std::cout << std::endl;
  }
}

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction) {
  colmap::CreateDirIfNotExists(reconstruction_path);
  reconstruction.WriteText(reconstruction_path);
}

}  // namespace glomap
