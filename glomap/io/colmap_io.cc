#include "glomap/io/colmap_io.h"

#include <colmap/util/misc.h>

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks) {
  colmap::Reconstruction reconstruction;
  ConvertGlomapToColmap(cameras, images, tracks, reconstruction);

  colmap::CreateDirIfNotExists(reconstruction_path);
  reconstruction.WriteText(reconstruction_path);
}

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction) {
  colmap::CreateDirIfNotExists(reconstruction_path);
  reconstruction.WriteText(reconstruction_path);
}

}  // namespace glomap
