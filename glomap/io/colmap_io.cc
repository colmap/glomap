#include "glomap/io/colmap_io.h"

#include <sys/stat.h>

namespace {
#define MKDIR(PATH) ::mkdir(PATH, 0755)

static bool do_mkdir(const std::string& path) {
  struct stat st;
  if (::stat(path.c_str(), &st) != 0) {
    if (MKDIR(path.c_str()) != 0 && errno != EEXIST) {
      return false;
    }
  } else if (!S_ISDIR(st.st_mode)) {
    errno = ENOTDIR;
    return false;
  }
  return true;
}

bool mkpath(std::string path) {
  std::string build;
  for (size_t pos = 0; (pos = path.find('/')) != std::string::npos;) {
    build += path.substr(0, pos + 1);
    do_mkdir(build);
    path.erase(0, pos + 1);
  }
  if (!path.empty()) {
    build += path;
    do_mkdir(build);
  }
  return true;
}
}  // namespace

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks) {
  colmap::Reconstruction reconstruction;
  ConvertGlomapToColmap(cameras, images, tracks, reconstruction);

  mkpath(reconstruction_path);
  reconstruction.WriteText(reconstruction_path);
}

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction) {
  mkpath(reconstruction_path);
  reconstruction.WriteText(reconstruction_path);
}
}  // namespace glomap
