#pragma once

#include "glomap/io/colmap_converter.h"
#include "glomap/scene/types_sfm.h"

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks);

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction);

}  // namespace glomap
