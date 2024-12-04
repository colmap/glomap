#pragma once

#include "glomap/colmap_migration/camera.h"
#include "glomap/colmap_migration/types.h"
#include "glomap/scene/image.h"
#include "glomap/scene/track.h"
#include "glomap/scene/types.h"

#include <string>

namespace glomap {
    struct Reconstruction;

    void WriteGlomapReconstruction(
        const std::string& reconstruction_path,
        const std::unordered_map<camera_t, Camera>& cameras,
        const std::unordered_map<image_t, migration::Image>& images,
        const std::unordered_map<glomap::track_t, Track>& tracks,
        const std::string output_format = "bin",
        const std::string image_path = "");

    void WriteColmapReconstruction(const std::string& reconstruction_path,
                                   const Reconstruction& reconstruction,
                                   const std::string output_format = "bin");

} // namespace glomap
