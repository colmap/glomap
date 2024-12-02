#pragma once

#include <string>
#include "glomap/colmap_migration/types.h"
#include "glomap/scene/types.h"
#include "glomap/scene/track.h"
#include "glomap/scene/image.h"
#include "glomap/scene/camera.h"


namespace glomap {
    struct Reconstruction;

    void WriteGlomapReconstruction(
        const std::string& reconstruction_path,
        const std::unordered_map<camera_t, Camera>& cameras,
        const std::unordered_map<image_t, Image>& images,
        const std::unordered_map<glomap::track_t, Track>& tracks,
        const std::string output_format = "bin",
        const std::string image_path = "");

    void WriteColmapReconstruction(const std::string& reconstruction_path,
                                   const Reconstruction& reconstruction,
                                   const std::string output_format = "bin");

} // namespace glomap
