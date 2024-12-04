#pragma once

#include "glomap/colmap_migration/pose.h"
#include "glomap/scene/types_sfm.h"

namespace glomap {

    Sim3d NormalizeReconstruction(
        std::unordered_map<camera_t, Camera>& cameras,
        std::unordered_map<image_t, migration::Image>& images,
        std::unordered_map<track_t, migration::Track>& tracks,
        bool fixed_scale = false,
        double extent = 10.,
        double p0 = 0.1,
        double p1 = 0.9);
} // namespace glomap
