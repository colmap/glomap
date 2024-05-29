
#pragma once
#ifndef GLOMAP_CONTROLLERS_TRACK_RETRIANGULATION_H_
#define GLOMAP_CONTROLLERS_TRACK_RETRIANGULATION_H_

#include <glomap/scene/types_sfm.h>

namespace glomap {

struct TriangulatorOptions {
    double tri_complete_max_reproj_error = 15.0;
    double tri_merge_max_reproj_error = 15.0;
    double tri_min_angle = 1.0;

    size_t min_num_matches = 15;

    std::string database_dir = "";
};

bool RetriangulateTracks(const TriangulatorOptions& options,
                        std::unordered_map<camera_t, Camera>& cameras,
                        std::unordered_map<image_t, Image>& images,
                        std::unordered_map<track_t, Track>& tracks);

}; // glomap

#endif  // GLOMAP_CONTROLLERS_TRACK_RETRIANGULATION_H_