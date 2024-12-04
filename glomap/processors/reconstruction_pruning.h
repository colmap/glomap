
#pragma once

#include "glomap/scene/types_sfm.h"

namespace glomap {

    image_t PruneWeaklyConnectedImages(std::unordered_map<image_t, migration::Image>& images,
                                       std::unordered_map<track_t, migration::Track>& tracks,
                                       int min_num_images = 2,
                                       int min_num_observations = 0);

} // namespace glomap
