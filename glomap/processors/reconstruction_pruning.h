
#pragma once
#ifndef GLOMAP_PROCESSORS_RECONSTRUCTION_PRUNING_H_
#define GLOMAP_PROCESSORS_RECONSTRUCTION_PRUNING_H_

#include "glomap/scene/types_sfm.h"

namespace glomap {
image_t PruneWeaklyConnectedImages(std::unordered_map<image_t, Image>& images,
                                   std::unordered_map<track_t, Track>& tracks,
                                   int min_num_observations = 0);
}  // namespace glomap

#endif  // GLOMAP_PROCESSORS_RECONSTRUCTION_PRUNING_H_