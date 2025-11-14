#pragma once

#include "glomap/scene/types_sfm.h"

#include <colmap/geometry/gps.h>
#include <colmap/optim/ransac.h>

namespace glomap {
// Applies a similarity transformation to align the reconstruction with the
// position prior.
bool AlignReconstructionToPosePositionPriors(
    const std::unordered_map<image_t, colmap::PosePrior>& pose_priors,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks,
    const colmap::RANSACOptions& ransac_options);
}  // namespace glomap