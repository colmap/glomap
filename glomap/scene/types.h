#pragma once

#include <colmap/geometry/rigid3.h>
#include <colmap/scene/database.h>
#include <colmap/util/types.h>

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace glomap {

////////////////////////////////////////////////////////////////////////////////
// Index types, determines the maximum number of objects.
////////////////////////////////////////////////////////////////////////////////

// Unique identifier for cameras.
using colmap::camera_t;

// Unique identifier for images.
using colmap::image_t;

// Each image pair gets a unique ID, see `Database::ImagePairToPairId`.
typedef uint64_t image_pair_t;

// Index per image, i.e. determines maximum number of 2D points per image.
typedef uint32_t feature_t;

// Unique identifier per added 3D point. Since we add many 3D points,
// delete them, and possibly re-add them again, the maximum number of allowed
// unique indices should be large.
typedef uint64_t track_t;

using colmap::Rigid3d;

const image_t kMaxNumImages = colmap::Database::kMaxNumImages;
const image_pair_t kInvalidImagePairId = -1;

}  // namespace glomap
