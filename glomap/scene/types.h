#ifndef GLOMAP_SCENE_TYPES_H_
#define GLOMAP_SCENE_TYPES_H_

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace glomap {

////////////////////////////////////////////////////////////////////////////////
// Index types, determines the maximum number of objects.
////////////////////////////////////////////////////////////////////////////////

// Unique identifier for cameras.
typedef uint32_t camera_t;

// Unique identifier for images.
typedef uint32_t image_t;

// Each image pair gets a unique ID, see `Database::ImagePairToPairId`.
typedef uint64_t image_pair_t;

// Index per image, i.e. determines maximum number of 2D points per image.
typedef uint32_t feature_t;

// Unique identifier per added 3D point. Since we add many 3D points,
// delete them, and possibly re-add them again, the maximum number of allowed
// unique indices should be large.
typedef uint64_t track_t;

const image_t kMaxNumImages = std::numeric_limits<image_t>::max();
const image_pair_t kInvalidImagePairId = -1;

};      // namespace glomap
#endif  // GLOMAP_SCENE_TYPES_H_