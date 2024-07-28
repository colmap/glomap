#pragma once

#include "glomap/scene/image.h"
#include <unordered_map>

namespace glomap {
// Require the gravity in the format: image_name, gravity (3 numbers)
// Gravity should be the direction of [0,1,0] in the image frame
// image.cam_from_world * [0,1,0]^T = g
void ReadGravity(const std::string& gravity_path,
                 std::unordered_map<image_t, Image>& images);

}  // namespace glomap