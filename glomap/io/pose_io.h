#pragma once

#include "glomap/scene/types_sfm.h"

#include <unordered_map>

namespace glomap {
void ReadRelPose(const std::string& file_path,
                 std::unordered_map<image_t, Image>& images,
                 ViewGraph& view_graph);

void WriteGlobalRotation(const std::string& file_path,
                         const std::unordered_map<image_t, Image>& images);
}  // namespace glomap