#pragma once

#include <string>
#include <unordered_set>

namespace glomap {
// Read a list of image filenames from a file.
// Required data structure:
// IMAGE_NAME_1
// IMAGE_NAME_2
// ...
// IMAGE_NAME_N
void ReadImageList(const std::string& file_path,
                   std::unordered_set<std::string>& image_filenames);
}  // namespace glomap
