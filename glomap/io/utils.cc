#include "glomap/io/utils.h"

#include <fstream>

#include "colmap/util/string.h"

namespace glomap {
void ReadImageList(const std::string& file_path,
                   std::unordered_set<std::string>& image_filenames) {
  std::ifstream image_list_file(file_path);
  std::string line;
  while (std::getline(image_list_file, line)) {
    if (!line.empty()) {
      colmap::StringTrim(&line);
      image_filenames.insert(line);
    }
  }
  image_list_file.close();
}

}  // namespace glomap