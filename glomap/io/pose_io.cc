#include "pose_io.h"

#include <fstream>

namespace glomap {
void ReadRelPose(const std::string& file_path,
                 std::unordered_map<image_t, Image>& images,
                 ViewGraph& view_graph) {
  std::unordered_map<std::string, image_t> name_idx;
  image_t max_image_id = 0;
  std::unordered_set<image_t> existing_images;
  for (const auto& [image_id, image] : images) {
    name_idx[image.file_name] = image_id;
    existing_images.insert(image_id);

    max_image_id = std::max(max_image_id, image_id);
  }

  std::ifstream file(file_path);

  // Read in data
  std::string line;
  std::string item;

  size_t counter = 0;

  // Required data structures
  // IMAGE_NAME_1 IMAGE_NAME_2 QW QX QY QZ TX TY TZ
  while (std::getline(file, line)) {
    std::stringstream line_stream(line);

    std::string file1, file2;
    std::getline(line_stream, item, ' ');
    file1 = item;
    std::getline(line_stream, item, ' ');
    file2 = item;

    if (name_idx.find(file1) == name_idx.end()) {
      max_image_id += 1;
      images.insert(
          std::make_pair(max_image_id, Image(max_image_id, -1, file1)));
      name_idx[file1] = max_image_id;
    }
    if (name_idx.find(file2) == name_idx.end()) {
      max_image_id += 1;
      images.insert(
          std::make_pair(max_image_id, Image(max_image_id, -1, file2)));
      name_idx[file2] = max_image_id;
    }

    image_t index1 = name_idx[file1];
    image_t index2 = name_idx[file2];

    image_pair_t pair_id = ImagePair::ImagePairToPairId(index1, index2);

    // rotation
    Rigid3d pose_rel;
    for (int i = 0; i < 4; i++) {
      std::getline(line_stream, item, ' ');
      pose_rel.rotation.coeffs()[(i + 3) % 4] = std::stod(item);
    }

    for (int i = 0; i < 3; i++) {
      std::getline(line_stream, item, ' ');
      pose_rel.translation[i] = std::stod(item);
    }

    view_graph.image_pairs.insert(
        std::make_pair(pair_id, ImagePair(index1, index2, pose_rel)));
    counter++;
  }
  LOG(INFO) << counter << " relpose are loaded" << std::endl;
}

void WriteGlobalRotation(const std::string& file_path,
                         const std::unordered_map<image_t, Image>& images) {
  std::ofstream file(file_path);
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    file << image.file_name << " ";
    for (int i = 0; i < 4; i++) {
      file << image.cam_from_world.rotation.coeffs()[(i + 3) % 4] << " ";
    }
    file << "\n";
  }
}
}  // namespace glomap