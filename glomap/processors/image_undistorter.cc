#include "glomap/processors/image_undistorter.h"

namespace glomap {

void UndistortImages(std::unordered_map<camera_t, Camera>& cameras,
                     std::unordered_map<image_t, Image>& images,
                     bool clean_points) {
  std::vector<image_t> image_ids;
  for (auto& [image_id, image] : images) {
    int num_points = image.features.size();

    if (image.features_undist.size() == num_points && !clean_points)
      continue;  // already undistorted
    image_ids.push_back(image_id);
  }

  LOG(INFO) << "Undistorting images..";
  const int num_images = image_ids.size();
#pragma omp parallel for
  for (int image_idx = 0; image_idx < num_images; image_idx++) {
    Image& image = images[image_ids[image_idx]];

    int camera_id = image.camera_id;
    Camera& camera = cameras[camera_id];
    int num_points = image.features.size();

    if (image.features_undist.size() == num_points && !clean_points)
      continue;  // already undistorted

    image.features_undist.clear();
    image.features_undist.reserve(num_points);
    for (int i = 0; i < num_points; i++) {
      image.features_undist.emplace_back(
          camera.CamFromImg(image.features[i]).homogeneous().normalized());
    }
  }
  LOG(INFO) << "Image undistortion done";
}

}  // namespace glomap
