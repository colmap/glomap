#ifndef GLOMAP_PROCESSOR_IMAGE_UNDISTORTER_H_
#define GLOMAP_PROCESSOR_IMAGE_UNDISTORTER_H_

#include "glomap/scene/types_sfm.h"

namespace glomap {

void UndistortImages(std::unordered_map<camera_t, Camera>& cameras,
                     std::unordered_map<image_t, Image>& images,
                     bool clean_points = true);

}  // namespace glomap

#endif  // GLOMAP_PROCESSOR_IMAGE_UNDISTORTER_H_