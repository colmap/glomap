#include "adaptive_scaler.h"

namespace glomap {

double AdaptiveFactor(const Camera& camera_1, const Camera& camera_2, double standard_size) {

    double cam_size1 = camera_1.Focal();
    double cam_size2 = camera_2.Focal();
    double cam_size_max = std::max(cam_size1, cam_size2);

    double scaler = std::max(8., std::min(0.5, cam_size_max / standard_size));

    return scaler;
}

} // glomap