#pragma once
#include "glomap/scene/camera.h"

namespace glomap {

double AdaptiveFactor(const Camera& camera_1, const Camera& camera_2, double standard_size=1024.);

} // glomap
