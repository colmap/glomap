#ifndef GLOMAP_TYPES_H_
#define GLOMAP_TYPES_H_

#include <Eigen/Core>

#include <string> 
#include <vector>
#include <iostream>

#include <limits>

namespace glomap {

constexpr double EPS = 1e-12;
constexpr double HALF_PI = 1.57079632679; // pi / 2
constexpr double TWO_PI = 6.28318530718; // 2 * pi


struct InlierThresholds {
    // Thresholds for 3D-2D matches
    double max_angle_error = 1.; // in degree, for global positioning
    double max_reprojection_error = 1e-2; // for bundle adjustment

    // Thresholds for image_pair
    double max_epipolar_error_E_RANSAC = 1.;
    double max_epipolar_error_E = 1.;
    double max_epipolar_error_F = 4.;
    double max_epipolar_error_H = 4.;

    // Thresholds for edges
    double min_inlier_num = 30;
    double min_inlier_ratio = 0.25;
    double max_roation_error = 10.; // in degree, for rotation averaging

};

}; // glomap
#endif // GLOMAP_TYPES_H_