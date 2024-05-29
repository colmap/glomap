#include "image_undistorter.h"

namespace glomap {

void UndistortImages(std::unordered_map<camera_t, Camera>& cameras,
                        std::unordered_map<image_t, Image>& images,
                        bool clean_points) {

    Eigen::Vector2d pt_undist;
    Eigen::Vector3d pt_undist_norm;

    for (auto &[image_id, image] : images) {
        int camera_id = image.camera_id;
        Camera& camera = cameras[camera_id];
        int num_points = image.features.size();

        if (image.features_undist.size() == num_points && !clean_points)
            continue; // already undistorted

        image.features_undist.clear();
        image.features_undist.reserve(num_points);
        for (int i = 0; i < num_points; i++) {
            // Undistort point in image
            pt_undist = camera.CamFromImg(image.features[i]);

            pt_undist_norm = pt_undist.homogeneous().normalized();
            image.features_undist.emplace_back(pt_undist_norm);
        }
    }
}

}; // glomap
