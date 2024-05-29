#ifndef GLOMAP_PROCESSORS_IMAGE_PAIR_INLIERS_H_
#define GLOMAP_PROCESSORS_IMAGE_PAIR_INLIERS_H_

#include <glomap/scene/types_sfm.h>
#include <glomap/types.h>

#include "glomap/math/rigid3d.h"


namespace glomap {
    
class ImagePairInliers {
public:
    ImagePairInliers(ImagePair& image_pair,
                        const std::unordered_map<image_t, Image>& images,
                        const InlierThresholds& options,
                        const std::unordered_map<camera_t, Camera>* cameras = nullptr) :
                        image_pair(image_pair), images(images), cameras(cameras), options(options) {};

    // use the sampson error and put the inlier result into the image pair
    double ScoreError();

protected:
    // Error for the case of essential matrix
    double ScoreErrorEssential();

    // Error for the case of fundamental matrix
    double ScoreErrorFundamental();
    
    // Error for the case of homography matrix
    double ScoreErrorHomography();

    ImagePair& image_pair;
    const std::unordered_map<image_t, Image>& images;
    const std::unordered_map<camera_t, Camera>* cameras;
    const InlierThresholds& options;
};

void ImagePairsInlierCount(ViewGraph& view_graph,
                        const std::unordered_map<camera_t, Camera>& cameras,
                        const std::unordered_map<image_t, Image>& images,
                        const InlierThresholds& options,
                        bool clean_inliers);


}; // glomap
#endif // GLOMAP_PROCESSORS_IMAGE_PAIR_INLIERS_H_