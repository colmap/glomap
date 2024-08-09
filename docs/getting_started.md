# Getting started

## Installation and end-to-end examples

Please refer to the main `README.md`.

## Recommended settings

The default parameters do not always gaurantee satisfying reconstructions.
Regarding this, there are several things which can generally help.

### Share camera parameters

If images are known to be taken with the same physical camera under identical
camera settings, or images are well organized and known to be taken by several
cameras, it is higly recommended to share the camera intrinsics as appropriate.
To achieve this, one can set `--ImageReader.single_camera_per_folder` or
`--ImageReader.single_camera_per_image` in `colmap feature_extractor`.

### Handle high-resolution or blurry images

If images have high resolution or are blurry, it is worth trying to increase the
allowed epipolar error by modifying `--RelPoseEstimation.max_epipolar_error`.
For example, increase it to 4 or 10.

### Speedup reconstruction process

#### Cap the number of tracks

If the number of images and points are large, the run-time of global bundle
adjustment can be long. In this case, to further speed up the overall
reconstruction process, the total number of points can be capped, by changing
`--TrackEstablishment.max_num_tracks`. Typically, one image should not need more
than 1000 tracks to achieve good performance, so this number can be adjusted to
$1000 \times n$. Afterwards, if a full point cloud is desired (for example, to
initialize a Gaussian Splatting), points can be triangulated directly by calling
`colmap point_triangulator`.

Note, if the `--skip_retriangulation` is not set when calling `glomap mapper`,
retriangulation should already been performed.

#### Limit optimization iterations

The number of global positioning and bundle adjustment iterations can be limited
using the `--GlobalPositioning.max_num_iterations` and
`--BundleAdjustment.max_num_iterations` options. 
