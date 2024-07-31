# Getting started
### Installation and End-to-End Examples
Please refer to the main `README.md`

### Recommended practice
The default parameters do not always gaurantee satisfying reconstructions.
Regarding this, there are several things which can generally help

#### Share camera parameters as much as possible
If images are known to be taken with the same camera, or images are well organized and known to be taken by several cameras, it is higly recommended to share the camera intrinsics
To achieve this, one can set `--ImageReader.single_camera_per_folder` or `--ImageReader.single_camera_per_image` in `colmap feature_extractor` to be 1.

#### Allow larger epipolar error
If images are of high resolution, or are blurry, it is worth trying to increase the allowed epipolar error by modifying `--RelPoseEstimation.max_epipolar_error`. For example, make it 4, or 10.

#### Cap the number of tracks
If the number of images and points are large, the run-time of global bundle adjustment can be long. In this case, to further speed up the overall reconstruction process, the total number of points can be capped, by changing `--TrackEstablishment.max_num_tracks`. Typically, one image should not need more than 1000 tracks to achieve good performance, so this number can be adjusted to $1000 \times n$.
Afterwards, if a full point cloud is desired (for example, to initialize a Gaussian Splatting), points can be triangulated directly by calling `colmap point_triangulator`.

Note, if the `--skip_retriangulation` is not set true when calling `glomap mapper`, retriangulation should already been performed.
