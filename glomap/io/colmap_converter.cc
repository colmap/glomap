#include "glomap/io/colmap_converter.h"

#include "glomap/math/two_view_geometry.h"
#include <cmath>  // For trigonometric functions and M_PI
#include <limits>
#include <algorithm>

namespace glomap {

void ConvertGlomapToColmapImage(const Image& image,
                                colmap::Image& image_colmap,
                                bool keep_points) {
  image_colmap.SetImageId(image.image_id);
  image_colmap.SetCameraId(image.camera_id);
  image_colmap.SetName(image.file_name);
  if (image.is_registered) {
    image_colmap.SetCamFromWorld(image.cam_from_world);
  }

  if (keep_points) {
    image_colmap.SetPoints2D(image.features);
  }
}

void ConvertGlomapToColmap(const std::unordered_map<camera_t, Camera>& cameras,
                           const std::unordered_map<image_t, Image>& images,
                           const std::unordered_map<track_t, Track>& tracks,
                           colmap::Reconstruction& reconstruction,
                           int cluster_id,
                           bool include_image_points) {
  // Clear the colmap reconstruction
  reconstruction = colmap::Reconstruction();

  // Add cameras
  for (const auto& [camera_id, camera] : cameras) {
    reconstruction.AddCamera(camera);
  }

  // Prepare the 2d-3d correspondences
  size_t min_supports = 2;
  std::unordered_map<image_t, std::vector<track_t>> image_to_point3D;
  if (tracks.size() > 0 || include_image_points) {
    // Initialize every point to corresponds to invalid point
    for (auto& [image_id, image] : images) {
      if (!image.is_registered ||
          (cluster_id != -1 && image.cluster_id != cluster_id))
        continue;
      image_to_point3D[image_id] =
          std::vector<track_t>(image.features.size(), -1);
    }

    if (tracks.size() > 0) {
      for (auto& [track_id, track] : tracks) {
        if (track.observations.size() < min_supports) {
          continue;
        }
        for (auto& observation : track.observations) {
          if (image_to_point3D.find(observation.first) !=
              image_to_point3D.end()) {
            image_to_point3D[observation.first][observation.second] = track_id;
          }
        }
      }
    }
  }

  // Add points
  for (const auto& [track_id, track] : tracks) {
    colmap::Point3D colmap_point;
    colmap_point.xyz = track.xyz;
    colmap_point.color = track.color;
    colmap_point.error = 0;

    // Add track element
    for (auto& observation : track.observations) {
      const Image& image = images.at(observation.first);
      if (!image.is_registered ||
          (cluster_id != -1 && image.cluster_id != cluster_id))
        continue;
      colmap::TrackElement colmap_track_el;
      colmap_track_el.image_id = observation.first;
      colmap_track_el.point2D_idx = observation.second;

      colmap_point.track.AddElement(colmap_track_el);
    }

    if (track.observations.size() < min_supports) continue;

    colmap_point.track.Compress();
    reconstruction.AddPoint3D(track_id, std::move(colmap_point));
  }

  // Add images
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered ||
        (cluster_id != -1 && image.cluster_id != cluster_id))
      continue;

    colmap::Image image_colmap;
    bool keep_points =
        image_to_point3D.find(image_id) != image_to_point3D.end();
    ConvertGlomapToColmapImage(image, image_colmap, keep_points);
    if (keep_points) {
      std::vector<track_t>& track_ids = image_to_point3D[image_id];
      for (size_t i = 0; i < image.features.size(); i++) {
        if (track_ids[i] != -1) {
          image_colmap.SetPoint3DForPoint2D(i, track_ids[i]);
        }
      }
    }

    reconstruction.AddImage(std::move(image_colmap));
  }

  reconstruction.UpdatePoint3DErrors();
}

void ConvertColmapToGlomap(const colmap::Reconstruction& reconstruction,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           std::unordered_map<track_t, Track>& tracks) {
  // Clear the glomap reconstruction
  cameras.clear();
  images.clear();

  // Add cameras
  for (const auto& [camera_id, camera] : reconstruction.Cameras()) {
    cameras[camera_id] = camera;
  }

  for (auto& [image_id, image_colmap] : reconstruction.Images()) {
    auto ite = images.insert(std::make_pair(image_colmap.ImageId(),
                                            Image(image_colmap.ImageId(),
                                                  image_colmap.CameraId(),
                                                  image_colmap.Name())));

    Image& image = ite.first->second;
    image.is_registered = image_colmap.HasPose();
    if (image_colmap.HasPose()) {
      image.cam_from_world = static_cast<Rigid3d>(image_colmap.CamFromWorld());
    }
    image.features.clear();
    image.features.reserve(image_colmap.NumPoints2D());

    for (auto& point2D : image_colmap.Points2D()) {
      image.features.push_back(point2D.xy);
    }
  }

  ConvertColmapPoints3DToGlomapTracks(reconstruction, tracks);
}

void ConvertColmapPoints3DToGlomapTracks(
    const colmap::Reconstruction& reconstruction,
    std::unordered_map<track_t, Track>& tracks) {
  // Read tracks
  tracks.clear();
  tracks.reserve(reconstruction.NumPoints3D());

  auto& points3D = reconstruction.Points3D();
  for (auto& [point3d_id, point3D] : points3D) {
    Track track;
    const colmap::Track& track_colmap = point3D.track;
    track.xyz = point3D.xyz;
    track.color = point3D.color;
    track.track_id = point3d_id;
    track.is_initialized = true;

    const std::vector<colmap::TrackElement>& elements = track_colmap.Elements();
    track.observations.reserve(track_colmap.Length());
    for (auto& element : elements) {
      track.observations.push_back(
          Observation(element.image_id, element.point2D_idx));
    }

    tracks.insert(std::make_pair(point3d_id, track));
  }
}

// For ease of debug, go through the database twice: first extract the available
// pairs, then read matches from pairs.
void ConvertDatabaseToGlomap(const colmap::Database& database,
                             ViewGraph& view_graph,
                             std::unordered_map<camera_t, Camera>& cameras,
                             std::unordered_map<image_t, Image>& images,
                             bool extract_pose_priors) {
  // Add the images
  const std::vector<colmap::Image> images_colmap = database.ReadAllImages();

  // First pass: compute the mean / centre of all valid pose prior positions
  colmap::GPSTransform gps_transform(colmap::GPSTransform::WGS84);
  Eigen::Vector3d mean_prior_position = Eigen::Vector3d::Zero();
  size_t num_valid_priors = 0;
  // Rotation from ECEF to local ENU centred at the mean location. Will be
  // initialised once the statistics of the first pass are available.
  Eigen::Matrix3d R_ecef_to_enu = Eigen::Matrix3d::Identity();
  bool enu_transform_valid = false;
  // Variables to accumulate the bounding box of all valid prior positions (ENU coordinates)
  double min_east = std::numeric_limits<double>::max();
  double max_east = std::numeric_limits<double>::lowest();
  double min_north = std::numeric_limits<double>::max();
  double max_north = std::numeric_limits<double>::lowest();
  bool have_prior_area = false;
  if (extract_pose_priors) {
    for (const auto& image : images_colmap) {
      const image_t image_id = image.ImageId();
      if (image_id == colmap::kInvalidImageId) continue;

      colmap::PosePrior prior = database.ReadPosePrior(image_id);

      if (!prior.IsValid()) {
        continue;
      }

      // Transform prior position to XYZ if needed (duplicate of logic below, but
      // lightweight compared to restructuring the entire function).
      if (prior.coordinate_system == colmap::PosePrior::CoordinateSystem::WGS84) {
        prior.position =
            gps_transform.EllToXYZ(std::vector<Eigen::Vector3d>{prior.position}).front();
      }

      mean_prior_position += prior.position;
      num_valid_priors++;
    }

    if (num_valid_priors > 0) {
      mean_prior_position /= static_cast<double>(num_valid_priors);

      // Compute the reference latitude / longitude from the mean Cartesian
      // coordinates. These will be used to define the local ENU frame so that
      // the Z axis coincides with the Up direction (i.e. altitude).
      Eigen::Vector3d mean_ell =
          gps_transform
              .XYZToEll(std::vector<Eigen::Vector3d>{mean_prior_position})
              .front();

      const double lat0_rad = mean_ell(0) * M_PI / 180.0;
      const double lon0_rad = mean_ell(1) * M_PI / 180.0;

      const double sin_lat0 = std::sin(lat0_rad);
      const double cos_lat0 = std::cos(lat0_rad);
      const double sin_lon0 = std::sin(lon0_rad);
      const double cos_lon0 = std::cos(lon0_rad);

      // Rotation from ECEF to local ENU frame centred at the mean location.
      R_ecef_to_enu << -sin_lon0, cos_lon0, 0.,
          -sin_lat0 * cos_lon0, -sin_lat0 * sin_lon0, cos_lat0,
          cos_lat0 * cos_lon0, cos_lat0 * sin_lon0, sin_lat0;

      // Mark the ENU transform as valid for use in the subsequent pass.
      enu_transform_valid = true;

      LOG(INFO) << "Computed mean prior position from " << num_valid_priors
                << " images: " << mean_prior_position.transpose();
    } else {
      mean_prior_position.setZero();
    }
  }

  image_t counter = 0;
  for (auto& image : images_colmap) {
    std::cout << "\r Loading Images " << counter + 1 << " / "
              << images_colmap.size() << std::flush;
    counter++;

    const image_t image_id = image.ImageId();
    if (image_id == colmap::kInvalidImageId) continue;
    auto ite = images.insert(std::make_pair(
        image_id, Image(image_id, image.CameraId(), image.Name())));

    Image& inserted_image = ite.first->second;
    if (!extract_pose_priors) {
      continue;
    }

    colmap::PosePrior prior = database.ReadPosePrior(image_id);
    if (prior.IsValid()) {
      // Thransform prior position to XYZ if needed.
      if (prior.coordinate_system ==
          colmap::PosePrior::CoordinateSystem::WGS84) {
        prior.position =
            gps_transform.EllToXYZ(std::vector<Eigen::Vector3d>{prior.position})
                .front();
        // Continue processing below.
      } else if (prior.coordinate_system ==
                 colmap::PosePrior::CoordinateSystem::UNDEFINED) {
        LOG(WARNING) << "Unknown coordinate system for image " << image_id
                     << ", assuming cartesian.";
      }

      // Convert the Cartesian position to local ENU coordinates so that the Z
      // axis matches the altitude (Up) direction.
      if (enu_transform_valid) {
        prior.position = R_ecef_to_enu * (prior.position - mean_prior_position);
        prior.coordinate_system = colmap::PosePrior::CoordinateSystem::CARTESIAN;
      } else {
        // Fallback: just centre the coordinates without rotation.
        prior.position -= mean_prior_position;
      }

      // Update bounding box extents (E, N components)
      min_east = std::min(min_east, prior.position.x());
      max_east = std::max(max_east, prior.position.x());
      min_north = std::min(min_north, prior.position.y());
      max_north = std::max(max_north, prior.position.y());
      have_prior_area = true;

      // Subtract the mean prior position so that the positions are centred.
      // LOG(INFO) << "Prior position (ENU): " << prior.position.transpose();

      const colmap::Rigid3d world_from_cam_prior(Eigen::Quaterniond::Identity(),
                                                 prior.position);
      inserted_image.cam_from_world = Rigid3d(Inverse(world_from_cam_prior));
      inserted_image.pose_prior = prior;
    } else {
      inserted_image.cam_from_world = Rigid3d();
      inserted_image.pose_prior = std::nullopt;
    }
  }
  std::cout << std::endl;

  // Compute and report approximate site area (axis-aligned bounding box)
  if (have_prior_area) {
    const double width_east  = max_east  - min_east;
    const double height_north = max_north - min_north;
    const double site_area_sqm = width_east * height_north;
    LOG(INFO) << "Site bounding box (E,N) size: " << width_east << " m x " << height_north << " m";
    LOG(INFO) << "Approximate site area: " << site_area_sqm << " square meters.";
  }

  // Read keypoints
  for (auto& [image_id, image] : images) {
    const colmap::FeatureKeypoints keypoints = database.ReadKeypoints(image_id);
    const int num_keypoints = keypoints.size();
    image.features.resize(num_keypoints);
    for (int i = 0; i < num_keypoints; i++) {
      image.features[i] = Eigen::Vector2d(keypoints[i].x, keypoints[i].y);
    }
  }

  // Add the cameras
  std::vector<colmap::Camera> cameras_colmap = database.ReadAllCameras();
  for (auto& camera : cameras_colmap) {
    cameras[camera.camera_id] = camera;
  }

  // Add the matches
  std::vector<std::pair<colmap::image_pair_t, colmap::FeatureMatches>>
      all_matches = database.ReadAllMatches();

  // Go through all matches and store the matche with enough observations in the
  // view_graph
  size_t invalid_count = 0;
  std::unordered_map<image_pair_t, ImagePair>& image_pairs =
      view_graph.image_pairs;
  for (size_t match_idx = 0; match_idx < all_matches.size(); match_idx++) {
    if ((match_idx + 1) % 1000 == 0 || match_idx == all_matches.size() - 1)
      std::cout << "\r Loading Image Pair " << match_idx + 1 << " / "
                << all_matches.size() << std::flush;
    // Read the image pair from COLMAP database
    colmap::image_pair_t pair_id = all_matches[match_idx].first;
    std::pair<colmap::image_t, colmap::image_t> image_pair_colmap =
        database.PairIdToImagePair(pair_id);
    colmap::image_t image_id1 = image_pair_colmap.first;
    colmap::image_t image_id2 = image_pair_colmap.second;

    colmap::FeatureMatches& feature_matches = all_matches[match_idx].second;

    // Initialize the image pair
    auto ite = image_pairs.insert(
        std::make_pair(ImagePair::ImagePairToPairId(image_id1, image_id2),
                       ImagePair(image_id1, image_id2)));
    ImagePair& image_pair = ite.first->second;

    colmap::TwoViewGeometry two_view =
        database.ReadTwoViewGeometry(image_id1, image_id2);

    // If the image is marked as invalid or watermark, then skip
    if (two_view.config == colmap::TwoViewGeometry::UNDEFINED ||
        two_view.config == colmap::TwoViewGeometry::DEGENERATE ||
        two_view.config == colmap::TwoViewGeometry::WATERMARK ||
        two_view.config == colmap::TwoViewGeometry::MULTIPLE) {
      image_pair.is_valid = false;
      invalid_count++;
      continue;
    }

    // Collect the fundemental matrices
    if (two_view.config == colmap::TwoViewGeometry::UNCALIBRATED) {
      image_pair.F = two_view.F;
    } else if (two_view.config == colmap::TwoViewGeometry::CALIBRATED) {
      FundamentalFromMotionAndCameras(
          cameras.at(images.at(image_pair.image_id1).camera_id),
          cameras.at(images.at(image_pair.image_id2).camera_id),
          two_view.cam2_from_cam1,
          &image_pair.F);
    } else if (two_view.config == colmap::TwoViewGeometry::PLANAR ||
               two_view.config == colmap::TwoViewGeometry::PANORAMIC ||
               two_view.config ==
                   colmap::TwoViewGeometry::PLANAR_OR_PANORAMIC) {
      image_pair.H = two_view.H;
      image_pair.F = two_view.F;
    }
    image_pair.config = two_view.config;

    // Collect the matches
    image_pair.matches = Eigen::MatrixXi(feature_matches.size(), 2);

    std::vector<Eigen::Vector2d>& keypoints1 =
        images[image_pair.image_id1].features;
    std::vector<Eigen::Vector2d>& keypoints2 =
        images[image_pair.image_id2].features;

    feature_t count = 0;
    for (int i = 0; i < feature_matches.size(); i++) {
      colmap::point2D_t point2D_idx1 = feature_matches[i].point2D_idx1;
      colmap::point2D_t point2D_idx2 = feature_matches[i].point2D_idx2;
      if (point2D_idx1 != colmap::kInvalidPoint2DIdx &&
          point2D_idx2 != colmap::kInvalidPoint2DIdx) {
        if (keypoints1.size() <= point2D_idx1 ||
            keypoints2.size() <= point2D_idx2)
          continue;
        image_pair.matches.row(count) << point2D_idx1, point2D_idx2;
        count++;
      }
    }
    image_pair.matches.conservativeResize(count, 2);
  }
  std::cout << std::endl;

  LOG(INFO) << "Pairs read done. " << invalid_count << " / "
            << view_graph.image_pairs.size() << " are invalid";
}

}  // namespace glomap
