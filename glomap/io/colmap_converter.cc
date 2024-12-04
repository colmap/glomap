#include "glomap/io/colmap_converter.h"

#include "glomap/math/two_view_geometry.h"

namespace glomap {

    void ConvertGlomapToColmapImage(const migration::Image& image,
                                    Image& image_colmap,
                                    bool keep_points) {
        image_colmap.SetImageId(image.image_id);
        image_colmap.SetCameraId(image.camera_id);
        image_colmap.SetName(image.file_name);
        if (image.is_registered)
        {
            image_colmap.SetCamFromWorld(image.cam_from_world);
        }

        if (keep_points)
        {
            image_colmap.SetPoints2D(image.features);
        }
    }

    void ConvertGlomapToColmap(const std::unordered_map<camera_t, Camera>& cameras,
                               const std::unordered_map<image_t, migration::Image>& images,
                               const std::unordered_map<track_t, migration::Track>& tracks,
                               Reconstruction& reconstruction,
                               int cluster_id,
                               bool include_image_points) {
        // Clear the colmap reconstruction
        reconstruction = Reconstruction();

        // Add cameras
        for (const auto& [camera_id, camera] : cameras)
        {
            reconstruction.AddCamera(camera);
        }

        // Prepare the 2d-3d correspondences
        std::unordered_map<image_t, std::vector<track_t>> image_to_point3D;
        if (tracks.size() > 0 || include_image_points)
        {
            // Initialize every point to corresponds to invalid point
            for (auto& [image_id, image] : images)
            {
                if (!image.is_registered ||
                    (cluster_id != -1 && image.cluster_id != cluster_id))
                    continue;
                image_to_point3D[image_id] =
                    std::vector<track_t>(image.features.size(), -1);
            }

            if (tracks.size() > 0)
            {
                for (auto& [track_id, track] : tracks)
                {
                    if (track.observations.size() < 3)
                    {
                        continue;
                    }
                    for (auto& observation : track.observations)
                    {
                        if (image_to_point3D.find(observation.first) !=
                            image_to_point3D.end())
                        {
                            image_to_point3D[observation.first][observation.second] = track_id;
                        }
                    }
                }
            }
        }

        // Add points
        for (const auto& [track_id, track] : tracks)
        {
            Point3D colmap_point;
            colmap_point.xyz = track.xyz;
            colmap_point.color = track.color;
            colmap_point.error = 0;

            // Add track element
            for (auto& observation : track.observations)
            {
                const migration::Image& image = images.at(observation.first);
                if (!image.is_registered ||
                    (cluster_id != -1 && image.cluster_id != cluster_id))
                    continue;
                TrackElement colmap_track_el;
                colmap_track_el.image_id = observation.first;
                colmap_track_el.point2D_idx = observation.second;

                colmap_point.track.AddElement(colmap_track_el);
            }

            if (colmap_point.track.Length() < 2)
                continue;

            colmap_point.track.Compress();
            reconstruction.AddPoint3D(track_id, std::move(colmap_point));
        }

        // Add images
        for (const auto& [image_id, image] : images)
        {
            if (!image.is_registered ||
                (cluster_id != -1 && image.cluster_id != cluster_id))
                continue;

            Image image_colmap;
            bool keep_points =
                image_to_point3D.find(image_id) != image_to_point3D.end();
            ConvertGlomapToColmapImage(image, image_colmap, keep_points);
            if (keep_points)
            {
                std::vector<track_t>& track_ids = image_to_point3D[image_id];
                for (size_t i = 0; i < image.features.size(); i++)
                {
                    if (track_ids[i] != -1)
                    {
                        image_colmap.SetPoint3DForPoint2D(i, track_ids[i]);
                    }
                }
            }

            reconstruction.AddImage(std::move(image_colmap));
        }

        reconstruction.UpdatePoint3DErrors();
    }

    void ConvertColmapToGlomap(const Reconstruction& reconstruction,
                               std::unordered_map<camera_t, Camera>& cameras,
                               std::unordered_map<image_t, migration::Image>& images,
                               std::unordered_map<track_t, migration::Track>& tracks) {
        // Clear the glomap reconstruction
        cameras.clear();
        images.clear();

        // Add cameras
        for (const auto& [camera_id, camera] : reconstruction.Cameras())
        {
            cameras[camera_id] = camera;
        }

        for (auto& [image_id, image_colmap] : reconstruction.Images())
        {
            auto ite = images.insert(std::make_pair(image_colmap.ImageId(),
                                                    migration::Image(image_colmap.ImageId(),
                                                          image_colmap.CameraId(),
                                                          image_colmap.Name())));

            migration::Image& image = ite.first->second;
            image.is_registered = image_colmap.HasPose();
            if (image_colmap.HasPose())
            {
                image.cam_from_world = static_cast<Rigid3d>(image_colmap.CamFromWorld());
            }
            image.features.clear();
            image.features.reserve(image_colmap.NumPoints2D());

            for (auto& point2D : image_colmap.Points2D())
            {
                image.features.push_back(point2D.xy);
            }
        }

        ConvertColmapPoints3DToGlomapTracks(reconstruction, tracks);
    }

    void ConvertColmapPoints3DToGlomapTracks(
        const Reconstruction& reconstruction,
        std::unordered_map<track_t, migration::Track>& tracks) {
        // Read tracks
        tracks.clear();
        tracks.reserve(reconstruction.NumPoints3D());

        auto& points3D = reconstruction.Points3D();
        for (auto& [point3d_id, point3D] : points3D)
        {
            migration::Track track;
            const Track& track_colmap = point3D.track;
            track.xyz = point3D.xyz;
            track.color = point3D.color;
            track.track_id = point3d_id;
            track.is_initialized = true;

            const std::vector<TrackElement>& elements = track_colmap.Elements();
            track.observations.reserve(track_colmap.Length());
            for (auto& element : elements)
            {
                track.observations.push_back(
                    migration::Observation(element.image_id, element.point2D_idx));
            }

            tracks.insert(std::make_pair(point3d_id, track));
        }
    }

    // For ease of debug, go through the database twice: first extract the available
    // pairs, then read matches from pairs.
    void ConvertDatabaseToGlomap(const Database& database,
                                 ViewGraph& view_graph,
                                 std::unordered_map<camera_t, Camera>& cameras,
                                 std::unordered_map<image_t, migration::Image>& images) {
        // Add the images
        std::vector<Image> images_colmap = database.ReadAllImages();
        image_t counter = 0;
        for (auto& image : images_colmap)
        {
            std::cout << "\r Loading Images " << counter + 1 << " / "
                      << images_colmap.size() << std::flush;
            counter++;

            image_t image_id = image.ImageId();
            if (image_id == kInvalidImageId)
                continue;
            auto ite = images.insert(std::make_pair(
                image_id,  migration::Image(image_id, image.CameraId(), image.Name())));
            const PosePrior prior = database.ReadPosePrior(image_id);
            if (prior.IsValid())
            {
                ite.first->second.cam_from_world = Rigid3d(
                    Rigid3d(Eigen::Quaterniond::Identity(), prior.position));
            } else
            {
                ite.first->second.cam_from_world = Rigid3d();
            }
        }
        std::cout << std::endl;

        // Read keypoints
        for (auto& [image_id, image] : images)
        {
            FeatureKeypoints keypoints = database.ReadKeypoints(image_id);

            image.features.reserve(keypoints.size());
            for (int i = 0; i < keypoints.size(); i++)
            {
                image.features.emplace_back(
                    Eigen::Vector2d(keypoints[i].x, keypoints[i].y));
            }
        }

        // Add the cameras
        std::vector<Camera> cameras_colmap = database.ReadAllCameras();
        for (auto& camera : cameras_colmap)
        {
            camera_t camera_id = camera.camera_id;
            cameras[camera_id] = camera;
        }

        // Add the matches
        std::vector<std::pair<image_pair_t, FeatureMatches>>
            all_matches = database.ReadAllMatches();

        // Go through all matches and store the matche with enough observations in the
        // view_graph
        size_t invalid_count = 0;
        std::unordered_map<image_pair_t, ImagePair>& image_pairs =
            view_graph.image_pairs;
        for (size_t match_idx = 0; match_idx < all_matches.size(); match_idx++)
        {
            if ((match_idx + 1) % 1000 == 0 || match_idx == all_matches.size() - 1)
                std::cout << "\r Loading Image Pair " << match_idx + 1 << " / "
                          << all_matches.size() << std::flush;
            // Read the image pair from COLMAP database
            image_pair_t pair_id = all_matches[match_idx].first;
            std::pair<image_t, image_t> image_pair_colmap =
                database.PairIdToImagePair(pair_id);
            image_t image_id1 = image_pair_colmap.first;
            image_t image_id2 = image_pair_colmap.second;

            FeatureMatches& feature_matches = all_matches[match_idx].second;

            // Initialize the image pair
            auto ite = image_pairs.insert(
                std::make_pair(ImagePair::ImagePairToPairId(image_id1, image_id2),
                               ImagePair(image_id1, image_id2)));
            ImagePair& image_pair = ite.first->second;

            TwoViewGeometry two_view =
                database.ReadTwoViewGeometry(image_id1, image_id2);

            // If the image is marked as invalid or watermark, then skip
            if (two_view.config == TwoViewGeometry::UNDEFINED ||
                two_view.config == TwoViewGeometry::DEGENERATE ||
                two_view.config == TwoViewGeometry::WATERMARK ||
                two_view.config == TwoViewGeometry::MULTIPLE)
            {
                image_pair.is_valid = false;
                invalid_count++;
                continue;
            }

            // Collect the fundemental matrices
            if (two_view.config == TwoViewGeometry::UNCALIBRATED)
            {
                image_pair.F = two_view.F;
            } else if (two_view.config == TwoViewGeometry::CALIBRATED)
            {
                FundamentalFromMotionAndCameras(
                    cameras.at(images.at(image_pair.image_id1).camera_id),
                    cameras.at(images.at(image_pair.image_id2).camera_id),
                    two_view.cam2_from_cam1,
                    &image_pair.F);
            } else if (two_view.config == TwoViewGeometry::PLANAR ||
                       two_view.config == TwoViewGeometry::PANORAMIC ||
                       two_view.config ==
                           TwoViewGeometry::PLANAR_OR_PANORAMIC)
            {
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
            for (int i = 0; i < feature_matches.size(); i++)
            {
                point2D_t point2D_idx1 = feature_matches[i].point2D_idx1;
                point2D_t point2D_idx2 = feature_matches[i].point2D_idx2;
                if (point2D_idx1 != kInvalidPoint2DIdx &&
                    point2D_idx2 != kInvalidPoint2DIdx)
                {
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

} // namespace glomap
