#include "global_mapper.h"

#include "glomap/io/colmap_converter.h"
#include "glomap/processors/image_pair_inliers.h"
#include "glomap/processors/image_undistorter.h"
#include "glomap/processors/reconstruction_normalizer.h"
#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/processors/relpose_filter.h"
#include "glomap/processors/track_filter.h"
#include "glomap/processors/view_graph_manipulation.h"

#include <colmap/util/file.h>
#include <colmap/util/timer.h>

namespace glomap {

bool GlobalMapper::Solve(const colmap::Database& database,
                         ViewGraph& view_graph,
                         std::unordered_map<camera_t, Camera>& cameras,
                         std::unordered_map<image_t, Image>& images,
                         std::unordered_map<track_t, Track>& tracks) {
  // 0. Preprocessing
  if (!options_.skip_preprocessing) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running preprocessing ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();
    // If camera intrinsics seem to be good, force the pair to use essential
    // matrix
    ViewGraphManipulater::UpdateImagePairsConfig(view_graph, cameras, images);
    ViewGraphManipulater::DecomposeRelPose(view_graph, cameras, images);
    run_timer.PrintSeconds();
  }

  // 1. Run view graph calibration
  if (!options_.skip_view_graph_calibration) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running view graph calibration ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    ViewGraphCalibrator vgcalib_engine(options_.opt_vgcalib);
    if (!vgcalib_engine.Solve(view_graph, cameras, images)) {
      return false;
    }
  }

  // 2. Run relative pose estimation
  if (!options_.skip_relative_pose_estimation) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running relative pose estimation ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();
    // Relative pose relies on the undistorted images
    UndistortImages(cameras, images, true);
    EstimateRelativePoses(view_graph, cameras, images, options_.opt_relpose);

    InlierThresholdOptions inlier_thresholds = options_.inlier_thresholds;
    // Undistort the images and filter edges by inlier number
    ImagePairsInlierCount(view_graph, cameras, images, inlier_thresholds, true);

    RelPoseFilter::FilterInlierNum(view_graph,
                                   options_.inlier_thresholds.min_inlier_num);
    RelPoseFilter::FilterInlierRatio(
        view_graph, options_.inlier_thresholds.min_inlier_ratio);

    if (view_graph.KeepLargestConnectedComponents(images) == 0) {
      LOG(ERROR) << "no connected components are found";
      return false;
    }

    run_timer.PrintSeconds();
  }

  // 3. Run rotation averaging for three times
  if (!options_.skip_rotation_averaging) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running rotation averaging ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();

    RotationEstimator ra_engine(options_.opt_ra);
    // The first run is for filtering
    ra_engine.EstimateRotations(view_graph, images);

    RelPoseFilter::FilterRotations(
        view_graph, images, options_.inlier_thresholds.max_rotation_error);
    if (view_graph.KeepLargestConnectedComponents(images) == 0) {
      LOG(ERROR) << "no connected components are found";
      return false;
    }

    // The second run is for final estimation
    if (!ra_engine.EstimateRotations(view_graph, images)) {
      return false;
    }
    RelPoseFilter::FilterRotations(
        view_graph, images, options_.inlier_thresholds.max_rotation_error);
    image_t num_img = view_graph.KeepLargestConnectedComponents(images);
    if (num_img == 0) {
      LOG(ERROR) << "no connected components are found";
      return false;
    }
    LOG(INFO) << num_img << " / " << images.size()
              << " images are within the connected component." << std::endl;

    run_timer.PrintSeconds();
  }

  // 4. Track establishment and selection
  if (!options_.skip_track_establishment) {
    colmap::Timer run_timer;
    run_timer.Start();

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running track establishment ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    TrackEngine track_engine(view_graph, images, options_.opt_track);
    std::unordered_map<track_t, Track> tracks_full;
    track_engine.EstablishFullTracks(tracks_full);

    // Filter the tracks
    track_t num_tracks = track_engine.FindTracksForProblem(tracks_full, tracks);
    LOG(INFO) << "Before filtering: " << tracks_full.size()
              << ", after filtering: " << num_tracks << std::endl;

    run_timer.PrintSeconds();
  }

  // 5. Global positioning
  if (!options_.skip_global_positioning) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running global positioning ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();
    // Undistort images in case all previous steps are skipped
    // Skip images where an undistortion already been done
    UndistortImages(cameras, images, false);

    GlobalPositioner gp_engine(options_.opt_gp);
    if (!gp_engine.Solve(view_graph, cameras, images, tracks)) {
      return false;
    }

    // If only camera-to-camera constraints are used for solving camera
    // positions, then points needs to be estimated separately
    if (options_.opt_gp.constraint_type ==
        GlobalPositionerOptions::ConstraintType::ONLY_CAMERAS) {
      GlobalPositionerOptions opt_gp_pt = options_.opt_gp;
      opt_gp_pt.constraint_type =
          GlobalPositionerOptions::ConstraintType::ONLY_POINTS;
      opt_gp_pt.optimize_positions = false;
      GlobalPositioner gp_engine_pt(opt_gp_pt);
      if (!gp_engine_pt.Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
    }

    // Filter tracks based on the estimation
    TrackFilter::FilterTracksByAngle(
        view_graph,
        cameras,
        images,
        tracks,
        options_.inlier_thresholds.max_angle_error);

    // Normalize the structure
    NormalizeReconstruction(cameras, images, tracks);

    run_timer.PrintSeconds();
  }

  // 6. Bundle adjustment
  if (!options_.skip_bundle_adjustment) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running bundle adjustment ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    LOG(INFO) << "Bundle adjustment start" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();

    for (int ite = 0; ite < options_.num_iteration_bundle_adjustment; ite++) {
      BundleAdjuster ba_engine(options_.opt_ba);

      BundleAdjusterOptions& ba_engine_options_inner = ba_engine.GetOptions();

      // Staged bundle adjustment
      // 6.1. First stage: optimize positions only
      ba_engine_options_inner.optimize_rotations = false;
      if (!ba_engine.Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
      LOG(INFO) << "Global bundle adjustment iteration " << ite + 1 << " / "
                << options_.num_iteration_bundle_adjustment
                << ", stage 1 finished (position only)";
      run_timer.PrintSeconds();

      // 6.2. Second stage: optimize rotations if desired
      ba_engine_options_inner.optimize_rotations =
          options_.opt_ba.optimize_rotations;
      if (ba_engine_options_inner.optimize_rotations &&
          !ba_engine.Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
      LOG(INFO) << "Global bundle adjustment iteration " << ite + 1 << " / "
                << options_.num_iteration_bundle_adjustment
                << ", stage 2 finished";
      if (ite != options_.num_iteration_bundle_adjustment - 1)
        run_timer.PrintSeconds();

      // Normalize the structure
      NormalizeReconstruction(cameras, images, tracks);

      // 6.3. Filter tracks based on the estimation
      // For the filtering, in each round, the criteria for outlier is
      // tightened. If only few tracks are changed, no need to start bundle
      // adjustment right away. Instead, use a more strict criteria to filter
      UndistortImages(cameras, images, true);
      LOG(INFO) << "Filtering tracks by reprojection ...";

      bool status = true;
      size_t filtered_num = 0;
      while (status && ite < options_.num_iteration_bundle_adjustment) {
        double scaling = std::max(3 - ite, 1);
        filtered_num += TrackFilter::FilterTracksByReprojection(
            view_graph,
            cameras,
            images,
            tracks,
            scaling * options_.inlier_thresholds.max_reprojection_error);

        if (filtered_num > 1e-3 * tracks.size()) {
          status = false;
        } else
          ite++;
      }
      if (status) {
        LOG(INFO) << "fewer than 0.1% tracks are filtered, stop the iteration.";
        break;
      }
    }

    // Filter tracks based on the estimation
    UndistortImages(cameras, images, true);
    LOG(INFO) << "Filtering tracks by reprojection ...";
    TrackFilter::FilterTracksByReprojection(
        view_graph,
        cameras,
        images,
        tracks,
        options_.inlier_thresholds.max_reprojection_error);
    TrackFilter::FilterTrackTriangulationAngle(
        view_graph,
        images,
        tracks,
        options_.inlier_thresholds.min_triangulation_angle);

    run_timer.PrintSeconds();
  }

  // 7. Retriangulation
  if (!options_.skip_retriangulation) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running retriangulation ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    for (int ite = 0; ite < options_.num_iteration_retriangulation; ite++) {
      colmap::Timer run_timer;
      run_timer.Start();
      RetriangulateTracks(
          options_.opt_triangulator, database, cameras, images, tracks);
      run_timer.PrintSeconds();

      std::cout << "-------------------------------------" << std::endl;
      std::cout << "Running bundle adjustment ..." << std::endl;
      std::cout << "-------------------------------------" << std::endl;
      LOG(INFO) << "Bundle adjustment start" << std::endl;
      BundleAdjuster ba_engine(options_.opt_ba);
      if (!ba_engine.Solve(view_graph, cameras, images, tracks)) {
        return false;
      }

      // Filter tracks based on the estimation
      UndistortImages(cameras, images, true);
      LOG(INFO) << "Filtering tracks by reprojection ...";
      TrackFilter::FilterTracksByReprojection(
          view_graph,
          cameras,
          images,
          tracks,
          options_.inlier_thresholds.max_reprojection_error);
      if (!ba_engine.Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
      run_timer.PrintSeconds();
    }

    // Normalize the structure
    NormalizeReconstruction(cameras, images, tracks);

    // Filter tracks based on the estimation
    UndistortImages(cameras, images, true);
    LOG(INFO) << "Filtering tracks by reprojection ...";
    TrackFilter::FilterTracksByReprojection(
        view_graph,
        cameras,
        images,
        tracks,
        options_.inlier_thresholds.max_reprojection_error);
    TrackFilter::FilterTrackTriangulationAngle(
        view_graph,
        images,
        tracks,
        options_.inlier_thresholds.min_triangulation_angle);
  }

  // 8. Reconstruction pruning
  if (!options_.skip_pruning) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Running postprocessing ..." << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    colmap::Timer run_timer;
    run_timer.Start();

    // Prune weakly connected images
    PruneWeaklyConnectedImages(images, tracks);

    run_timer.PrintSeconds();
  }

  return true;
}

}  // namespace glomap
