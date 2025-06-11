#include "global_mapper.h"

#include "glomap/estimators/bundle_adjustment.h"
#include "glomap/io/colmap_converter.h"
#include "glomap/processors/image_pair_inliers.h"
#include "glomap/processors/image_undistorter.h"
#include "glomap/processors/reconstruction_normalizer.h"
#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/processors/reconstruction_aligner.h"
#include "glomap/processors/relpose_filter.h"
#include "glomap/processors/track_filter.h"
#include "glomap/processors/view_graph_manipulation.h"

#include <colmap/util/file.h>
#include <colmap/util/timer.h>
#include <colmap/estimators/similarity_transform.h>
#include <colmap/geometry/sim3.h>

#include <memory>

namespace glomap {
namespace {
void RestoreTranslationToPriorPosition(
    std::unordered_map<image_t, Image>& images) {
  for (auto& [_, image] : images) {
    if (!image.pose_prior) {
      continue;
    }
    // t=-Rc
    image.cam_from_world.translation =
        -(image.cam_from_world.rotation * image.pose_prior->position);
  }
}

PosePriorBundleAdjusterOptions ExtractPosePriorBAOptions(
    const GlobalMapperOptions& options) {
  PosePriorBundleAdjusterOptions pose_prior_options(
      options.opt_pose_prior.use_robust_loss_on_prior_position,
      options.opt_pose_prior.prior_position_loss_threshold,
      options.opt_pose_prior.prior_position_scaled_loss_factor,
      options.opt_pose_prior.alignment_ransac_max_error);
  return pose_prior_options;
}
}  // namespace

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

    RestoreTranslationToPriorPosition(images);

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

    // Do not gernerate random image positions if use prior position.
    gp_engine.GetOptions().generate_random_positions =
        !options_.opt_pose_prior.use_pose_position_prior;

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

    // Normalize the structure if do not use prior position.
    if (!options_.opt_pose_prior.use_pose_position_prior) {
      NormalizeReconstruction(cameras, images, tracks);
    }

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

    // -------------------------------------------------------------------
    // Record the initial Z value (camera center altitude) of every image
    // before any bundle-adjustment takes place. We use the image centre in
    // world coordinates, i.e. `Image::Center()`, instead of the raw
    // translation element, because the centre is invariant to the
    // parameterisation of the pose (t depends on R).
    // -------------------------------------------------------------------
    std::unordered_map<image_t, double> initial_center_zs;
    initial_center_zs.reserve(images.size());
    for (const auto& [img_id, img] : images) {
      initial_center_zs[img_id] = img.Center()(2);
    }

    // -------------------------------------------------------------------
    // Compute mean absolute Z error to pose priors BEFORE bundle adjustment
    // This shows how far camera heights deviate from their priors initially.
    // -------------------------------------------------------------------
    double sum_abs_z_err_prior_before = 0.0;
    size_t num_prior_z = 0;
    for (const auto& [img_id, img] : images) {
      if (img.pose_prior.has_value() &&
          img.pose_prior->coordinate_system ==
              colmap::PosePrior::CoordinateSystem::CARTESIAN) {
        sum_abs_z_err_prior_before += std::abs(img.Center()(2) -
                                              img.pose_prior->position(2));
        ++num_prior_z;
      }
    }
    const double mean_abs_z_err_prior_before = (num_prior_z == 0)
                                                   ? 0.0
                                                   : sum_abs_z_err_prior_before /
                                                         static_cast<double>(num_prior_z);

    std::cout << "Mean absolute Z error to priors BEFORE bundle adjustment: "
              << mean_abs_z_err_prior_before << std::endl;
    LOG(INFO) << "Mean absolute Z error to priors BEFORE bundle adjustment: "
              << mean_abs_z_err_prior_before;

    for (int ite = 0; ite < options_.num_iteration_bundle_adjustment; ite++) {
      std::unique_ptr<BundleAdjuster> ba_engine;

      if (options_.opt_pose_prior.use_pose_position_prior) {
        PosePriorBundleAdjusterOptions opt_prior_ba =
            ExtractPosePriorBAOptions(options_);
        ba_engine = std::make_unique<PosePriorBundleAdjuster>(options_.opt_ba,
                                                              opt_prior_ba);
      } else {
        ba_engine = std::make_unique<BundleAdjuster>(options_.opt_ba);
      }

      BundleAdjusterOptions& ba_engine_options_inner = ba_engine->GetOptions();

      // Staged bundle adjustment
      // 6.1. First stage: optimize positions only
      ba_engine_options_inner.optimize_rotations = false;
      if (!ba_engine->Solve(view_graph, cameras, images, tracks)) {
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
          !ba_engine->Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
      LOG(INFO) << "Global bundle adjustment iteration " << ite + 1 << " / "
                << options_.num_iteration_bundle_adjustment
                << ", stage 2 finished";
      if (ite != options_.num_iteration_bundle_adjustment - 1)
        run_timer.PrintSeconds();

      // Normalize the structure if do not use prior position.
      if (!options_.opt_pose_prior.use_pose_position_prior) {
        NormalizeReconstruction(cameras, images, tracks);
      }

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

    // -------------------------------------------------------------------
    // Compute and report the mean absolute change in Z after bundle-adjustment.
    // -------------------------------------------------------------------
    // 1. Change relative to initial reconstruction
    double sum_abs_z_error = 0.0;
    for (const auto& [img_id, img] : images) {
      const auto it = initial_center_zs.find(img_id);
      if (it != initial_center_zs.end()) {
        sum_abs_z_error += std::abs(img.Center()(2) - it->second);
      }
    }
    const double mean_abs_z_error = images.empty()
                                     ? 0.0
                                     : sum_abs_z_error / static_cast<double>(images.size());

    std::cout << "Mean absolute Z change after bundle adjustment: "
              << mean_abs_z_error << std::endl;
    LOG(INFO) << "Mean absolute Z change after bundle adjustment: "
              << mean_abs_z_error;

    // 2. Error to priors AFTER bundle adjustment
    double sum_abs_z_err_prior_after = 0.0;
    for (const auto& [img_id, img] : images) {
      if (img.pose_prior.has_value() &&
          img.pose_prior->coordinate_system ==
              colmap::PosePrior::CoordinateSystem::CARTESIAN) {
        sum_abs_z_err_prior_after += std::abs(img.Center()(2) -
                                             img.pose_prior->position(2));
      }
    }
    const double mean_abs_z_err_prior_after = (num_prior_z == 0)
                                                   ? 0.0
                                                   : sum_abs_z_err_prior_after /
                                                         static_cast<double>(num_prior_z);

    std::cout << "Mean absolute Z error to priors AFTER bundle adjustment: "
              << mean_abs_z_err_prior_after << std::endl;
    LOG(INFO) << "Mean absolute Z error to priors AFTER bundle adjustment: "
              << mean_abs_z_err_prior_after;

    // -------------------------------------------------------------------
    // (Optional) Align reconstruction to pose priors for evaluation only
    // if pose priors were NOT used during optimisation. This provides an
    // apples-to-apples Z-error measure even when the reconstruction coordinate
    // frame differs from the prior frame.
    // -------------------------------------------------------------------
    if (!options_.opt_pose_prior.use_pose_position_prior && num_prior_z > 0) {
      // Build point correspondences: reconstruction centres -> prior positions
      std::vector<Eigen::Vector3d> src_locations;
      std::vector<Eigen::Vector3d> tgt_locations;
      src_locations.reserve(num_prior_z);
      tgt_locations.reserve(num_prior_z);
      for (const auto& [img_id, img] : images) {
        if (img.pose_prior.has_value() &&
            img.pose_prior->coordinate_system ==
                colmap::PosePrior::CoordinateSystem::CARTESIAN) {
          src_locations.emplace_back(img.Center());
          tgt_locations.emplace_back(img.pose_prior->position);
        }
      }

      if (!src_locations.empty()) {
        colmap::Sim3d tform;
        bool align_success = false;
        const double max_error = (options_.opt_pose_prior.alignment_ransac_max_error > 0)
                                     ? options_.opt_pose_prior.alignment_ransac_max_error
                                     : -1.0;
        if (max_error > 0) {
          colmap::RANSACOptions ransac_opt;
          ransac_opt.max_error = max_error;
          auto report = colmap::EstimateSim3dRobust(
              src_locations, tgt_locations, ransac_opt, tform);
          align_success = report.success;
        } else {
          align_success = colmap::EstimateSim3d(src_locations, tgt_locations, tform);
        }

        if (align_success) {
          double sum_abs_z_err_aligned = 0.0;
          for (size_t i = 0; i < src_locations.size(); ++i) {
            const double z_est = (tform * src_locations[i])(2);
            const double z_true = tgt_locations[i](2);
            sum_abs_z_err_aligned += std::abs(z_est - z_true);
          }
          const double mean_abs_z_err_aligned =
              sum_abs_z_err_aligned / static_cast<double>(src_locations.size());

          std::cout << "Mean absolute Z error to priors AFTER alignment (evaluation only): "
                    << mean_abs_z_err_aligned << std::endl;
          LOG(INFO) << "Mean absolute Z error to priors AFTER alignment (evaluation only): "
                    << mean_abs_z_err_aligned;
        } else {
          std::cout << "Could not align reconstruction to pose priors for evaluation." << std::endl;
          LOG(WARNING) << "Could not align reconstruction to pose priors for evaluation.";
        }
      }
    }

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

      std::unique_ptr<BundleAdjuster> ba_engine;

      if (options_.opt_pose_prior.use_pose_position_prior) {
        PosePriorBundleAdjusterOptions pose_prior_ba_options =
            ExtractPosePriorBAOptions(options_);
        ba_engine = std::make_unique<PosePriorBundleAdjuster>(
            options_.opt_ba, pose_prior_ba_options);
      } else {
        ba_engine = std::make_unique<BundleAdjuster>(options_.opt_ba);
      }

      if (!ba_engine->Solve(view_graph, cameras, images, tracks)) {
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
      if (!ba_engine->Solve(view_graph, cameras, images, tracks)) {
        return false;
      }
      run_timer.PrintSeconds();
    }

    // Normalize the structure if do not use prior position.
    if (!options_.opt_pose_prior.use_pose_position_prior) {
      NormalizeReconstruction(cameras, images, tracks);
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

    // -------------------------------------------------------------------
    // Optionally align the reconstruction into metric scale (meters) using
    // pose position priors, but only for evaluation / export. This is
    // controlled by `transform_to_meter`. It is skipped if priors are not
    // loaded or if pose priors were already used during optimisation (since
    // the reconstruction should already be in metric scale in that case).
    // -------------------------------------------------------------------
    if (options_.transform_to_meter &&
        !options_.opt_pose_prior.use_pose_position_prior) {
      std::cout << "-------------------------------------" << std::endl;
      std::cout << "Aligning reconstruction to meter-scale ..." << std::endl;
      std::cout << "-------------------------------------" << std::endl;
      std::unordered_map<image_t, colmap::PosePrior> priors;
      for (const auto& [img_id, img] : images) {
        if (img.pose_prior.has_value()) {
          priors[img_id] = img.pose_prior.value();
        }
      }

      if (!priors.empty()) {
        LOG(INFO) << "Aligning reconstruction to pose priors for meter-scale export.";

        if (AlignReconstructionToPosePositionPriors(priors, images, tracks)) {
          LOG(INFO) << "Alignment to priors successful. Reconstruction now in meter scale.";
        } else {
          LOG(WARNING) << "Alignment to priors failed. Keeping original scale.";
        }
      } else {
        LOG(INFO) << "No pose priors available; metric transform skipped.";
      }
    }
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
