#include "option_manager.h"

#include "glomap/controllers/global_mapper.h"

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace config = boost::program_options;

namespace glomap {

OptionManager::OptionManager(bool add_project_options) {
  database_path = std::make_shared<std::string>();
  image_path = std::make_shared<std::string>();

  mapper = std::make_shared<GlobalMapperOptions>();
  Reset();

  desc_->add_options()("help,h", "");
}

void OptionManager::AddAllOptions() {
  AddDatabaseOptions();
  AddImageOptions();
  AddGlobalMapperOptions();
  AddViewGraphCalibrationOptions();
  AddRelativePoseEstimationOptions();
  AddRotationEstimatorOptions();
  AddTrackEstablishmentOptions();
  AddGlobalPositionerOptions();
  AddBundleAdjusterOptions();
  AddTriangulatorOptions();
  AddInlierThresholdOptions();
}

void OptionManager::AddDatabaseOptions() {
  if (added_database_options_) {
    return;
  }
  added_database_options_ = true;

  AddAndRegisterRequiredOption("database_path", database_path.get());
}

void OptionManager::AddImageOptions() {
  if (added_image_options_) {
    return;
  }
  added_image_options_ = true;

  AddAndRegisterRequiredOption("image_path", image_path.get());
}

void OptionManager::AddGlobalMapperOptions() {
  if (added_mapper_options_) {
    return;
  }
  added_mapper_options_ = true;

  AddAndRegisterDefaultOption("ba_iteration_num",
                              &mapper->num_iteration_bundle_adjustment);
  AddAndRegisterDefaultOption("retriangulation_iteration_num",
                              &mapper->num_iteration_retriangulation);
  AddAndRegisterDefaultOption("skip_preprocessing",
                              &mapper->skip_preprocessing);
  AddAndRegisterDefaultOption("skip_view_graph_calibration",
                              &mapper->skip_view_graph_calibration);
  AddAndRegisterDefaultOption("skip_relative_pose_estimation",
                              &mapper->skip_relative_pose_estimation);
  AddAndRegisterDefaultOption("skip_rotation_averaging",
                              &mapper->skip_rotation_averaging);
  AddAndRegisterDefaultOption("skip_global_positioning",
                              &mapper->skip_global_positioning);
  AddAndRegisterDefaultOption("skip_bundle_adjustment",
                              &mapper->skip_bundle_adjustment);
  AddAndRegisterDefaultOption("skip_retriangulation",
                              &mapper->skip_retriangulation);
  AddAndRegisterDefaultOption("skip_postprocessing",
                              &mapper->skip_postprocessing);
}

void OptionManager::AddGlobalMapperFullOptions() {
  AddGlobalMapperOptions();

  AddViewGraphCalibrationOptions();
  AddRelativePoseEstimationOptions();
  AddRotationEstimatorOptions();
  AddTrackEstablishmentOptions();
  AddGlobalPositionerOptions();
  AddBundleAdjusterOptions();
  AddTriangulatorOptions();
  AddInlierThresholdOptions();
}

void OptionManager::AddGlobalMapperResumeOptions() {
  if (added_mapper_options_) {
    return;
  }
  added_mapper_options_ = true;

  // These several steps cannot be used if the reconstruction is resumed from a
  // reconstruction
  mapper->skip_preprocessing = true;
  mapper->skip_view_graph_calibration = true;
  mapper->skip_relative_pose_estimation = true;
  mapper->skip_rotation_averaging = true;
  mapper->skip_track_establishment = true;
  mapper->skip_retriangulation = true;

  AddAndRegisterDefaultOption("ba_iteration_num",
                              &mapper->num_iteration_bundle_adjustment);
  AddAndRegisterDefaultOption("retriangulation_iteration_num",
                              &mapper->num_iteration_retriangulation);
  AddAndRegisterDefaultOption("skip_global_positioning",
                              &mapper->skip_global_positioning);
  AddAndRegisterDefaultOption("skip_bundle_adjustment",
                              &mapper->skip_bundle_adjustment);
  AddAndRegisterDefaultOption("skip_postprocessing",
                              &mapper->skip_postprocessing);
}

void OptionManager::AddGlobalMapperResumeFullOptions() {
  AddGlobalMapperResumeOptions();

  AddGlobalPositionerOptions();
  AddBundleAdjusterOptions();
  AddTriangulatorOptions();
  AddInlierThresholdOptions();
}

void OptionManager::AddViewGraphCalibrationOptions() {
  if (added_view_graph_calibration_options_) {
    return;
  }
  added_view_graph_calibration_options_ = true;
  AddAndRegisterDefaultOption("ViewGraphCalib.thres_lower_ratio",
                              &mapper->opt_vgcalib.thres_lower_ratio);
  AddAndRegisterDefaultOption("ViewGraphCalib.thres_higher_ratio",
                              &mapper->opt_vgcalib.thres_higher_ratio);
  AddAndRegisterDefaultOption("ViewGraphCalib.robust_loss_thres",
                              &mapper->opt_vgcalib.thres_two_view_error);
}

void OptionManager::AddRelativePoseEstimationOptions() {
  if (added_relative_pose_options_) {
    return;
  }
  added_relative_pose_options_ = true;
}
void OptionManager::AddRotationEstimatorOptions() {
  if (added_rotation_averaging_options_) {
    return;
  }
  added_rotation_averaging_options_ = true;
  // TODO: maybe add options for rotation averaging
}

void OptionManager::AddTrackEstablishmentOptions() {
  if (added_track_establishment_options_) {
    return;
  }
  // TODO: maybe add options for track establishment
}

void OptionManager::AddGlobalPositionerOptions() {
  if (added_global_positioning_options_) {
    return;
  }
  added_global_positioning_options_ = true;
  AddAndRegisterDefaultOption("GlobalPositioning.optimize_positions",
                              &mapper->opt_gp.optimize_positions);
  AddAndRegisterDefaultOption("GlobalPositioning.optimize_points",
                              &mapper->opt_gp.optimize_points);
  AddAndRegisterDefaultOption("GlobalPositioning.optimize_scales",
                              &mapper->opt_gp.optimize_scales);
  AddAndRegisterDefaultOption("GlobalPositioning.thres_loss_function",
                              &mapper->opt_gp.thres_loss_function);
  AddAndRegisterDefaultOption(
      "GlobalPositioning.max_num_iterations",
      &mapper->opt_gp.solver_options.max_num_iterations);

  // TODO: move the constrain type selection here
}
void OptionManager::AddBundleAdjusterOptions() {
  if (added_bundle_adjustment_options_) {
    return;
  }
  added_bundle_adjustment_options_ = true;
  AddAndRegisterDefaultOption("BundleAdjustment.optimize_rotations",
                              &mapper->opt_ba.optimize_rotations);
  AddAndRegisterDefaultOption("BundleAdjustment.optimize_translation",
                              &mapper->opt_ba.optimize_translation);
  AddAndRegisterDefaultOption("BundleAdjustment.optimize_intrinsics",
                              &mapper->opt_ba.optimize_intrinsics);
  AddAndRegisterDefaultOption("BundleAdjustment.optimize_points",
                              &mapper->opt_ba.optimize_points);
  AddAndRegisterDefaultOption("BundleAdjustment.thres_loss_function",
                              &mapper->opt_ba.thres_loss_function);
  AddAndRegisterDefaultOption(
      "BundleAdjustment.max_num_iterations",
      &mapper->opt_ba.solver_options.max_num_iterations);
}
void OptionManager::AddTriangulatorOptions() {
  if (added_triangulation_options_) {
    return;
  }
  added_triangulation_options_ = true;
  AddAndRegisterDefaultOption(
      "Triangulation.complete_max_reproj_error",
      &mapper->opt_triangulator.tri_complete_max_reproj_error);
  AddAndRegisterDefaultOption(
      "Triangulation.merge_max_reproj_error",
      &mapper->opt_triangulator.tri_merge_max_reproj_error);
  AddAndRegisterDefaultOption("Triangulation.min_angle",
                              &mapper->opt_triangulator.tri_min_angle);
  AddAndRegisterDefaultOption("Triangulation.min_num_matches",
                              &mapper->opt_triangulator.min_num_matches);
}
void OptionManager::AddInlierThresholdOptions() {
  if (added_inliers_options_) {
    return;
  }
  added_inliers_options_ = true;
  // TODO: maybe add options for inlier threshold
}

void OptionManager::Reset() {
  const bool kResetPaths = true;
  ResetOptions(kResetPaths);

  desc_ = std::make_shared<boost::program_options::options_description>();

  options_bool_.clear();
  options_int_.clear();
  options_double_.clear();
  options_string_.clear();

  added_mapper_options_ = false;
  added_view_graph_calibration_options_ = false;
  added_relative_pose_options_ = false;
  added_rotation_averaging_options_ = false;
  added_track_establishment_options_ = false;
  added_global_positioning_options_ = false;
  added_bundle_adjustment_options_ = false;
  added_triangulation_options_ = false;
  added_inliers_options_ = false;
}

void OptionManager::ResetOptions(const bool reset_paths) {
  if (reset_paths) {
    *database_path = "";
    *image_path = "";
  }
  *mapper = GlobalMapperOptions();
}

void OptionManager::Parse(const int argc, char** argv) {
  config::variables_map vmap;

  try {
    config::store(config::parse_command_line(argc, argv, *desc_), vmap);

    if (vmap.count("help")) {
      LOG(ERROR) << "Options can be specified via command-line.\n" << *desc_;
      // NOLINTNEXTLINE(concurrency-mt-unsafe)
      exit(EXIT_SUCCESS);
    }

    vmap.notify();

  } catch (std::exception& exc) {
    LOG(ERROR) << "Failed to parse options - " << exc.what() << ".";
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    exit(EXIT_FAILURE);
  } catch (...) {
    LOG(ERROR) << "Failed to parse options for unknown reason.";
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    exit(EXIT_FAILURE);
  }
}

}  // namespace glomap
