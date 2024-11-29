#include "option_manager.h"

namespace glomap {

namespace {
template<typename T>
inline constexpr bool always_false_v = false;

template<typename T>
T ParseValue(const std::string& value) {
  if constexpr (std::is_same_v<T, bool>) {
    return value == "true" || value == "1" || value == "yes";
  } else if constexpr (std::is_same_v<T, int>) {
    return std::stoi(value);
  } else if constexpr (std::is_same_v<T, double>) {
    return std::stod(value);
  } else if constexpr (std::is_same_v<T, std::string>) {
    return value;
  } else {
    static_assert(always_false_v<T>, "Unsupported option type");
    return T{};  // Never reached, just to satisfy compiler
  }
}
}

// Constructor and Initialization
OptionManager::OptionManager(bool add_project_options) {
  if (add_project_options) {
    InitializeDefaultOptions();
  }
}

void OptionManager::InitializeDefaultOptions() {
  if (initialized_) return;

  mapper = std::make_unique<GlobalMapperOptions>();

  // Register option groups
  option_groups_ = {
      {"database", {false, [this]() { AddDatabaseOptions(); }}},
      {"image", {false, [this]() { AddImageOptions(); }}},
      {"mapper", {false, [this]() { AddMapperOptions(); }}}
  };

  AddOptionWithDefault("help", new bool(false), "Show this help message", false);
  initialized_ = true;
}

// Core functionality
void OptionManager::Parse(std::span<char*> args) {
  if (args.empty()) {
    throw OptionError("No arguments provided");
  }

  args = args.subspan(1);  // Skip program name

  for (size_t i = 0; i < args.size(); ++i) {
    std::string_view arg = args[i];
    if (arg.starts_with("--")) {
      arg.remove_prefix(2);
    } else if (arg.starts_with('-')) {
      arg.remove_prefix(1);
    } else {
      throw OptionError("Invalid argument format: " + std::string(arg));
    }

    auto eq_pos = arg.find('=');
    std::string name{arg.substr(0, eq_pos)};

    if (name == "help") {
      PrintHelp();
      std::exit(0);
    }

    std::string value;
    if (eq_pos != std::string_view::npos) {
      value = std::string{arg.substr(eq_pos + 1)};
    } else if (i + 1 < args.size()) {
      value = args[++i];
    } else {
      throw OptionError("No value provided for option: " + name);
    }

    ParseOption(name + "=" + value);
  }

  ValidateOptions();
}

void OptionManager::ParseOption(const std::string& arg) {
  auto eq_pos = arg.find('=');
  if (eq_pos == std::string::npos) {
    throw OptionError("Invalid option format: " + arg);
  }

  std::string name = arg.substr(0, eq_pos);
  std::string value = arg.substr(eq_pos + 1);

  auto it = options_.find(name);
  if (it == options_.end()) {
    throw OptionError("Unknown option: " + name);
  }

  std::visit([&](auto&& opt) {
    using T = std::decay_t<decltype(*opt.value)>;
    *opt.value = ParseValue<T>(value);
  }, it->second);
}

void OptionManager::ValidateOptions() const {
  for (const auto& [name, option] : options_) {
    std::visit([&name](const auto& opt) {
      using T = std::remove_pointer_t<decltype(opt.value)>;
      if (opt.required && !opt.default_value &&
          (opt.value == nullptr || *opt.value == T{})) {
        throw OptionError("Required option '" + name + "' not provided");
      }
    }, option);
  }
}

void OptionManager::PrintHelp() const {
  std::cout << "Usage: program [options]\n\nOptions:\n";

  std::vector<std::pair<std::string, const OptionValue*>> sorted_options;
  for (const auto& [name, option] : options_) {
    sorted_options.emplace_back(name, &option);
  }
  std::ranges::sort(sorted_options, {}, &std::pair<std::string, const OptionValue*>::first);

  for (const auto& [name, option] : sorted_options) {
    std::visit([&](const auto& opt) {
      std::cout << "  --" << std::left << std::setw(30) << name
                << " " << opt.description << "\n";
      if (opt.default_value) {
        std::cout << std::string(32, ' ') << "Default: "
                  << *opt.default_value << "\n";
      }
      if (opt.required) {
        std::cout << std::string(32, ' ') << "Required\n";
      }
    }, *option);
  }
}

void OptionManager::Reset() {
  options_.clear();
  option_groups_.clear();
  initialized_ = false;
  InitializeDefaultOptions();
}

// Option group implementations
void OptionManager::AddDatabaseOptions() {
  if (auto& group = option_groups_["database"]; !group.is_added) {
    AddRequiredOption("database_path", &database_path, "Path to the database file");
    group.is_added = true;
  }
}

void OptionManager::AddImageOptions() {
  if (auto& group = option_groups_["image"]; !group.is_added) {
    AddRequiredOption("image_path", &image_path, "Path to the image directory");
    group.is_added = true;
  }
}

void OptionManager::AddMapperOptions() {
  if (auto& group = option_groups_["mapper"]; !group.is_added) {
    AddOptionWithDefault("ba_iteration_num",
                         &mapper->num_iteration_bundle_adjustment,
                         "Number of bundle adjustment iterations", 10);

    AddOptionWithDefault("retriangulation_iteration_num",
                         &mapper->num_iteration_retriangulation,
                         "Number of retriangulation iterations", 5);

    AddOptionWithDefault("skip_preprocessing",
                         &mapper->skip_preprocessing,
                         "Skip preprocessing step", false);

    AddOptionWithDefault("skip_view_graph_calibration",
                         &mapper->skip_view_graph_calibration,
                         "Skip view graph calibration step", false);

    AddOptionWithDefault("skip_relative_pose_estimation",
                         &mapper->skip_relative_pose_estimation,
                         "Skip relative pose estimation step", false);

    AddOptionWithDefault("skip_rotation_averaging",
                         &mapper->skip_rotation_averaging,
                         "Skip rotation averaging step", false);

    AddOptionWithDefault("skip_global_positioning",
                         &mapper->skip_global_positioning,
                         "Skip global positioning step", false);

    AddOptionWithDefault("skip_bundle_adjustment",
                         &mapper->skip_bundle_adjustment,
                         "Skip bundle adjustment step", false);

    AddOptionWithDefault("skip_retriangulation",
                         &mapper->skip_retriangulation,
                         "Skip retriangulation step", false);

    AddOptionWithDefault("skip_pruning",
                         &mapper->skip_pruning,
                         "Skip pruning step", false);

    group.is_added = true;
  }
}

void OptionManager::AddMapperResumeOptions() {
  if (auto& group = option_groups_["mapper_resume"]; !group.is_added) {
    // Set non-configurable options
    mapper->skip_preprocessing = true;
    mapper->skip_view_graph_calibration = true;
    mapper->skip_relative_pose_estimation = true;
    mapper->skip_rotation_averaging = true;
    mapper->skip_track_establishment = true;
    mapper->skip_retriangulation = true;

    AddOptionWithDefault("ba_iteration_num",
                         &mapper->num_iteration_bundle_adjustment,
                         "Number of bundle adjustment iterations", 10);

    AddOptionWithDefault("retriangulation_iteration_num",
                         &mapper->num_iteration_retriangulation,
                         "Number of retriangulation iterations", 5);

    AddOptionWithDefault("skip_global_positioning",
                         &mapper->skip_global_positioning,
                         "Skip global positioning step", false);

    AddOptionWithDefault("skip_bundle_adjustment",
                         &mapper->skip_bundle_adjustment,
                         "Skip bundle adjustment step", false);

    AddOptionWithDefault("skip_pruning",
                         &mapper->skip_pruning,
                         "Skip pruning step", false);

    group.is_added = true;
  }
}

void OptionManager::AddViewGraphCalibrationOptions() {
  if (auto& group = option_groups_["view_graph_calibration"]; !group.is_added) {
    AddOptionWithDefault("ViewGraphCalib.thres_lower_ratio",
                         &mapper->opt_vgcalib.thres_lower_ratio,
                         "Lower threshold ratio for view graph calibration", 0.75);

    AddOptionWithDefault("ViewGraphCalib.thres_higher_ratio",
                         &mapper->opt_vgcalib.thres_higher_ratio,
                         "Higher threshold ratio for view graph calibration", 0.85);

    AddOptionWithDefault("ViewGraphCalib.thres_two_view_error",
                         &mapper->opt_vgcalib.thres_two_view_error,
                         "Two-view error threshold for calibration", 4.0);

    group.is_added = true;
  }
}

void OptionManager::AddRelativePoseEstimationOptions() {
  if (auto& group = option_groups_["relative_pose"]; !group.is_added) {
    AddOptionWithDefault("RelPoseEstimation.max_epipolar_error",
                         &mapper->opt_relpose.ransac_options.max_epipolar_error,
                         "Maximum epipolar error for RANSAC", 4.0);

    group.is_added = true;
  }
}

void OptionManager::AddRotationEstimatorOptions() {
  if (auto& group = option_groups_["rotation_estimator"]; !group.is_added) {
    // Currently empty as per original code comment
    // TODO: Add rotation averaging options when implemented
    group.is_added = true;
  }
}

void OptionManager::AddTrackEstablishmentOptions() {
  if (auto& group = option_groups_["track_establishment"]; !group.is_added) {
    AddOptionWithDefault("TrackEstablishment.min_num_tracks_per_view",
                         &mapper->opt_track.min_num_tracks_per_view,
                         "Minimum number of tracks per view", 20);

    AddOptionWithDefault("TrackEstablishment.min_num_view_per_track",
                         &mapper->opt_track.min_num_view_per_track,
                         "Minimum number of views per track", 2);

    AddOptionWithDefault("TrackEstablishment.max_num_view_per_track",
                         &mapper->opt_track.max_num_view_per_track,
                         "Maximum number of views per track", 100);

    AddOptionWithDefault("TrackEstablishment.max_num_tracks",
                         &mapper->opt_track.max_num_tracks,
                         "Maximum number of tracks", 1000000);

    group.is_added = true;
  }
}

void OptionManager::AddGlobalPositionerOptions() {
  if (auto& group = option_groups_["global_positioner"]; !group.is_added) {
    AddOptionWithDefault("GlobalPositioning.optimize_positions",
                         &mapper->opt_gp.optimize_positions,
                         "Optimize camera positions", true);

    AddOptionWithDefault("GlobalPositioning.optimize_points",
                         &mapper->opt_gp.optimize_points,
                         "Optimize 3D points", true);

    AddOptionWithDefault("GlobalPositioning.optimize_scales",
                         &mapper->opt_gp.optimize_scales,
                         "Optimize relative scales", true);

    AddOptionWithDefault("GlobalPositioning.thres_loss_function",
                         &mapper->opt_gp.thres_loss_function,
                         "Loss function threshold", 4.0);

    AddOptionWithDefault("GlobalPositioning.max_num_iterations",
                         &mapper->opt_gp.solver_options.max_num_iterations,
                         "Maximum number of iterations", 100);

    group.is_added = true;
  }
}

void OptionManager::AddBundleAdjusterOptions() {
  if (auto& group = option_groups_["bundle_adjuster"]; !group.is_added) {
    AddOptionWithDefault("BundleAdjustment.optimize_rotations",
                         &mapper->opt_ba.optimize_rotations,
                         "Optimize camera rotations", true);

    AddOptionWithDefault("BundleAdjustment.optimize_translation",
                         &mapper->opt_ba.optimize_translation,
                         "Optimize camera translations", true);

    AddOptionWithDefault("BundleAdjustment.optimize_intrinsics",
                         &mapper->opt_ba.optimize_intrinsics,
                         "Optimize camera intrinsics", false);

    AddOptionWithDefault("BundleAdjustment.optimize_points",
                         &mapper->opt_ba.optimize_points,
                         "Optimize 3D points", true);

    AddOptionWithDefault("BundleAdjustment.thres_loss_function",
                         &mapper->opt_ba.thres_loss_function,
                         "Loss function threshold", 4.0);

    AddOptionWithDefault("BundleAdjustment.max_num_iterations",
                         &mapper->opt_ba.solver_options.max_num_iterations,
                         "Maximum number of iterations", 100);

    group.is_added = true;
  }
}

void OptionManager::AddTriangulatorOptions() {
  if (auto& group = option_groups_["triangulator"]; !group.is_added) {
    AddOptionWithDefault("Triangulation.complete_max_reproj_error",
                         &mapper->opt_triangulator.tri_complete_max_reproj_error,
                         "Maximum reprojection error for complete triangulation", 4.0);

    AddOptionWithDefault("Triangulation.merge_max_reproj_error",
                         &mapper->opt_triangulator.tri_merge_max_reproj_error,
                         "Maximum reprojection error for merging", 4.0);

    AddOptionWithDefault("Triangulation.min_angle",
                         &mapper->opt_triangulator.tri_min_angle,
                         "Minimum triangulation angle in degrees", 1.5);

    AddOptionWithDefault("Triangulation.min_num_matches",
                         &mapper->opt_triangulator.min_num_matches,
                         "Minimum number of matches for triangulation", 3);

    group.is_added = true;
  }
}

void OptionManager::AddInlierThresholdOptions() {
  if (auto& group = option_groups_["inlier_thresholds"]; !group.is_added) {
    AddOptionWithDefault("Thresholds.max_epipolar_error_E",
                         &mapper->inlier_thresholds.max_epipolar_error_E,
                         "Maximum epipolar error for essential matrix", 4.0);

    AddOptionWithDefault("Thresholds.max_epipolar_error_F",
                         &mapper->inlier_thresholds.max_epipolar_error_F,
                         "Maximum epipolar error for fundamental matrix", 4.0);

    AddOptionWithDefault("Thresholds.max_epipolar_error_H",
                         &mapper->inlier_thresholds.max_epipolar_error_H,
                         "Maximum epipolar error for homography matrix", 4.0);

    AddOptionWithDefault("Thresholds.min_inlier_num",
                         &mapper->inlier_thresholds.min_inlier_num,
                         "Minimum number of inliers", 30.0);

    AddOptionWithDefault("Thresholds.min_inlier_ratio",
                         &mapper->inlier_thresholds.min_inlier_ratio,
                         "Minimum inlier ratio", 0.25);

    AddOptionWithDefault("Thresholds.max_rotation_error",
                         &mapper->inlier_thresholds.max_rotation_error,
                         "Maximum rotation error in degrees", 5.0);

    group.is_added = true;
  }
}

} // namespace glomap