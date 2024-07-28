#include "glomap/estimators/view_graph_calibration.h"

#include "glomap/estimators/cost_function.h"
#include "glomap/math/two_view_geometry.h"

#include <colmap/scene/two_view_geometry.h>

namespace glomap {

bool ViewGraphCalibrator::Solve(ViewGraph& view_graph,
                                std::unordered_map<camera_t, Camera>& cameras,
                                std::unordered_map<image_t, Image>& images) {
  // Reset the problem
  LOG(INFO) << "Start ViewGraphCalibrator";
  Reset(cameras);

  // Set the solver options.
  if (cameras.size() < 50)
    options_.solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  else
    options_.solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  // Add the image pairs into the problem
  AddImagePairsToProblem(view_graph, cameras, images);

  // Set the cameras to be constant if they have prior intrinsics
  size_t num_cameras = ParameterizeCameras(cameras);

  if (num_cameras == 0) {
    LOG(INFO) << "No cameras to optimize";
    return true;
  }

  // Solve the problem
  ceres::Solver::Summary summary;
  options_.solver_options.minimizer_progress_to_stdout = options_.verbose;
  ceres::Solve(options_.solver_options, problem_.get(), &summary);

  // Print the summary only if verbose
  if (options_.verbose) LOG(INFO) << summary.FullReport();

  // Convert the results back to the camera
  ConvertResults(cameras);

  // Filter the image pairs
  FilterImagePair(view_graph, images);

  return summary.IsSolutionUsable();
}

void ViewGraphCalibrator::Reset(std::unordered_map<camera_t, Camera>& cameras) {
  // Initialize the problem
  focals_.clear();
  for (auto& [camera_id, camera] : cameras) {
    focals_[camera_id] = camera.Focal();
  }

  // Set up the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
  options_.loss_function =
      std::make_shared<ceres::CauchyLoss>(options_.thres_loss_function);
}

void ViewGraphCalibrator::AddImagePairsToProblem(
    ViewGraph& view_graph,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images) {
  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.config != colmap::TwoViewGeometry::CALIBRATED &&
        image_pair.config != colmap::TwoViewGeometry::UNCALIBRATED)
      continue;
    if (image_pair.is_valid == false) continue;

    AddImagePair(image_pair, cameras, images);
  }
}

void ViewGraphCalibrator::AddImagePair(
    ImagePair& image_pair,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images) {
  // Collect the cost terms
  image_t image_id1 = image_pair.image_id1;
  image_t image_id2 = image_pair.image_id2;

  camera_t camera_id1 = images.at(image_id1).camera_id;
  camera_t camera_id2 = images.at(image_id2).camera_id;

  ceres::CostFunction* cost_function;
  if (camera_id1 == camera_id2) {
    cost_function = FetzerFocalLengthSameCameraCost::Create(
        image_pair.F, cameras[camera_id1].PrincipalPoint());
    problem_->AddResidualBlock(
        cost_function, options_.loss_function.get(), &(focals_[camera_id1]));
  } else {
    cost_function =
        FetzerFocalLengthCost::Create(image_pair.F,
                                      cameras[camera_id1].PrincipalPoint(),
                                      cameras[camera_id2].PrincipalPoint());
    problem_->AddResidualBlock(cost_function,
                               options_.loss_function.get(),
                               &(focals_[camera_id1]),
                               &(focals_[camera_id2]));
  }
}

size_t ViewGraphCalibrator::ParameterizeCameras(
    std::unordered_map<camera_t, Camera>& cameras) {
  size_t num_cameras = 0;
  for (auto& [camera_id, camera] : cameras) {
    if (!problem_->HasParameterBlock(&(focals_[camera_id]))) continue;

    num_cameras++;
    problem_->SetParameterLowerBound(&(focals_[camera_id]), 0, 1e-3);
    if (camera.has_prior_focal_length) {
      problem_->SetParameterBlockConstant(&(focals_[camera_id]));
      num_cameras--;
    }
  }

  return num_cameras;
}

void ViewGraphCalibrator::ConvertResults(
    std::unordered_map<camera_t, Camera>& cameras) {
  size_t counter = 0;
  for (auto& [camera_id, camera] : cameras) {
    if (!problem_->HasParameterBlock(&(focals_[camera_id]))) continue;

    // if the estimated parameter is too crazy, reject it
    if (focals_[camera_id] / camera.Focal() > options_.thres_higher_ratio ||
        focals_[camera_id] / camera.Focal() < options_.thres_lower_ratio) {
      if (options_.verbose)
        LOG(INFO) << "NOT ACCEPTED: Camera " << camera_id
                  << " focal: " << focals_[camera_id]
                  << " original focal: " << camera.Focal();
      counter++;

      continue;
    }

    // Marke that the camera has refined intrinsics
    camera.has_refined_focal_length = true;

    // Update the focal length
    for (const size_t idx : camera.FocalLengthIdxs()) {
      camera.params[idx] = focals_[camera_id];
      if (options_.verbose)
        LOG(INFO) << "Camera " << idx << " focal: " << focals_[camera_id]
                  << std::endl;
    }
  }
  LOG(INFO) << counter << " cameras are rejected in view graph calibration";
}

size_t ViewGraphCalibrator::FilterImagePair(
    ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  ceres::Problem::EvaluateOptions EvalOpts;
  EvalOpts.num_threads = 16;
  EvalOpts.apply_loss_function = false;
  std::vector<double> residuals;
  problem_->Evaluate(EvalOpts, nullptr, &residuals, nullptr, nullptr);

  // Dump the residuals into the original data structure
  size_t counter = 0;
  size_t invalid_counter = 0;

  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.config != colmap::TwoViewGeometry::CALIBRATED &&
        image_pair.config != colmap::TwoViewGeometry::UNCALIBRATED)
      continue;
    if (image_pair.is_valid == false) continue;

    Eigen::Vector2d error =
        Eigen::Vector2d(residuals[counter], residuals[counter + 1]);

    image_t image_id1 = image_pair.image_id1;
    image_t image_id2 = image_pair.image_id2;

    // Set the two view geometry to be invalid if the error is too high
    if (error.norm() > options_.thres_two_view_error) {
      invalid_counter++;
      image_pair.is_valid = false;
    }
    counter += 2;
  }

  LOG(INFO) << "invalid / total number of two view geometry: "
            << invalid_counter << " / " << counter;

  return invalid_counter;
}

}  // namespace glomap