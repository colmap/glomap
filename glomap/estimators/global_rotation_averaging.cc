#include "global_rotation_averaging.h"

#include "glomap/math/l1_solver.h"
#include "glomap/math/rigid3d.h"
#include "glomap/math/tree.h"

#include <iostream>
#include <queue>

namespace glomap {
namespace {
double RelAngleError(double angle_12, double angle_1, double angle_2) {
  double est = (angle_2 - angle_1) - angle_12;

  while (est >= EIGEN_PI) est -= TWO_PI;

  while (est < -EIGEN_PI) est += TWO_PI;

  // Inject random noise if the angle is too close to the boundary to break the
  // possible balance at the local minima
  if (est > EIGEN_PI - 0.01 || est < -EIGEN_PI + 0.01) {
    if (est < 0)
      est += (rand() % 1000) / 1000.0 * 0.01;
    else
      est -= (rand() % 1000) / 1000.0 * 0.01;
  }

  return est;
}
}  // namespace

bool RotationEstimator::EstimateRotations(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  // Initialize the rotation from maximum spanning tree
  if (!options_.skip_initialization && !options_.use_gravity) {
    InitializeFromMaximumSpanningTree(view_graph, images);
  }

  // Set up the linear system
  SetupLinearSystem(view_graph, images);

  // Solve the linear system for L1 norm optimization
  if (options_.max_num_l1_iterations > 0) {
    if (!SolveL1Regression(view_graph, images)) {
      return false;
    }
  }

  // Solve the linear system for IRLS optimization
  if (options_.max_num_irls_iterations > 0) {
    if (!SolveIRLS(view_graph, images)) {
      return false;
    }
  }

  // Convert the final results
  for (auto& [image_id, image] : images) {
    if (!image.is_registered) continue;

    if (options_.use_gravity && image.gravity_info.has_gravity) {
      image.cam_from_world.rotation = Eigen::Quaterniond(
          image.gravity_info.GetRAlign() *
          AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id]]));
    } else {
      image.cam_from_world.rotation = Eigen::Quaterniond(AngleAxisToRotation(
          rotation_estimated_.segment(image_id_to_idx_[image_id], 3)));
    }
    // Restore the prior position (t = -Rc = R * R_ori * t_ori = R * t_ori)
    image.cam_from_world.translation =
        (image.cam_from_world.rotation * image.cam_from_world.translation);
  }

  return true;
}

void RotationEstimator::InitializeFromMaximumSpanningTree(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  // Here, we assume that largest connected component is already retrieved, so
  // we do not need to do that again compute maximum spanning tree.
  std::unordered_map<image_t, image_t> parents;
  image_t root = MaximumSpanningTree(view_graph, images, parents, INLIER_NUM);

  // Iterate through the tree to initialize the rotation
  // Establish child info
  std::unordered_map<image_t, std::vector<image_t>> children;
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    children.insert(std::make_pair(image_id, std::vector<image_t>()));
  }
  for (auto& [child, parent] : parents) {
    if (root == child) continue;
    children[parent].emplace_back(child);
  }

  std::queue<image_t> indexes;
  indexes.push(root);

  while (!indexes.empty()) {
    image_t curr = indexes.front();
    indexes.pop();

    // Add all children into the tree
    for (auto& child : children[curr]) indexes.push(child);
    // If it is root, then fix it to be the original estimation
    if (curr == root) continue;

    // Directly use the relative pose for estimation rotation
    const ImagePair& image_pair = view_graph.image_pairs.at(
        ImagePair::ImagePairToPairId(curr, parents[curr]));
    if (image_pair.image_id1 == curr) {
      // 1_R_w = 2_R_1^T * 2_R_w
      images[curr].cam_from_world.rotation =
          (Inverse(image_pair.cam2_from_cam1) *
           images[parents[curr]].cam_from_world)
              .rotation;
    } else {
      // 2_R_w = 2_R_1 * 1_R_w
      images[curr].cam_from_world.rotation =
          (image_pair.cam2_from_cam1 * images[parents[curr]].cam_from_world)
              .rotation;
    }
  }
}

void RotationEstimator::SetupLinearSystem(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  // Clear all the structures
  sparse_matrix_.resize(0, 0);
  tangent_space_step_.resize(0);
  tangent_space_residual_.resize(0);
  rotation_estimated_.resize(0);
  image_id_to_idx_.clear();
  rel_temp_info_.clear();

  // Initialize the structures for estimated rotation
  image_id_to_idx_.reserve(images.size());
  rotation_estimated_.resize(
      3 * images.size());  // allocate more memory than needed
  image_t num_dof = 0;
  for (auto& [image_id, image] : images) {
    if (!image.is_registered) continue;
    image_id_to_idx_[image_id] = num_dof;
    if (options_.use_gravity && image.gravity_info.has_gravity) {
      rotation_estimated_[num_dof] =
          RotUpToAngle(image.gravity_info.GetRAlign().transpose() *
                       image.cam_from_world.rotation.toRotationMatrix());
      num_dof++;

      if (fixed_camera_id_ == -1) {
        fixed_camera_rotation_ =
            Eigen::Vector3d(0, rotation_estimated_[num_dof - 1], 0);
        fixed_camera_id_ = image_id;
      }
    } else {
      rotation_estimated_.segment(num_dof, 3) =
          Rigid3dToAngleAxis(image.cam_from_world);
      num_dof += 3;
    }
  }

  // If no cameras are set to be fixed, then take the first camera
  if (fixed_camera_id_ == -1) {
    for (auto& [image_id, image] : images) {
      if (!image.is_registered) continue;
      fixed_camera_id_ = image_id;
      fixed_camera_rotation_ = Rigid3dToAngleAxis(image.cam_from_world);
      break;
    }
  }

  rotation_estimated_.conservativeResize(num_dof);

  // Prepare the relative information
  int counter = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;

    int image_id1 = image_pair.image_id1;
    int image_id2 = image_pair.image_id2;

    rel_temp_info_[pair_id].R_rel =
        image_pair.cam2_from_cam1.rotation.toRotationMatrix();

    // Align the relative rotation to the gravity
    if (options_.use_gravity) {
      if (images[image_id1].gravity_info.has_gravity) {
        rel_temp_info_[pair_id].R_rel =
            rel_temp_info_[pair_id].R_rel *
            images[image_id1].gravity_info.GetRAlign();
      }

      if (images[image_id2].gravity_info.has_gravity) {
        rel_temp_info_[pair_id].R_rel =
            images[image_id2].gravity_info.GetRAlign().transpose() *
            rel_temp_info_[pair_id].R_rel;
      }
    }

    if (options_.use_gravity && images[image_id1].gravity_info.has_gravity &&
        images[image_id2].gravity_info.has_gravity) {
      counter++;
      Eigen::Vector3d aa = RotationToAngleAxis(rel_temp_info_[pair_id].R_rel);
      double error = aa[0] * aa[0] + aa[2] * aa[2];

      // Keep track of the error for x and z axis for gravity-aligned relative
      // pose
      rel_temp_info_[pair_id].xz_error = error;
      rel_temp_info_[pair_id].has_gravity = true;
      rel_temp_info_[pair_id].angle_rel = aa[1];
    } else {
      rel_temp_info_[pair_id].has_gravity = false;
    }
  }

  VLOG(2) << counter << " image pairs are gravity aligned" << std::endl;

  std::vector<Eigen::Triplet<double>> coeffs;
  coeffs.reserve(rel_temp_info_.size() * 6 + 3);

  // Establish linear systems
  size_t curr_pos = 0;
  std::vector<double> weights;
  weights.reserve(3 * view_graph.image_pairs.size());
  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;

    int image_id1 = image_pair.image_id1;
    int image_id2 = image_pair.image_id2;

    int vector_idx1 = image_id_to_idx_[image_id1];
    int vector_idx2 = image_id_to_idx_[image_id2];

    rel_temp_info_[pair_id].index = curr_pos;

    if (rel_temp_info_[pair_id].has_gravity) {
      coeffs.emplace_back(Eigen::Triplet<double>(curr_pos, vector_idx1, -1));
      coeffs.emplace_back(Eigen::Triplet<double>(curr_pos, vector_idx2, 1));
      if (image_pair.weight >= 0)
        weights.emplace_back(image_pair.weight);
      else
        weights.emplace_back(1);
      curr_pos++;
    } else {
      // If it is not gravity aligned, then we need to consider 3 dof
      if (!options_.use_gravity ||
          !images[image_id1].gravity_info.has_gravity) {
        for (int i = 0; i < 3; i++) {
          coeffs.emplace_back(
              Eigen::Triplet<double>(curr_pos + i, vector_idx1 + i, -1));
        }
      } else
        // else, other components are zero, and can be safely ignored
        coeffs.emplace_back(
            Eigen::Triplet<double>(curr_pos + 1, vector_idx1, -1));

      // Similarly for the second componenet
      if (!options_.use_gravity ||
          !images[image_id2].gravity_info.has_gravity) {
        for (int i = 0; i < 3; i++) {
          coeffs.emplace_back(
              Eigen::Triplet<double>(curr_pos + i, vector_idx2 + i, 1));
        }
      } else
        coeffs.emplace_back(
            Eigen::Triplet<double>(curr_pos + 1, vector_idx2, 1));
      for (int i = 0; i < 3; i++) {
        if (image_pair.weight >= 0)
          weights.emplace_back(image_pair.weight);
        else
          weights.emplace_back(1);
      }

      curr_pos += 3;
    }
  }

  // Set some cameras to be fixed
  // if some cameras have gravity, then add a single term constraint
  // Else, change to 3 constriants
  if (options_.use_gravity &&
      images[fixed_camera_id_].gravity_info.has_gravity) {
    coeffs.emplace_back(Eigen::Triplet<double>(
        curr_pos, image_id_to_idx_[fixed_camera_id_], 1));
    weights.emplace_back(1);
    curr_pos++;
  } else {
    for (int i = 0; i < 3; i++) {
      coeffs.emplace_back(Eigen::Triplet<double>(
          curr_pos + i, image_id_to_idx_[fixed_camera_id_] + i, 1));
      weights.emplace_back(1);
    }
    curr_pos += 3;
  }

  sparse_matrix_.resize(curr_pos, num_dof);
  sparse_matrix_.setFromTriplets(coeffs.begin(), coeffs.end());

  // Set up the weight matrix for the linear system
  if (!options_.use_weight) {
    weights_ = Eigen::ArrayXd::Ones(curr_pos);
  } else {
    weights_ = Eigen::ArrayXd(weights.size());
    for (size_t i = 0; i < weights.size(); i++) weights_[i] = weights[i];
  }

  // Initialize x and b
  tangent_space_step_.resize(num_dof);
  tangent_space_residual_.resize(curr_pos);
}

bool RotationEstimator::SolveL1Regression(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  L1SolverOptions opt_l1_solver;
  opt_l1_solver.max_num_iterations = 10;

  L1Solver<Eigen::SparseMatrix<double>> l1_solver(
      opt_l1_solver, weights_.matrix().asDiagonal() * sparse_matrix_);
  double last_norm = 0;
  double curr_norm = 0;

  ComputeResiduals(view_graph, images);
  VLOG(2) << "ComputeResiduals done";

  int iteration = 0;
  for (iteration = 0; iteration < options_.max_num_l1_iterations; iteration++) {
    VLOG(2) << "L1 ADMM iteration: " << iteration;

    last_norm = curr_norm;
    // use the current residual as b (Ax - b)

    tangent_space_step_.setZero();
    l1_solver.Solve(weights_.matrix().asDiagonal() * tangent_space_residual_,
                    &tangent_space_step_);
    if (tangent_space_step_.array().isNaN().any()) {
      LOG(ERROR) << "nan error";
      iteration++;
      return false;
    }

    if (VLOG_IS_ON(2))
      LOG(INFO) << "residual:"
                << (sparse_matrix_ * tangent_space_step_ -
                    tangent_space_residual_)
                       .array()
                       .abs()
                       .sum();

    curr_norm = tangent_space_step_.norm();
    UpdateGlobalRotations(view_graph, images);
    ComputeResiduals(view_graph, images);

    // Check the residual. If it is small, stop
    // TODO: strange bug for the L1 solver: update norm state constant
    if (ComputeAverageStepSize(images) <
            options_.l1_step_convergence_threshold ||
        std::abs(last_norm - curr_norm) < EPS) {
      if (std::abs(last_norm - curr_norm) < EPS)
        LOG(INFO) << "std::abs(last_norm - curr_norm) < EPS";
      iteration++;
      break;
    }
    opt_l1_solver.max_num_iterations =
        std::min(opt_l1_solver.max_num_iterations * 2, 100);
  }
  VLOG(2) << "L1 ADMM total iteration: " << iteration;
  return true;
}

bool RotationEstimator::SolveIRLS(const ViewGraph& view_graph,
                                  std::unordered_map<image_t, Image>& images) {
  // TODO: Determine what is the best solver for this part
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> llt;

  // weight_matrix.setIdentity();
  // sparse_matrix_ = A_ori;

  llt.analyzePattern(sparse_matrix_.transpose() * sparse_matrix_);

  const double sigma = DegToRad(options_.irls_loss_parameter_sigma);
  VLOG(2) << "sigma: " << options_.irls_loss_parameter_sigma;

  Eigen::ArrayXd weights_irls(sparse_matrix_.rows());
  Eigen::SparseMatrix<double> at_weight;

  if (options_.use_gravity && images[fixed_camera_id_].gravity_info.has_gravity)
    weights_irls[sparse_matrix_.rows() - 1] = 1;
  else
    weights_irls.segment(sparse_matrix_.rows() - 3, 3).setConstant(1);

  ComputeResiduals(view_graph, images);
  int iteration = 0;
  for (iteration = 0; iteration < options_.max_num_irls_iterations;
       iteration++) {
    VLOG(2) << "IRLS iteration: " << iteration;

    // Compute the weights for IRLS
    for (auto& [pair_id, pair_info] : rel_temp_info_) {
      image_pair_t image_pair_pos = pair_info.index;
      double err_squared = 0;
      double w = 0;
      // If both cameras have gravity, then we only consider the y-axis
      if (pair_info.has_gravity)
        err_squared = std::pow(tangent_space_residual_[image_pair_pos], 2) +
                      pair_info.xz_error;
      // Otherwise, we consider all 3 dof
      else
        err_squared =
            tangent_space_residual_.segment<3>(image_pair_pos).squaredNorm();

      // Compute the weight
      if (options_.weight_type == RotationEstimatorOptions::GEMAN_MCCLURE) {
        double tmp = err_squared + sigma * sigma;
        w = sigma * sigma / (tmp * tmp);
      } else if (options_.weight_type == RotationEstimatorOptions::HALF_NORM) {
        w = std::pow(err_squared, (0.5 - 2) / 2);
      }

      if (std::isnan(w)) {
        LOG(ERROR) << "nan weight!";
        return false;
      }

      // If both cameras have gravity, then only 1 equation
      if (pair_info.has_gravity) weights_irls[image_pair_pos] = w;
      // Otherwise, 3 equations
      else
        weights_irls.segment<3>(image_pair_pos).setConstant(w);
    }

    // Update the factorization for the weighted values.
    at_weight = sparse_matrix_.transpose() *
                weights_irls.matrix().asDiagonal() *
                weights_.matrix().asDiagonal();

    llt.factorize(at_weight * sparse_matrix_);

    // Solve the least squares problem..
    tangent_space_step_.setZero();
    tangent_space_step_ = llt.solve(at_weight * tangent_space_residual_);
    UpdateGlobalRotations(view_graph, images);
    ComputeResiduals(view_graph, images);

    // Check the residual. If it is small, stop
    if (ComputeAverageStepSize(images) <
        options_.irls_step_convergence_threshold) {
      iteration++;
      break;
    }
  }
  VLOG(2) << "IRLS total iteration: " << iteration;

  return true;
}

void RotationEstimator::UpdateGlobalRotations(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;

    image_t vector_idx = image_id_to_idx_[image_id];
    if (!(options_.use_gravity && image.gravity_info.has_gravity)) {
      Eigen::Matrix3d R_ori =
          AngleAxisToRotation(rotation_estimated_.segment(vector_idx, 3));

      rotation_estimated_.segment(vector_idx, 3) = RotationToAngleAxis(
          R_ori *
          AngleAxisToRotation(-tangent_space_step_.segment(vector_idx, 3)));
    } else {
      rotation_estimated_[vector_idx] -= tangent_space_step_[vector_idx];
    }
  }
}

void RotationEstimator::ComputeResiduals(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  int curr_pos = 0;
  for (auto& [pair_id, pair_info] : rel_temp_info_) {
    image_t image_id1 = view_graph.image_pairs.at(pair_id).image_id1;
    image_t image_id2 = view_graph.image_pairs.at(pair_id).image_id2;

    image_t idx1 = image_id_to_idx_[image_id1];
    image_t idx2 = image_id_to_idx_[image_id2];

    if (pair_info.has_gravity) {
      tangent_space_residual_[pair_info.index] =
          (RelAngleError(pair_info.angle_rel,
                         rotation_estimated_[image_id_to_idx_[image_id1]],
                         rotation_estimated_[image_id_to_idx_[image_id2]]));
    } else {
      Eigen::Matrix3d R_1, R_2;
      if (options_.use_gravity && images[image_id1].gravity_info.has_gravity) {
        R_1 = AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id1]]);
      } else {
        R_1 = AngleAxisToRotation(
            rotation_estimated_.segment(image_id_to_idx_[image_id1], 3));
      }

      if (options_.use_gravity && images[image_id2].gravity_info.has_gravity) {
        R_2 = AngleToRotUp(rotation_estimated_[image_id_to_idx_[image_id2]]);
      } else {
        R_2 = AngleAxisToRotation(
            rotation_estimated_.segment(image_id_to_idx_[image_id2], 3));
      }

      tangent_space_residual_.segment(pair_info.index, 3) =
          -RotationToAngleAxis(R_2.transpose() * pair_info.R_rel * R_1);
    }
  }

  if (options_.use_gravity && images[fixed_camera_id_].gravity_info.has_gravity)
    tangent_space_residual_[tangent_space_residual_.size() - 1] =
        rotation_estimated_[image_id_to_idx_[fixed_camera_id_]] -
        fixed_camera_rotation_[1];
  else
    tangent_space_residual_.segment(tangent_space_residual_.size() - 3, 3) =
        RotationToAngleAxis(
            AngleAxisToRotation(fixed_camera_rotation_).transpose() *
            AngleAxisToRotation(rotation_estimated_.segment(
                image_id_to_idx_[fixed_camera_id_], 3)));
}

double RotationEstimator::ComputeAverageStepSize(
    const std::unordered_map<image_t, Image>& images) {
  double total_update = 0;
  for (const auto& [image_id, image] : images) {
    if (!image.is_registered) continue;

    if (options_.use_gravity && image.gravity_info.has_gravity) {
      total_update += std::abs(tangent_space_step_[image_id_to_idx_[image_id]]);
    } else {
      total_update +=
          tangent_space_step_.segment(image_id_to_idx_[image_id], 3).norm();
    }
  }
  return total_update / image_id_to_idx_.size();
}

}  // namespace glomap