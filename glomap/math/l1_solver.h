// This code is adapted from Theia library (http://theia-sfm.org/),
// with its original L1 solver adapted from
//  "https://web.stanford.edu/~boyd/papers/admm/least_abs_deviations/lad.html"

#pragma once

#include <colmap/util/logging.h>

#include <Eigen/Cholesky>
#include <Eigen/CholmodSupport>
#include <Eigen/Core>

// An L1 norm (|| A * x - b ||_1) approximation solver based on ADMM
// (alternating direction method of multipliers,
// https://web.stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf).
namespace glomap {

// TODO: L1 solver for dense matrix
struct L1SolverOptions {
  int max_num_iterations = 1000;
  // Rho is the augmented Lagrangian parameter.
  double rho = 1.0;
  // Alpha is the over-relaxation parameter (typically between 1.0 and 1.8).
  double alpha = 1.0;

  double absolute_tolerance = 1e-4;
  double relative_tolerance = 1e-2;
};

template <class MatrixType>
class L1Solver {
 public:
  L1Solver(const L1SolverOptions& options, const MatrixType& mat)
      : options_(options), a_(mat) {
    // Pre-compute the sparsity pattern.
    const MatrixType spd_mat = a_.transpose() * a_;
    linear_solver_.compute(spd_mat);
  }

  void Solve(const Eigen::VectorXd& rhs, Eigen::VectorXd* solution) {
    Eigen::VectorXd& x = *solution;
    Eigen::VectorXd z(a_.rows()), u(a_.rows());
    z.setZero();
    u.setZero();

    Eigen::VectorXd a_times_x(a_.rows()), z_old(z.size()), ax_hat(a_.rows());
    // Precompute some convergence terms.
    const double rhs_norm = rhs.norm();
    const double primal_abs_tolerance_eps =
        std::sqrt(a_.rows()) * options_.absolute_tolerance;
    const double dual_abs_tolerance_eps =
        std::sqrt(a_.cols()) * options_.absolute_tolerance;

    const std::string row_format =
        "  % 4d     % 4.4e     % 4.4e     % 4.4e     % 4.4e";
    for (int i = 0; i < options_.max_num_iterations; i++) {
      // Update x.
      x.noalias() = linear_solver_.solve(a_.transpose() * (rhs + z - u));
      if (linear_solver_.info() != Eigen::Success) {
        LOG(ERROR) << "L1 Minimization failed. Could not solve the sparse "
                      "linear system with Cholesky Decomposition";
        return;
      }

      a_times_x.noalias() = a_ * x;
      ax_hat.noalias() = options_.alpha * a_times_x;
      ax_hat.noalias() += (1.0 - options_.alpha) * (z + rhs);

      // Update z and set z_old.
      std::swap(z, z_old);
      z.noalias() = Shrinkage(ax_hat - rhs + u, 1.0 / options_.rho);

      // Update u.
      u.noalias() += ax_hat - z - rhs;

      // Compute the convergence terms.
      const double r_norm = (a_times_x - z - rhs).norm();
      const double s_norm =
          (-options_.rho * a_.transpose() * (z - z_old)).norm();
      const double max_norm = std::max({a_times_x.norm(), z.norm(), rhs_norm});
      const double primal_eps =
          primal_abs_tolerance_eps + options_.relative_tolerance * max_norm;
      const double dual_eps = dual_abs_tolerance_eps +
                              options_.relative_tolerance *
                                  (options_.rho * a_.transpose() * u).norm();

      // Determine if the minimizer has converged.
      if (r_norm < primal_eps && s_norm < dual_eps) {
        break;
      }
    }
  }

 private:
  const L1SolverOptions& options_;

  // Matrix A in || Ax - b ||_1
  const MatrixType a_;

  // Cholesky linear solver. Since our linear system will be a SPD matrix we can
  // utilize the Cholesky factorization.
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> linear_solver_;

  static Eigen::VectorXd Shrinkage(const Eigen::VectorXd& vec,
                                   const double kappa) {
    Eigen::ArrayXd zero_vec(vec.size());
    zero_vec.setZero();
    return zero_vec.max(vec.array() - kappa) -
           zero_vec.max(-vec.array() - kappa);
  }
};

}  // namespace glomap
