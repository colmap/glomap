
#ifndef GLOMAP_ESTIMATORS_OPTIMIZATION_BASE_H_
#define GLOMAP_ESTIMATORS_OPTIMIZATION_BASE_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace glomap {
struct OptimizationBaseOptions {
    // Logging control
    bool verbose = false;

    // The threshold for the loss function
    double thres_loss_function = 1e-1;

    // The loss function for the calibration
    ceres::LossFunction* loss_function;

    // The options for the solver
    ceres::Solver::Options solver_options;

    OptimizationBaseOptions() {
        solver_options.num_threads = 16;
        solver_options.max_num_iterations = 100;
        solver_options.minimizer_progress_to_stdout = verbose;
        solver_options.function_tolerance = 1e-5;
    }

};

}  // namespace glomap


#endif  // GLOMAP_ESTIMATORS_OPTIMIZATION_BASE_H_