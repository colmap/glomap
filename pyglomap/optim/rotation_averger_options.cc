
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"

#include "glomap/estimators/global_rotation_averaging.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

void BindRotationAveragerOptions(py::module& m) {
  // Bind RotationEstimatorOptions
  py::class_<RotationEstimatorOptions, std::shared_ptr<RotationEstimatorOptions>> PyRotationEstimatorOptions(m, "RotationEstimatorOptions");

  PyRotationEstimatorOptions.def(py::init<>())
    .def_readwrite("max_num_l1_iterations",
      &RotationEstimatorOptions::max_num_l1_iterations,
      "The maximum number of L1 iterations.")
    .def_readwrite("l1_step_convergence_threshold",
      &RotationEstimatorOptions::l1_step_convergence_threshold,
      "The L1 step convergence threshold.")
    .def_readwrite("max_num_irls_iterations",
      &RotationEstimatorOptions::max_num_irls_iterations,
      "The maximum number of IRLS iterations.")
    .def_readwrite("irls_step_convergence_threshold",
      &RotationEstimatorOptions::irls_step_convergence_threshold,
      "The IRLS step convergence threshold.")
    // .def_readwrite("axis",
    //   &RotationEstimatorOptions::axis,
    //   "The axis of rotation.")
    .def_readwrite("irls_loss_parameter_sigma",
      &RotationEstimatorOptions::irls_loss_parameter_sigma,
      "The IRLS loss parameter sigma.")
    // .def_readwrite("weight_type",
    //   &RotationEstimatorOptions::weight_type,
    //   "The weight type.")
    // .def_readwrite("use_gravity",
    //   &RotationEstimatorOptions::use_gravity,
    //   "Whether to use gravity.")
    .def_readwrite("use_weight",
      &RotationEstimatorOptions::use_weight,
      "Whether to use weight.")
    .def_readwrite("skip_initialization",
      &RotationEstimatorOptions::skip_initialization,
      "Whether to skip initialization with Maximum Spanning Tree.");
    
  MakeDataclass(PyRotationEstimatorOptions);
}
