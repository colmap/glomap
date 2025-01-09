#include "glomap/estimators/bundle_adjustment.h"

#include <memory>
#include <sstream>

#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

void BindBundleAdjusterOptions(py::module& m) {
  // Bind BundleAdjusterOptions
  py::class_<BundleAdjusterOptions, std::shared_ptr<BundleAdjusterOptions>>
      PyBundleAdjusterOptions(m, "BundleAdjusterOptions");

  PyBundleAdjusterOptions.def(py::init<>())
      .def_readwrite("thres_loss_function",
                     &BundleAdjusterOptions::thres_loss_function,
                     "The threshold for the loss function.")
      .def_readwrite("optimize_rotations",
                     &BundleAdjusterOptions::optimize_rotations,
                     "Whether optimize the rotations.")
      .def_readwrite("optimize_translation",
                     &BundleAdjusterOptions::optimize_translation,
                     "Whether optimize the translation.")
      .def_readwrite("optimize_intrinsics",
                     &BundleAdjusterOptions::optimize_intrinsics,
                     "Whether optimize the intrinsics.")
      .def_readwrite("optimize_points",
                     &BundleAdjusterOptions::optimize_points,
                     "Whether optimize the points.")
      .def_readwrite("min_num_view_per_track",
                     &BundleAdjusterOptions::min_num_view_per_track,
                     "Constrain the minimum number of views per track.")
      .def_property(
          "max_num_iterations",
          [](const BundleAdjusterOptions& self) -> int {
            return self.solver_options.max_num_iterations;
          },
          [](BundleAdjusterOptions& self, const int value) {
            self.solver_options.max_num_iterations = value;
          });

  MakeDataclass(PyBundleAdjusterOptions);
}
