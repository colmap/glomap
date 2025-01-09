#include "glomap/estimators/global_positioning.h"

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

void BindGlobalPositionerOptions(py::module& m) {
  // Bind GlobalPositionerOptions
  py::class_<GlobalPositionerOptions, std::shared_ptr<GlobalPositionerOptions>>
      PyGlobalPositionerOptions(m, "GlobalPositionerOptions");

  PyGlobalPositionerOptions.def(py::init<>())
      .def_readwrite("thres_loss_function",
                     &GlobalPositionerOptions::thres_loss_function,
                     "The threshold for the loss function.")
      .def_readwrite("generate_random_positions",
                     &GlobalPositionerOptions::generate_random_positions,
                     "Whether initialize the reconstruction randomly.")
      .def_readwrite("generate_random_points",
                     &GlobalPositionerOptions::generate_random_points,
                     "Whether initialize the points randomly.")
      .def_readwrite("generate_scales",
                     &GlobalPositionerOptions::generate_scales,
                     "Whether initialize the scales randomly.")
      .def_readwrite("optimize_positions",
                     &GlobalPositionerOptions::optimize_positions,
                     "Whether optimize the positions.")
      .def_readwrite("optimize_points",
                     &GlobalPositionerOptions::optimize_points,
                     "Whether optimize the points.")
      .def_readwrite("optimize_scales",
                     &GlobalPositionerOptions::optimize_scales,
                     "Whether optimize the scales.")
      .def_readwrite("min_num_view_per_track",
                     &GlobalPositionerOptions::min_num_view_per_track,
                     "Constrain the minimum number of views per track.")
      .def_property(
          "max_num_iterations",
          [](const GlobalPositionerOptions& self) -> int {
            return self.solver_options.max_num_iterations;
          },
          [](GlobalPositionerOptions& self, const int value) {
            self.solver_options.max_num_iterations = value;
          });
  MakeDataclass(PyGlobalPositionerOptions);
}