
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/types.h"

#include "glomap/scene/view_graph.h"

#include <memory>
#include <optional>
#include <sstream>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

void BindViewGraph(py::module& m) {
  py::class_<ViewGraph, std::shared_ptr<ViewGraph>> PyViewGraph(m, "ViewGraph");
  PyViewGraph.def(py::init<>())
      .def_property_readonly("num_images",
          [](const ViewGraph& self) -> image_t {
            return self.num_images;
          },
          "Number of images.")
      .def_property_readonly("num_pairs",
          [](const ViewGraph& self) -> image_t {
            return self.num_pairs;
          },
          "Number of image pairs.")
      .def_property_readonly("image_pairs",
          [](const ViewGraph& self) -> const std::unordered_map<image_pair_t, ImagePair>& {
            return self.image_pairs;
          },
          py::return_value_policy::reference_internal,
          "The image pairs.");
  MakeDataclass(PyViewGraph);

}
