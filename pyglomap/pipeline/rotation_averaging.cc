#include "glomap/estimators/global_rotation_averaging.h"
#include "glomap/scene/types_sfm.h"

#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"

#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

std::unordered_map<image_t, Image> RunRotationAveraging(
            ViewGraph& view_graph,
            std::unordered_map<image_t, Image>& images,
            RotationEstimatorOptions options) {

  // Establish the maximum connected component
  view_graph.KeepLargestConnectedComponents(images);

  // Undistort images
  RotationEstimator ra_engine(options);
  ra_engine.EstimateRotations(view_graph, images);

  return images;
}

void BindRotationAveraging(py::module& m) {
  m.def(
      "run_rotation_averaging",
      &RunRotationAveraging,
      "view_graph"_a,
      "images"_a,
      py::arg_v("options", RotationEstimatorOptions(), "RotationEstimatorOptions()"),
      "Estimate global rotations from relative rotations");
}
