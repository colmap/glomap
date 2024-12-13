#include "glomap/estimators/global_positioning.h"
#include "glomap/processors/image_undistorter.h"
#include "glomap/processors/reconstruction_normalizer.h"
#include "glomap/scene/types_sfm.h"

#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/colmap_bindings.h"

#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

py::dict RunGlobalPostioning(
            ViewGraph& view_graph,
            std::unordered_map<camera_t, Camera>& cameras,
            std::unordered_map<image_t, Image>& images,
            std::unordered_map<track_t, Track>& tracks,
            GlobalPositionerOptions options) {
              
  // Establish the maximum connected component
  view_graph.KeepLargestConnectedComponents(images);

  // Undistort images
  UndistortImages(cameras, images);
  GlobalPositioner gp_engine(options);
  gp_engine.Solve(view_graph, cameras, images, tracks);

  NormalizeReconstruction(cameras, images, tracks);

  py::dict output;
  output["cameras"] = cameras;
  output["images"] = images;
  output["tracks"] = tracks;

  return output;
}

void BindGlobalPositioning(py::module& m) {
  m.def(
      "run_global_positioning",
      &RunGlobalPostioning,
      "view_graph"_a,
      "cameras"_a,
      "images"_a,
      "tracks"_a,
      py::arg_v("options", GlobalPositionerOptions(), "GlobalPositionerOptions()"),
      "Run global positioning");

}
