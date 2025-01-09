#include "glomap/processors/track_filter.h"

#include "glomap/scene/types_sfm.h"

#include <memory>

#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/colmap_bindings.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

py::dict RunFilterTracksByReprojection(
    ViewGraph& view_graph,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks,
    double max_reprojection_error,
    bool in_normalized_image) {
  int counter = TrackFilter::FilterTracksByReprojection(view_graph,
                                                        cameras,
                                                        images,
                                                        tracks,
                                                        max_reprojection_error,
                                                        in_normalized_image);

  py::dict output;
  output["tracks"] = tracks;
  output["counter"] = counter;

  return output;
}

py::dict RunFilterTracksByAngle(
    const ViewGraph& view_graph,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks,
    double max_angle_error) {
  int counter = TrackFilter::FilterTracksByAngle(
      view_graph, cameras, images, tracks, max_angle_error);

  py::dict output;
  output["tracks"] = tracks;
  output["counter"] = counter;

  return output;
}

void BindTrackFilter(py::module& m) {
  m.def("filter_tracks_by_reprojection",
        &RunFilterTracksByReprojection,
        "view_graph"_a,
        "cameras"_a,
        "images"_a,
        "tracks"_a,
        "max_reprojection_error"_a = 1e-2,
        "in_normalized_image"_a = true,
        "Filter tracks by reprojection error.");

  m.def("filter_tracks_by_angle",
        &RunFilterTracksByAngle,
        "view_graph"_a,
        "cameras"_a,
        "images"_a,
        "tracks"_a,
        "max_angle_error"_a = 1.,
        "Filter tracks by angle error.");
}
