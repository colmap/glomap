#include "glomap/controllers/track_establishment.h"

#include "glomap/scene/types_sfm.h"

#include <memory>

#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

std::unordered_map<track_t, Track> EstablishFullTracks(
    const ViewGraph& view_graph,
    const std::unordered_map<image_t, Image>& images,
    TrackEstablishmentOptions options) {
  std::unordered_map<track_t, Track> tracks;
  TrackEngine track_engine(view_graph, images, options);
  track_engine.EstablishFullTracks(tracks);
  return tracks;
}

std::unordered_map<track_t, Track> FindTracksForProblem(
    const ViewGraph& view_graph,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks_full,
    TrackEstablishmentOptions options) {
  std::unordered_map<track_t, Track> tracks_selected;
  TrackEngine track_engine(view_graph, images, options);
  track_engine.FindTracksForProblem(tracks_full, tracks_selected);

  return tracks_selected;
}

void BindTrackEstablishment(py::module& m) {
  m.def("establish_full_tracks",
        &EstablishFullTracks,
        "view_graph"_a,
        "images"_a,
        py::arg_v("options",
                  TrackEstablishmentOptions(),
                  "TrackEstablishmentOptions()"),
        "Establish full tracks from the view graph.");

  m.def("find_tracks_for_problem",
        &FindTracksForProblem,
        "view_graph"_a,
        "images"_a,
        "tracks_full"_a,
        py::arg_v("options",
                  TrackEstablishmentOptions(),
                  "TrackEstablishmentOptions()"),
        "Find tracks for the problem.");
}
