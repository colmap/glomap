
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/types.h"
#include "pyglomap/scene/colmap_bindings.h"

#include "glomap/scene/types_sfm.h"

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


void BindTrack(py::module& m) {
  py::class_<Track, std::shared_ptr<Track>> PyTrack(m, "Track");
  PyTrack.def(py::init<>())
      .def_property_readonly("track_id",
          [](const Track& self) -> track_t {
            return self.track_id;
          },
          "The unique identifier of the image.")
			.def_readwrite("xyz",
				&Track::xyz,
				"The 3D point.")
			.def_readwrite("color",
				&Track::color,
				"The color of the track.")
      .def_property(
        "observations",
        [](const Track& self) -> std::vector<Observation> {
					return self.observations;
        },
        [](Track& self, const std::vector<Observation> & value) {
					self.observations = value;
        },
        "Observations of the track."
      )
      .def_readonly(
          "is_initialized", &Track::is_initialized, "Whether the point is initialized.")
			.def("__repr__", [](const Track& self) {
				std::ostringstream ss;
				ss << "Track(" << self.track_id << ", num_observations=" << self.observations.size() << ")";
				return ss.str();
			});
  MakeDataclass(PyTrack);

  py::bind_map<TrackMap>(m, "MapTrackIdToTrack");
}
