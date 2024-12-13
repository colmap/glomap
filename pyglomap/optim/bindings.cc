#include <pybind11/pybind11.h>

namespace py = pybind11;

void BindRotationAveragerOptions(py::module& m);
void BindTrackEstablishmentOptions(py::module& m);
void BindGlobalPositionerOptions(py::module& m);
void BindBundleAdjusterOptions(py::module& m);


void BindOptim(py::module& m) {
  BindRotationAveragerOptions(m);
  BindTrackEstablishmentOptions(m);
  BindGlobalPositionerOptions(m);
  BindBundleAdjusterOptions(m);
}

