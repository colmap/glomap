// Here, we want to bind several functions
//  - Rotation averaging
//  - Global positioning
//  - Bundle adjustment

#include <pybind11/pybind11.h>

namespace py = pybind11;

void BindRotationAveraging(py::module& m);
void BindTrackEstablishment(py::module& m);
void BindGlobalPositioning(py::module& m);
void BindBundleAdjustment(py::module& m);
void BindTrackFilter(py::module& m);
void BindColmapIO(py::module& m);

void BindPipelines(py::module& m) {
    BindRotationAveraging(m);
    BindTrackEstablishment(m);
    BindGlobalPositioning(m);
    BindBundleAdjustment(m);
    BindTrackFilter(m);
    BindColmapIO(m);
}