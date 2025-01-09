#include "pycolmap/scene/types.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

void BindImage(py::module& m);
void BindTrack(py::module& m);
void BindImagePair(py::module& m);
void BindViewGraph(py::module& m);

void BindScene(py::module& m) {
  BindImage(m);
  BindTrack(m);
  BindImagePair(m);
  BindViewGraph(m);
}
