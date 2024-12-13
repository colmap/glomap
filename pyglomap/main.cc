#include "glomap/scene/types_sfm.h"
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"

#include <ceres/version.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// using namespace glomap;
namespace py = pybind11;

void BindOptim(py::module& m);
void BindScene(py::module& m);
void BindPipelines(py::module& m);

PYBIND11_MODULE(pyglomap, m) {
    m.doc() = "GLOMAP plugin";
    // #ifdef VERSION_INFO
    //   m.attr("__version__") = py::str(VERSION_INFO);
    // #else
    // #endif
    m.attr("__version__") = py::str("dev");
    
  BindOptim(m);
  BindScene(m);
  BindPipelines(m);
}
