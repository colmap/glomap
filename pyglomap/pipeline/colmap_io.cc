#include "glomap/io/colmap_io.h"

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

void WriteGlomapReconstructionWrapper(
    const std::string& reconstruction_path,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks,
    const std::string output_format,
    const std::string image_path) {
  WriteGlomapReconstruction(
      reconstruction_path, cameras, images, tracks, output_format, image_path);
}

void BindColmapIO(py::module& m) {
  m.def("write_glomap_reconstruction",
        &WriteGlomapReconstructionWrapper,
        "reconstruction_path"_a,
        "cameras"_a,
        "images"_a,
        "tracks"_a,
        "output_format"_a = "bin",
        "image_path"_a = "",
        "Write the reconstruction to the disk.");
}
