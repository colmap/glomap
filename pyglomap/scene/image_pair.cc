
#include "glomap/scene/types_sfm.h"

#include <memory>
#include <optional>
#include <sstream>

#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/colmap_bindings.h"
#include "pyglomap/scene/types.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace glomap;
using namespace pybind11::literals;
namespace py = pybind11;

std::shared_ptr<ImagePair> MakeImagePair(image_t image_id1,
                                         image_t image_id2,
                                         Rigid3d pose_rel) {
  return std::make_shared<ImagePair>(image_id1, image_id2, pose_rel);
}

void BindImagePair(py::module& m) {
  py::class_<ImagePair, std::shared_ptr<ImagePair>> PyImagePair(m, "ImagePair");
  PyImagePair.def(py::init<>())
      .def(py::init(&MakeImagePair),
           "image_id1"_a = -1,
           "image_id2"_a = -1,
           "pose_rel"_a = Rigid3d())
      .def_property_readonly(
          "image_id1",
          [](const ImagePair& self) -> image_t { return self.image_id1; },
          "The unique identifier of the image 1.")
      .def_property_readonly(
          "image_id2",
          [](const ImagePair& self) -> image_t { return self.image_id2; },
          "The unique identifier of the image 2.")
      .def_property_readonly(
          "pair_id",
          [](const ImagePair& self) -> image_pair_t { return self.pair_id; },
          "The unique identifier of the image pair.")
      .def_readwrite(
          "is_valid", &ImagePair::is_valid, "Whether the image pair is valid.")
      .def_readwrite("weight", &ImagePair::weight, "The initial inlier rate.")
      .def_property(
          "cam2_from_cam1",
          [](const ImagePair& self) -> Rigid3d { return self.cam2_from_cam1; },
          [](ImagePair& self, const Rigid3d& value) {
            self.cam2_from_cam1 = value;
          },
          "The relative pose from camera 1 to camera 2.")
      .def_readwrite(
          "matches", &ImagePair::matches, "Matches between the two images.")
      .def_readwrite("inliers",
                     &ImagePair::inliers,
                     "Row index of inliers in the matches matrix.")
      .def("__repr__", CreateRepresentation<ImagePair>)
      .def("summary", [](const ImagePair& self) {
        std::ostringstream ss;
        ss << "ImagePair: " << self.image_id1 << " - " << self.image_id2
           << std::endl;
        ss << "  pair_id: " << self.pair_id << std::endl;
        ss << "  is_valid: " << self.is_valid << std::endl;
        ss << "  weight: " << self.weight << std::endl;
        ss << "  cam2_from_cam1: " << self.cam2_from_cam1 << std::endl;
        ss << "  num_matches: " << self.matches.rows() << std::endl;
        ss << "  num_inliers: " << self.inliers.size() << std::endl;
        return ss.str();
      });
  MakeDataclass(PyImagePair);

  py::bind_map<ImagePairMap>(m, "MapImagePairIdToImagePair");
}
