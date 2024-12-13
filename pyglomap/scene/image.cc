
#include "pyglomap/helpers.h"
#include "pyglomap/pybind11_extension.h"
#include "pyglomap/scene/types.h"
#include "pyglomap/scene/colmap_bindings.h"

#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"

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


std::shared_ptr<Image> MakeImage(image_t image_id, camera_t camera_id, std::string file_name) {
  return std::make_shared<Image>(image_id, camera_id, file_name);
}


void BindImage(py::module& m) {
  py::class_<Image, std::shared_ptr<Image>> PyImage(m, "Image");
  PyImage.def(py::init<>())
      .def(py::init(&MakeImage),
          "image_id"_a = -1,
          "camera_id"_a = -1,
          "file_name"_a = "")
      .def_property_readonly("image_id",
          [](const Image& self) -> image_t {
            return self.image_id;
          },
          "The unique identifier of the image.")
      .def_property_readonly("file_name",
          [](const Image& self) -> std::string {
            return self.file_name;
          },
          "Name of the image.")
      .def_property(
        "cam_from_world",
        [](const Image& self) -> colmap::Rigid3d {
          return self.cam_from_world;
        },
        [](Image& self, const colmap::Rigid3d& value) {
          self.cam_from_world = value;
        },
        "The pose of the image, defined as the transformation from world to "
      )
      .def_property(
        "features",
        [](const Image& self) -> std::vector<Eigen::Vector2d> {
          return self.features;
        },
        [](Image& self, const std::vector<Eigen::Vector2d>& value) {
          self.features = value;
        },
        "Distorted feature points in pixels."
      )
      .def_property(
        "features_undist",
        [](const Image& self) -> std::vector<Eigen::Vector3d> {
          return self.features_undist;
        },
        [](Image& self, const std::vector<Eigen::Vector3d>& value) {
          self.features_undist = value;
        },
        "Normalized feature rays, can be obtained by calling UndistortImages."
      )
      .def_readonly(
          "is_registered", &Image::is_registered, "Whether the image is registered.");
  // TODO: improve the printing here
  MakeDataclass(PyImage);

  py::bind_map<ImageMap>(m, "MapImageIdToImage");

  // TODO: Refactor this to a new place
  py::bind_map<CameraMap>(m, "MapCameraIdToCamera");
}
