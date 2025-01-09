
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace pybind11::literals;
namespace py = pybind11;

// Casting between Eigen::Quaterniond and pycolmap.Rotation3d
namespace pybind11 {
namespace detail {

template <>
struct type_caster<Eigen::Quaterniond> {
  PYBIND11_TYPE_CASTER(Eigen::Quaterniond, _("Rotation3d"));

  // Python to C++ (load)
  bool load(handle src, bool) {
    if (!src) return false;
    try {
      py::module pycolmap = py::module::import("pycolmap");
      py::object rotation_class = pycolmap.attr("Rotation3d");
      if (!py::isinstance(src, rotation_class)) return false;

      py::object quat_py = src.attr("quat");
      value.coeffs() = quat_py.cast<Eigen::Vector4d>();

      return true;
    } catch (const py::error_already_set& e) {
      return false;
    }
  }

  // C++ to Python (cast)
  static handle cast(const Eigen::Quaterniond& src,
                     return_value_policy,
                     handle) {
    try {
      py::module pycolmap = py::module::import("pycolmap");
      py::object rotation_class = pycolmap.attr("Rotation3d");
      py::object rotation_instance = rotation_class();
      rotation_instance.attr("quat") = py::cast(src.coeffs());
      return rotation_instance.release();
    } catch (const py::error_already_set& e) {
      return nullptr;
    }
  }
};

// Casting between colmap::Rigid3d and pycolmap.Rigid3d
template <>
struct type_caster<colmap::Rigid3d> {
  // set things up and gives you a `colmap::Rigid3d value;` member
  PYBIND11_TYPE_CASTER(colmap::Rigid3d, _("Rigid3d"));

  bool load(handle src, bool) {
    if (!src) return false;
    py::module pycolmap = py::module::import("pycolmap");
    py::object rigid3d_class = pycolmap.attr("Rigid3d");
    py::object rotation_class = pycolmap.attr("Rotation3d");

    if (!py::isinstance(src, rigid3d_class)) return false;

    try {
      py::object rotation_py = src.attr("rotation");
      py::object translation_py = src.attr("translation");

      value.translation = translation_py.cast<Eigen::Vector3d>();
      value.rotation = rotation_py.cast<Eigen::Quaterniond>();
    } catch (const py::cast_error&) {
      return false;
    }
    return true;
  }

  static handle cast(const colmap::Rigid3d& v,
                     return_value_policy /*policy*/,
                     handle /*parent*/) {
    try {
      py::module pycolmap = py::module::import("pycolmap");
      py::object rigid3d_class = pycolmap.attr("Rigid3d");
      return rigid3d_class(v.rotation, v.translation).release();
    } catch (const py::error_already_set& e) {
      return nullptr;
    }
  }
};

// Casting between colmap::CameraModelId and pycolmap.CameraModelId
template <>
struct type_caster<colmap::CameraModelId> {
  // set things up and gives you a `CameraModelId value;` member
  PYBIND11_TYPE_CASTER(colmap::CameraModelId, _("CameraModelId"));

  bool load(handle src, bool) {
    if (!src) return false;
    py::module pycolmap = py::module::import("pycolmap");
    py::object camera_model_id_class = pycolmap.attr("CameraModelId");

    if (!py::isinstance(src, camera_model_id_class)) return false;
    try {
      value = colmap::CameraModelNameToId(src.attr("name").cast<std::string>());

    } catch (const py::cast_error&) {
      return false;
    }
    return true;
  }

  static handle cast(colmap::CameraModelId v,
                     return_value_policy /*policy*/,
                     handle /*parent*/) {
    try {
      py::module pycolmap = py::module::import("pycolmap");
      py::object camera_model_id_class = pycolmap.attr("CameraModelId");
      return camera_model_id_class(colmap::CameraModelIdToName(v)).release();
    } catch (const py::error_already_set& e) {
      return nullptr;
    }
  }
};

// Casting between colmap::Camera and pycolmap.Camera
template <>
struct type_caster<glomap::Camera> {
  // set things up and gives you a `Camera value;` member
  PYBIND11_TYPE_CASTER(glomap::Camera, _("Camera"));

  bool load(handle src, bool) {
    if (!src) return false;
    py::module pycolmap = py::module::import("pycolmap");
    py::object camera_class = pycolmap.attr("Camera");

    if (!py::isinstance(src, camera_class)) return false;

    try {
      value.camera_id = src.attr("camera_id").cast<colmap::camera_t>();
      value.model_id = src.attr("model").cast<colmap::CameraModelId>();
      value.width = src.attr("width").cast<size_t>();
      value.height = src.attr("height").cast<size_t>();

      value.params = src.attr("params").cast<std::vector<double>>();
      value.has_prior_focal_length =
          src.attr("has_prior_focal_length").cast<bool>();
    } catch (const py::cast_error&) {
      return false;
    }
    return true;
  }

  static handle cast(const glomap::Camera& v,
                     return_value_policy /*policy*/,
                     handle /*parent*/) {
    try {
      py::module pycolmap = py::module::import("pycolmap");
      py::object camera_class = pycolmap.attr("Camera");

      py::dict d;
      d["camera_id"] = v.camera_id;
      d["model"] = v.ModelName();
      d["width"] = v.width;
      d["height"] = v.height;
      d["params"] = v.params;
      d["has_prior_focal_length"] = v.has_prior_focal_length;

      // use the "create" function in camera_class to create a new camera object
      return camera_class(d).release();

    } catch (const py::error_already_set& e) {
      return nullptr;
    }
  }
};

}  // namespace detail
}  // namespace pybind11
