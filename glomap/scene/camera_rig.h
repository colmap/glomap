#pragma once

#include "glomap/scene/types.h"
#include "glomap/types.h"
// #include "glomap/scene/types_sfm.h"
#include "glomap/scene/camera.h"
#include "glomap/scene/image.h"
// #include ""
// #include "glomap/scene/view_graph.h"

#include <colmap/scene/camera_rig.h>

namespace glomap {

struct CameraRig : public colmap::CameraRig {
  CameraRig() : colmap::CameraRig() {}
  CameraRig(const colmap::CameraRig& camera_rig)
      : colmap::CameraRig(camera_rig) {}

  // double ComputeRigFromWorldScale(const std::unordered_map<image_t, Image>&
  // images) const;

  // bool ComputeCamsFromRigs(const std::unordered_map<image_t, Image>& images);

  Rigid3d ComputeRigFromWorld(
      size_t snapshot_idx,
      const std::unordered_map<image_t, Image>& images) const;
};
}  // namespace glomap
