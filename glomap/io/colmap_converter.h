#ifndef GLOMAP_IO_COLMAP_CONVERTER_H_
#define GLOMAP_IO_COLMAP_CONVERTER_H_
#include <colmap/scene/reconstruction.h>

#include "glomap/scene/types_sfm.h"

namespace glomap {

void ConvertGlomapToColmap(const std::unordered_map<camera_t, Camera>& cameras,
                        const std::unordered_map<image_t, Image>& images,
                        const std::unordered_map<track_t, Track>& tracks,
                        colmap::Reconstruction& reconstruction,
                        bool include_image_points = false);

void ConvertColmapToGlomap(const colmap::Reconstruction& reconstruction,
                        std::unordered_map<camera_t, Camera>& cameras,
                        std::unordered_map<image_t, Image>& images,
                        std::unordered_map<track_t, Track>& tracks);

void ConvertColmapPoints3DToGlomapTracks(
                        const colmap::Reconstruction& reconstruction,
                        std::unordered_map<track_t, Track>& tracks);

void ConvertDatabaseToGlomap(const std::string& database_path,
                        ViewGraph& view_graph,
                        std::unordered_map<camera_t, Camera>& cameras,
                        std::unordered_map<image_t, Image>& images);

}  // namespace glomap


#endif  // GLOMAP_IO_COLMAP_CONVERTER_H_