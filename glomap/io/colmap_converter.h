#pragma once
#include "glomap/scene/types_sfm.h"
#include <glomap/colmap_migration/database.h>
#include <glomap/colmap_migration/image.h>
#include <glomap/colmap_migration/reconstruction.h>

namespace glomap {

    void ConvertGlomapToColmapImage(const Image& image,
                                    Image& colmap_image,
                                    bool keep_points = false);

    void ConvertGlomapToColmap(const std::unordered_map<camera_t, Camera>& cameras,
                               const std::unordered_map<image_t, Image>& images,
                               const std::unordered_map<track_t, Track>& tracks,
                               Reconstruction& reconstruction,
                               int cluster_id = -1,
                               bool include_image_points = false);

    void ConvertColmapToGlomap(const Reconstruction& reconstruction,
                               std::unordered_map<camera_t, Camera>& cameras,
                               std::unordered_map<image_t, Image>& images,
                               std::unordered_map<track_t, Track>& tracks);

    void ConvertColmapPoints3DToGlomapTracks(
        const Reconstruction& reconstruction,
        std::unordered_map<track_t, Track>& tracks);

    void ConvertDatabaseToGlomap(const Database& database,
                                 ViewGraph& view_graph,
                                 std::unordered_map<camera_t, Camera>& cameras,
                                 std::unordered_map<image_t, Image>& images);

} // namespace glomap
