#include "glomap/controllers/track_retriangulation.h"

#include "glomap/io/colmap_converter.h"

#include <colmap/controllers/incremental_pipeline.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/database_cache.h>

#include <set>

namespace glomap {

bool RetriangulateTracks(const TriangulatorOptions& options,
                         const colmap::Database& database,
                         std::unordered_map<camera_t, Camera>& cameras,
                         std::unordered_map<image_t, Image>& images,
                         std::unordered_map<track_t, Track>& tracks) {
  // Following code adapted from COLMAP
  auto database_cache =
      colmap::DatabaseCache::Create(database,
                                    options.min_num_matches,
                                    false,  // ignore_watermarks
                                    {}      // reconstruct all possible images
      );

  // Check whether the image is in the database cache. If not, set the image
  // as not registered to avoid memory error.
  std::vector<image_t> image_ids_notconnected;
  for (auto& image : images) {
    if (!database_cache->ExistsImage(image.first) &&
        image.second.is_registered) {
      image.second.is_registered = false;
      image_ids_notconnected.push_back(image.first);
    }
  }

  // Convert the glomap data structures to colmap data structures
  std::shared_ptr<colmap::Reconstruction> reconstruction_ptr =
      std::make_shared<colmap::Reconstruction>();
  ConvertGlomapToColmap(cameras,
                        images,
                        std::unordered_map<track_t, Track>(),
                        *reconstruction_ptr);

  colmap::IncrementalPipelineOptions options_colmap;
  options_colmap.triangulation.complete_max_reproj_error =
      options.tri_complete_max_reproj_error;
  options_colmap.triangulation.merge_max_reproj_error =
      options.tri_merge_max_reproj_error;
  options_colmap.triangulation.min_angle = options.tri_min_angle;

  reconstruction_ptr->DeleteAllPoints2DAndPoints3D();
  reconstruction_ptr->TranscribeImageIdsToDatabase(database);

  colmap::IncrementalMapper mapper(database_cache);
  mapper.BeginReconstruction(reconstruction_ptr);

  // Triangulate all images.
  const auto tri_options = options_colmap.Triangulation();
  const auto mapper_options = options_colmap.Mapper();

  const std::set<image_t>& reg_image_ids = reconstruction_ptr->RegImageIds();

  size_t image_idx = 0;
  for (const image_t image_id : reg_image_ids) {
    std::cout << "\r Triangulating image " << image_idx++ + 1 << " / "
              << reg_image_ids.size() << std::flush;

    const auto& image = reconstruction_ptr->Image(image_id);

    int num_tris = mapper.TriangulateImage(tri_options, image_id);
  }
  std::cout << std::endl;

  // Merge and complete tracks.
  mapper.CompleteAndMergeTracks(tri_options);

  auto ba_options = options_colmap.GlobalBundleAdjustment();
  ba_options.refine_focal_length = false;
  ba_options.refine_principal_point = false;
  ba_options.refine_extra_params = false;
  ba_options.refine_extrinsics = false;

  // Configure bundle adjustment.
  colmap::BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  colmap::ObservationManager observation_manager(*reconstruction_ptr);

  for (int i = 0; i < options_colmap.ba_global_max_refinements; ++i) {
    std::cout << "\r Global bundle adjustment iteration " << i + 1 << " / "
              << options_colmap.ba_global_max_refinements << std::flush;
    // Avoid degeneracies in bundle adjustment.
    observation_manager.FilterObservationsWithNegativeDepth();

    const size_t num_observations =
        reconstruction_ptr->ComputeNumObservations();

    // PrintHeading1("Bundle adjustment");
    colmap::BundleAdjuster bundle_adjuster(ba_options, ba_config);
    // THROW_CHECK(bundle_adjuster.Solve(reconstruction.get()));
    if (!bundle_adjuster.Solve(reconstruction_ptr.get())) {
      return false;
    }

    size_t num_changed_observations = 0;
    num_changed_observations += mapper.CompleteAndMergeTracks(tri_options);
    num_changed_observations += mapper.FilterPoints(mapper_options);
    const double changed =
        static_cast<double>(num_changed_observations) / num_observations;
    if (changed < options_colmap.ba_global_max_refinement_change) {
      break;
    }
  }
  std::cout << std::endl;

  // Add the removed images to the reconstruction
  for (const auto& image_id : image_ids_notconnected) {
    images[image_id].is_registered = true;
    colmap::Image image_colmap;
    ConvertGlomapToColmapImage(images[image_id], image_colmap, true);
    reconstruction_ptr->AddImage(std::move(image_colmap));
  }

  // Convert the colmap data structures back to glomap data structures
  ConvertColmapToGlomap(*reconstruction_ptr, cameras, images, tracks);

  return true;
}

}  // namespace glomap
