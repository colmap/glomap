// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "glomap/controllers/global_mapper.h"

#include "glomap/io/colmap_io.h"
#include "glomap/types.h"

#include <colmap/estimators/alignment.h>
#include <colmap/scene/synthetic.h>
#include <colmap/util/testing.h>

#include <gtest/gtest.h>

namespace glomap {
namespace {

void ExpectEqualReconstructions(const colmap::Reconstruction& gt,
                                const colmap::Reconstruction& computed,
                                const double max_rotation_error_deg,
                                const double max_proj_center_error,
                                const double num_obs_tolerance) {
  EXPECT_EQ(computed.NumCameras(), gt.NumCameras());
  EXPECT_EQ(computed.NumImages(), gt.NumImages());
  EXPECT_EQ(computed.NumRegImages(), gt.NumRegImages());
  EXPECT_GE(computed.ComputeNumObservations(),
            (1 - num_obs_tolerance) * gt.ComputeNumObservations());

  colmap::Sim3d gtFromComputed;
  colmap::AlignReconstructionsViaProjCenters(computed,
                                             gt,
                                             /*max_proj_center_error=*/0.1,
                                             &gtFromComputed);

  const std::vector<colmap::ImageAlignmentError> errors =
      colmap::ComputeImageAlignmentError(computed, gt, gtFromComputed);
  EXPECT_EQ(errors.size(), gt.NumImages());
  for (const auto& error : errors) {
    EXPECT_LT(error.rotation_error_deg, max_rotation_error_deg);
    EXPECT_LT(error.proj_center_error, max_proj_center_error);
  }
}

TEST(IncrementalMapperController, WithoutNoise) {
  const std::string database_path = colmap::CreateTestDir() + "/database.db";

  colmap::Database database(database_path);
  colmap::Reconstruction gt_reconstruction;
  colmap::SyntheticDatasetOptions synthetic_dataset_options;
  synthetic_dataset_options.num_cameras = 2;
  synthetic_dataset_options.num_images = 7;
  synthetic_dataset_options.num_points3D = 50;
  synthetic_dataset_options.point2D_stddev = 0;
  colmap::SynthesizeDataset(
      synthetic_dataset_options, &gt_reconstruction, &database);

  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  GlobalMapperOptions options;
  // Control the verbosity of the global sfm
  options.opt_vgcalib.verbose = false;
  options.opt_ra.verbose = false;
  options.opt_gp.verbose = false;
  options.opt_ba.verbose = false;

  // Control the flow of the global sfm
  options.skip_view_graph_calibration = false;
  options.skip_relative_pose_estimation = false;
  options.skip_rotation_averaging = false;
  options.skip_track_establishment = false;
  options.skip_global_positioning = false;
  options.skip_bundle_adjustment = false;
  options.skip_retriangulation = false;

  GlobalMapper global_mapper(options);
  global_mapper.Solve(database, view_graph, cameras, images, tracks);

  colmap::Reconstruction reconstruction;
  ConvertGlomapToColmap(cameras, images, tracks, reconstruction);

  ExpectEqualReconstructions(gt_reconstruction,
                             reconstruction,
                             /*max_rotation_error_deg=*/1e-2,
                             /*max_proj_center_error=*/1e-4,
                             /*num_obs_tolerance=*/0);
}

}  // namespace
}  // namespace glomap
