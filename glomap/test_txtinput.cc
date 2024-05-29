#include "glomap/io/colmap_io.h"
#include "glomap/controllers/global_mapper.h"

#include "glomap/types.h"

#include <colmap/util/timer.h>

#include <string> 

using namespace glomap;

int main (int argc, char** argv) {
    std::string database_dir, output_dir;

    database_dir = argv[1];
    output_dir = argv[2];

    // Load the database
    ViewGraph view_graph;
    std::unordered_map<camera_t, Camera> cameras;
    std::unordered_map<image_t, Image> images;
    std::unordered_map<track_t, Track> tracks;

    ConvertDatabaseToGlomap(database_dir, view_graph, cameras, images);
    std::cout << "Loaded database" << std::endl;

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

    options.opt_triangulator.database_dir = database_dir;


    colmap::Timer run_timer;
    run_timer.Start();

    GlobalMapper global_mapper(options);
    global_mapper.Solve(view_graph, cameras, images, tracks);

    run_timer.Pause();

    std::cout << "Recontruction done in " << run_timer.ElapsedSeconds() << " seconds" << std::endl;

    WriteGlomapReconstruction(output_dir, cameras, images, tracks);

    std::cout << "Exported to COLMAP reconstruction done" << std::endl;



    return 0;
};