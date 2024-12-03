#include "glomap/controllers/global_mapper.h"

#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_io.h"
#include <glomap/colmap_migration/file.h>
#include <glomap/colmap_migration/timer.h>
#include <glomap/colmap_migration/reconstruction.h>

namespace glomap {
    // -------------------------------------
    // Mappers starting from COLMAP database
    // -------------------------------------
    int RunMapper(int argc, char** argv) {
        std::string database_path;
        std::string output_path;
        std::string image_path = "";
        std::string constraint_type = "ONLY_POINTS";
        std::string output_format = "bin";

        OptionManager options;

        // Add basic options
        options.AddRequiredOption("database_path", &database_path, "Path to database file");
        options.AddRequiredOption("output_path", &output_path, "Path to output the reconstruction");
        options.AddOption("image_path", &image_path, "Path to image directory", false,
                          std::optional<std::string>{""});
        options.AddOption("constraint_type", &constraint_type,
                          "Type of constraints to use in global positioning. "
                          "Options: {ONLY_POINTS, ONLY_CAMERAS, "
                          "POINTS_AND_CAMERAS_BALANCED, POINTS_AND_CAMERAS}",
                          false, std::optional<std::string>{"ONLY_POINTS"});
        options.AddOption("output_format", &output_format,
                          "Format for output reconstruction. Options: {bin, txt}",
                          false, std::optional<std::string>{"bin"});

        // Add all mapper options
        options.AddMapperOptions();
        options.AddViewGraphCalibrationOptions();
        options.AddRelativePoseEstimationOptions();
        options.AddRotationEstimatorOptions();
        options.AddTrackEstablishmentOptions();
        options.AddGlobalPositionerOptions();
        options.AddBundleAdjusterOptions();
        options.AddTriangulatorOptions();
        options.AddInlierThresholdOptions();

        // Parse command line
        options.Parse({argv, static_cast<size_t>(argc)});

        // Validate inputs
        if (!ExistsFile(database_path))
        {
            LOG(ERROR) << "`database_path` is not a file";
            return EXIT_FAILURE;
        }

        // Set constraint type
        if (constraint_type == "ONLY_POINTS")
        {
            options.mapper->opt_gp.constraint_type = GlobalPositionerOptions::ONLY_POINTS;
        } else if (constraint_type == "ONLY_CAMERAS")
        {
            options.mapper->opt_gp.constraint_type = GlobalPositionerOptions::ONLY_CAMERAS;
        } else if (constraint_type == "POINTS_AND_CAMERAS_BALANCED")
        {
            options.mapper->opt_gp.constraint_type = GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED;
        } else if (constraint_type == "POINTS_AND_CAMERAS")
        {
            options.mapper->opt_gp.constraint_type = GlobalPositionerOptions::POINTS_AND_CAMERAS;
        } else
        {
            LOG(ERROR) << "Invalid constraint type";
            return EXIT_FAILURE;
        }

        // Validate output format
        if (output_format != "bin" && output_format != "txt")
        {
            LOG(ERROR) << "Invalid output format";
            return EXIT_FAILURE;
        }

        // Load database
        ViewGraph view_graph;
        std::unordered_map<camera_t, Camera> cameras;
        std::unordered_map<image_t, Image> images;
        std::unordered_map<track_t, Track> tracks;

        const Database database(database_path);
        ConvertDatabaseToGlomap(database, view_graph, cameras, images);

        if (view_graph.image_pairs.empty())
        {
            LOG(ERROR) << "Can't continue without image pairs";
            return EXIT_FAILURE;
        }

        // Run mapper
        GlobalMapper global_mapper(*options.mapper);

        LOG(INFO) << "Loaded database";
        Timer run_timer;
        run_timer.Start();
        global_mapper.Solve(database, view_graph, cameras, images, tracks);
        run_timer.Pause();

        LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds() << " seconds";

        WriteGlomapReconstruction(output_path, cameras, images, tracks, output_format, image_path);
        LOG(INFO) << "Export to COLMAP reconstruction done";

        return EXIT_SUCCESS;
    }

    int RunMapperResume(int argc, char** argv) {
        std::string input_path;
        std::string output_path;
        std::string image_path = "";
        std::string output_format = "bin";

        OptionManager options;

        // Add basic options
        options.AddRequiredOption("input_path", &input_path,
                                  "Path to input COLMAP reconstruction");
        options.AddRequiredOption("output_path", &output_path,
                                  "Path to output the reconstruction");
        options.AddOption("image_path", &image_path,
                          "Path to image directory",
                          false, std::optional<std::string>{""});
        options.AddOption("output_format", &output_format,
                          "Format for output reconstruction. Options: {bin, txt}",
                          false, std::optional<std::string>{"bin"});

        // Add all mapper resume options
        options.AddMapperResumeOptions();
        options.AddGlobalPositionerOptions();
        options.AddBundleAdjusterOptions();
        options.AddTriangulatorOptions();
        options.AddInlierThresholdOptions();

        // Parse command line
        options.Parse({argv, static_cast<size_t>(argc)});

        // Validate inputs
        if (!ExistsDir(input_path))
        {
            LOG(ERROR) << "`input_path` is not a directory";
            return EXIT_FAILURE;
        }

        // Validate output format
        if (output_format != "bin" && output_format != "txt")
        {
            LOG(ERROR) << "Invalid output format";
            return EXIT_FAILURE;
        }

        // Load the reconstruction
        ViewGraph view_graph;      // dummy variable
        Database database; // dummy variable

        std::unordered_map<camera_t, Camera> cameras;
        std::unordered_map<image_t, Image> images;
        std::unordered_map<track_t, Track> tracks;

        Reconstruction reconstruction;
        reconstruction.Read(input_path);
        ConvertColmapToGlomap(reconstruction, cameras, images, tracks);

        // Run mapper
        GlobalMapper global_mapper(*options.mapper);

        Timer run_timer;
        run_timer.Start();
        global_mapper.Solve(database, view_graph, cameras, images, tracks);
        run_timer.Pause();

        LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds() << " seconds";

        WriteGlomapReconstruction(output_path, cameras, images, tracks, output_format, image_path);
        LOG(INFO) << "Export to COLMAP reconstruction done";

        return EXIT_SUCCESS;
    }

} // namespace glomap
