#include "glomap/controllers/global_mapper.h"
#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_io.h"
#include "glomap/types.h"

#include <colmap/util/misc.h>
#include <colmap/util/timer.h>


namespace glomap {
// -------------------------------------
// Mappers starting from COLMAP database
// -------------------------------------
int RunMapperImpl(int argc, char** argv, bool customized=false) {
  std::string database_path;
  std::string output_path;

  std::string constraint_type = "ONLY_POINTS";

  OptionManager options;
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("constraint_type", &constraint_type,
    "{ONLY_POINTS, ONLY_CAMERAS, POINTS_AND_CAMERAS_BALANCED, POINTS_AND_CAMERAS}");
  if (!customized)
    options.AddGlobalMapperOptions();
  else
    options.AddGlobalMapperFullOptions();

  options.Parse(argc, argv);

  if (!colmap::ExistsFile(database_path)) {
    std::cout << "`database_path` is not a file" << std::endl;
    return EXIT_FAILURE;
  }

  if (constraint_type == "ONLY_POINTS") {
    options.mapper->opt_gp.constraint_type
      = GlobalPositionerOptions::ONLY_POINTS;
  } else if (constraint_type == "ONLY_CAMERAS") {
    options.mapper->opt_gp.constraint_type
      = GlobalPositionerOptions::ONLY_CAMERAS;
  } else if (constraint_type == "POINTS_AND_CAMERAS_BALANCED") {
    options.mapper->opt_gp.constraint_type
      = GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED;
  } else if (constraint_type == "POINTS_AND_CAMERAS") {
    options.mapper->opt_gp.constraint_type
      = GlobalPositionerOptions::POINTS_AND_CAMERAS;
  } else {
    std::cout << "Invalid constriant Type type";
    return EXIT_FAILURE;
  }

  // Load the database
  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  const colmap::Database database(database_path);
  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  GlobalMapper global_mapper(*options.mapper);

  // Main solver
  std::cout << "Loaded database" << std::endl;
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(database, view_graph, cameras, images, tracks);
  run_timer.Pause();

  std::cout << "Recontruction done in " << run_timer.ElapsedSeconds()
            << " seconds" << std::endl;

  WriteGlomapReconstruction(output_path, cameras, images, tracks);
  std::cout << "Exported to COLMAP reconstruction done" << std::endl;


  return EXIT_SUCCESS;
}

int RunMapper(int argc, char** argv) {
  return RunMapperImpl(argc, argv, false);
}

int RunCustomizedMapper(int argc, char** argv) {
  return RunMapperImpl(argc, argv, true);
}

// -------------------------------------
// Mappers starting from COLMAP reconstruction
// -------------------------------------
int RunMapperResumeImp(int argc, char** argv, bool customized) {
  std::string input_path;
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  if (!customized)
    options.AddGlobalMapperResumeOptions();
  else
    options.AddGlobalMapperResumeFullOptions();

  options.Parse(argc, argv);

  if (!colmap::ExistsDir(input_path)) {
    std::cout << "`input_path` is not a directory" << std::endl;
    return EXIT_FAILURE;
  }

  // Load the reconstruction
  ViewGraph view_graph; // dummy variable
  colmap::Database database; // dummy variable

  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;
  colmap::Reconstruction reconstruction;
  reconstruction.Read(input_path);
  ConvertColmapToGlomap(reconstruction, cameras, images, tracks);


  GlobalMapper global_mapper(*options.mapper);

  // Main solver
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(database, view_graph, cameras, images, tracks);
  run_timer.Pause();

  std::cout << "Recontruction done in " << run_timer.ElapsedSeconds()
            << " seconds" << std::endl;

  WriteGlomapReconstruction(output_path, cameras, images, tracks);
  std::cout << "Exported to COLMAP reconstruction done" << std::endl;


  return EXIT_SUCCESS;
}
int RunMapperResume(int argc, char** argv) {
  return RunMapperResumeImp(argc, argv, false);
}

// Have more control over parameters
int RunCustomizedMapperResume(int argc, char** argv) {
  return RunMapperResumeImp(argc, argv, true);
}

} // glomap