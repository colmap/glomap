#pragma once

#include "glomap/controllers/global_mapper.h"

namespace glomap {

// Use default values for most of the settings from database
int RunMapper(int argc, char** argv);

// Use default values for most of the settings from colmap reconstruction
int RunMapperResume(int argc, char** argv);

// Only run the relative pose estimation for later uses
int RunRelativePoseEstimator(int argc, char** argv);
}  // namespace glomap