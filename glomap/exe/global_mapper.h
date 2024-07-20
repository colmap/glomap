#pragma once

#include "glomap/controllers/global_mapper.h"

namespace glomap {

// Use default values for most of the settings from database
int RunMapper(int argc, char** argv);

// Have more control over parameters
int RunCustomizedMapper(int argc, char** argv);

// Use default values for most of the settings from colmap reconstruction
int RunMapperResume(int argc, char** argv);

// Have more control over parameters
int RunCustomizedMapperResume(int argc, char** argv);

}  // namespace glomap