# Set PROJECT_NAME_UPPERCASE and PROJECT_NAME_LOWERCASE variables
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWERCASE)

# Library name (by default is the project name)
if(NOT LIBRARY_NAME)
  set(LIBRARY_NAME ${PROJECT_NAME_LOWERCASE})
endif()

# Library folder name (by default is the project name in lowercase)
# Example: #include <foo/foo.h>
if(NOT LIBRARY_FOLDER)
  set(LIBRARY_FOLDER ${PROJECT_NAME_LOWERCASE})
endif()

# Make sure different configurations don't collide
set(CMAKE_DEBUG_POSTFIX "d")

# Select library type (default: STATIC)
option(BUILD_SHARED_LIBS "Build ${LIBRARY_NAME} as a shared library." OFF)
message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")

# Build type (default: RELEASE)
#
# No reason to set CMAKE_CONFIGURATION_TYPES if it's not a multiconfig generator
# Also no reason of using CMAKE_BUILD_TYPE if it's a multiconfig generator.
#
get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(isMultiConfig)
  set(CMAKE_CONFIGURATION_TYPES
      "Release;Debug;MinSizeRel;RelWithDebInfo" CACHE STRING "" FORCE)

  message(STATUS "CMAKE_CONFIGURATION_TYPES: ${CMAKE_CONFIGURATION_TYPES}")
  message(STATUS "CMAKE_GENERATOR: Multi-config")
else()
  # Set a default build type if none was specified
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  endif()

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
               "Release" "Debug" "MinSizeRel" "RelWithDebInfo")

  message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
  message(STATUS "CMAKE_GENERATOR: Single-config")
endif()
message(STATUS "CMAKE_GENERATOR: ${CMAKE_GENERATOR}")

# Generated headers folder
set(GENERATED_HEADERS_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated_headers"
)

# Create 'version.h'
configure_file(
  "${PROJECT_SOURCE_DIR}/${LIBRARY_FOLDER}/version.h.in"
  "${GENERATED_HEADERS_DIR}/${LIBRARY_FOLDER}/version.h"
  @ONLY
)

# Introduce variables:
#   - CMAKE_INSTALL_LIBDIR
#   - CMAKE_INSTALL_BINDIR
#   - CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

# Layout. This works for all platforms:
#   - <prefix>/lib*/cmake/<PROJECT-NAME>
#   - <prefix>/lib*/
#   - <prefix>/include/
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

# Configuration
set(GENERATED_DIR       "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG_FILE "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG_FILE "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")

# Include module with functions:
#   - write_basic_package_version_file(...)
#   - configure_package_config_file(...)
include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Use:
#   - PROJECT_VERSION
write_basic_package_version_file(
    "${VERSION_CONFIG_FILE}"
    VERSION "${${PROJECT_NAME}_VERSION}"
    COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Use variables:
#   - TARGETS_EXPORT_NAME
#   - PROJECT_NAME
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${PROJECT_CONFIG_FILE}"
      INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Uninstall targets
configure_file("${PROJECT_SOURCE_DIR}/cmake/Uninstall.cmake.in"
  "${GENERATED_DIR}/Uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${GENERATED_DIR}/Uninstall.cmake)

# Always full RPATH (for shared libraries)
#  https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
if(BUILD_SHARED_LIBS)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH FALSE)

  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # the RPATH to be used when installing, but only if it's not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
      set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif()
endif()

# CMake Registry
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeRegistry.cmake)
