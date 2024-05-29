# Target
add_library(${LIBRARY_NAME}
  ${SOURCES}
  ${HEADERS_PUBLIC}
  ${HEADERS_PRIVATE}
  )

# Alias:
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# C++11
target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)

# Add definitions for targets
# Values:
#   - Debug  : -DPOSELIB_DEBUG=1
#   - Release: -DPOSELIB_DEBUG=0
#   - others : -DPOSELIB_DEBUG=0
target_compile_definitions(${LIBRARY_NAME} PUBLIC
  "${PROJECT_NAME_UPPERCASE}_DEBUG=$<CONFIG:Debug>")

# Global includes. Used by all targets
# Note:
#   - header can be included by C++ code `#include <PoseLib/poselib.h`
#   - header location in project: ${CMAKE_CURRENT_BINARY_DIR}/generated_headers
target_include_directories(
  ${LIBRARY_NAME} PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${GENERATED_HEADERS_DIR}>"
    "$<INSTALL_INTERFACE:.>"
)

# Targets:
#   - <prefix>/lib/libPoseLib.a
#   - header location after install: <prefix>/PoseLib/poselib.h
#   - headers can be included by C++ code `#include <PoseLib/poselib.h>`
install(
    TARGETS              "${LIBRARY_NAME}"
    EXPORT               "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION  "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)


# Headers:
#   - PoseLib/*.h -> <prefix>/include/PoseLib/*.h
foreach ( file ${HEADERS_PUBLIC} )
    get_filename_component( dir ${file} DIRECTORY )
    install( FILES ${file} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_FOLDER}/${dir}" )
endforeach()
#install( FILES "${GENERATED_HEADERS_DIR}/PoseLib/poselib.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_FOLDER}")

# Headers:
#   - generated_headers/PoseLib/version.h -> <prefix>/include/PoseLib/version.h
install(
    FILES       "${GENERATED_HEADERS_DIR}/${LIBRARY_FOLDER}/version.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_FOLDER}"
)

# Config
#   - <prefix>/lib/cmake/PoseLib/PoseLibConfig.cmake
#   - <prefix>/lib/cmake/PoseLib/PoseLibConfigVersion.cmake
install(
    FILES       "${PROJECT_CONFIG_FILE}"
                "${VERSION_CONFIG_FILE}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Config
#   - <prefix>/lib/cmake/PoseLib/PoseLibTargets.cmake
install(
  EXPORT      "${TARGETS_EXPORT_NAME}"
  FILE        "${PROJECT_NAME}Targets.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}"
  NAMESPACE   "${PROJECT_NAME}::"
)
