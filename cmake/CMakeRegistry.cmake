#
# CMake Registry
#
#   Export package to CMake registry such that it can be easily found by
#   CMake even if it has not been installed to a standard directory.
#
#   Note: this feature is disabled by default. Possible options:
#     -DCMAKE_REGISTRY_FOLDER="OFF"           (disable CMake registry [default])
#     -DCMAKE_REGISTRY_FOLDER="INSTALL_FOLDER"
#     -DCMAKE_REGISTRY_FOLDER="BUILD_FOLDER"
#
if( NOT (DEFINED CMAKE_REGISTRY_FOLDER) )
  set(CMAKE_REGISTRY_FOLDER "OFF"
    CACHE STRING "Choose CMake registry folder." FORCE)
endif()

# Set possible values for cmake-gui
set_property(CACHE CMAKE_REGISTRY_FOLDER PROPERTY STRINGS
  "BUILD_FOLDER" "INSTALL_FOLDER" "OFF")
message(STATUS "CMAKE_REGISTRY_FOLDER: ${CMAKE_REGISTRY_FOLDER}")


# CMake Register (build directory)
if(CMAKE_REGISTRY_FOLDER STREQUAL "BUILD_FOLDER")
  export(PACKAGE ${PROJECT_NAME})
endif()

#
# Register installed package with CMake
#
# This function adds an entry to the CMake registry for packages with the
# path of the directory where the package configuration file of the installed
# package is located in order to help CMake find the package in a custom
# installation prefix. This differs from CMake's export(PACKAGE) command
# which registers the build directory instead.
#
function (register_package PACKAGE_FOLDER)
  if (NOT IS_ABSOLUTE "${PACKAGE_FOLDER}")
    set (PACKAGE_FOLDER "${CMAKE_INSTALL_PREFIX}/${PACKAGE_FOLDER}")
  endif ()

  string (MD5 REGISTRY_ENTRY "${PACKAGE_FOLDER}")

  if (WIN32)
    install (CODE
      "execute_process (
         COMMAND reg add \"HKCU\\\\Software\\\\Kitware\\\\CMake\\\\Packages\\\\${PROJECT_NAME}\" /v \"${REGISTRY_ENTRY}\" /d \"${PACKAGE_FOLDER}\" /t REG_SZ /f
         RESULT_VARIABLE RT
         ERROR_VARIABLE  ERR
         OUTPUT_QUIET
       )
       if (RT EQUAL 0)
         message (STATUS \"Register:   Added HKEY_CURRENT_USER\\\\Software\\\\Kitware\\\\CMake\\\\Packages\\\\${PROJECT_NAME}\\\\${REGISTRY_ENTRY}\")
       else ()
         string (STRIP \"\${ERR}\" ERR)
         message (STATUS \"Register:   Failed to add registry entry: \${ERR}\")
       endif ()"
    )
  elseif (IS_DIRECTORY "$ENV{HOME}")
    file (WRITE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-registry-entry" "${PACKAGE_FOLDER}")
    install (
      FILES       "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-registry-entry"
      DESTINATION "$ENV{HOME}/.cmake/packages/${PROJECT_NAME}"
      RENAME      "${REGISTRY_ENTRY}"
    )
    message (STATUS "CMake registry: $ENV{HOME}/.cmake/packages/${PROJECT_NAME}")
  endif ()
endfunction ()

# CMake Register (install directory)
if(CMAKE_REGISTRY_FOLDER STREQUAL "INSTALL_FOLDER")
  register_package(${CONFIG_INSTALL_DIR})
endif ()
