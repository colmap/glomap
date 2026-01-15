set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS C CXX)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    find_package(Glog REQUIRED)
endif()

if(DEFINED glog_VERSION_MAJOR)
    # Older versions of glog don't export version variables.
    add_definitions("-DGLOG_VERSION_MAJOR=${glog_VERSION_MAJOR}")
    add_definitions("-DGLOG_VERSION_MINOR=${glog_VERSION_MINOR}")
endif()

if(TESTS_ENABLED)
    message(STATUS "Enabling tests")
    find_package(GTest REQUIRED)
endif()

include(FetchContent)

# Use local PoseLib with EQUIRECTANGULAR support
FetchContent_Declare(PoseLib
    SOURCE_DIR        /home/selstad/Desktop/colmap_workspace/PoseLib
    EXCLUDE_FROM_ALL
    SYSTEM
)
message(STATUS "Configuring PoseLib (local with EQUIRECTANGULAR)...")
if (FETCH_POSELIB)
    FetchContent_MakeAvailable(PoseLib)
else()
    find_package(PoseLib REQUIRED)
endif()
message(STATUS "Configuring PoseLib... done")

# Use local COLMAP with EQUIRECTANGULAR support
FetchContent_Declare(COLMAP
    SOURCE_DIR        /home/selstad/Desktop/colmap_workspace/colmap
    EXCLUDE_FROM_ALL
)
message(STATUS "Configuring COLMAP (local with EQUIRECTANGULAR)...")
set(UNINSTALL_ENABLED OFF CACHE INTERNAL "")
set(GUI_ENABLED OFF CACHE INTERNAL "")
if (FETCH_COLMAP)
    FetchContent_MakeAvailable(COLMAP)
else()
    find_package(COLMAP REQUIRED)
endif()
message(STATUS "Configuring COLMAP... done")

set(CUDA_MIN_VERSION "7.0")
if(CUDA_ENABLED)
    if(CMAKE_VERSION VERSION_LESS 3.17)
        find_package(CUDA QUIET)
        if(CUDA_FOUND)
            message(STATUS "Found CUDA version ${CUDA_VERSION} installed in "
                    "${CUDA_TOOLKIT_ROOT_DIR} via legacy CMake (<3.17) module. "
                    "Using the legacy CMake module means that any installation of "
                    "COLMAP will require that the CUDA libraries are "
                    "available under LD_LIBRARY_PATH.")
            message(STATUS "Found CUDA ")
            message(STATUS "  Includes : ${CUDA_INCLUDE_DIRS}")
            message(STATUS "  Libraries : ${CUDA_LIBRARIES}")

            enable_language(CUDA)

            macro(declare_imported_cuda_target module)
                add_library(CUDA::${module} INTERFACE IMPORTED)
                target_include_directories(
                    CUDA::${module} INTERFACE ${CUDA_INCLUDE_DIRS})
                target_link_libraries(
                    CUDA::${module} INTERFACE ${CUDA_${module}_LIBRARY} ${ARGN})
            endmacro()

            declare_imported_cuda_target(cudart ${CUDA_LIBRARIES})
            declare_imported_cuda_target(curand ${CUDA_LIBRARIES})

            set(CUDAToolkit_VERSION "${CUDA_VERSION_STRING}")
            set(CUDAToolkit_BIN_DIR "${CUDA_TOOLKIT_ROOT_DIR}/bin")
        else()
            message(STATUS "CUDA not found")
        endif()
    else()
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            set(CUDA_FOUND ON)
            enable_language(CUDA)
        else()
            message(STATUS "CUDA not found")
        endif()
    endif()
endif()

if(CUDA_ENABLED AND CUDA_FOUND)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()

    add_definitions("-DGLOMAP_CUDA_ENABLED")

    # Do not show warnings if the architectures are deprecated.
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Wno-deprecated-gpu-targets")
    # Explicitly set PIC flags for CUDA targets.
    if(NOT IS_MSVC)
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --compiler-options -fPIC")
    endif()

    message(STATUS "Enabling CUDA support (version: ${CUDAToolkit_VERSION}, "
                    "archs: ${CMAKE_CUDA_ARCHITECTURES})")
else()
    set(CUDA_ENABLED OFF)
    message(STATUS "Disabling CUDA support")
endif()
