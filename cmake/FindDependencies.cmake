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

set(CUDA_MIN_VERSION "7.0")
if(CUDA_ENABLED)
    find_package(CUDAToolkit QUIET)
    if(CUDAToolkit_FOUND)
        set(CUDA_FOUND ON)
    else()
        message(STATUS "CUDA not found")
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
