cmake_minimum_required(VERSION 3.28)

option(CUDA_ENABLED "Whether to enable CUDA, if available" ON)
option(TESTS_ENABLED "Whether to build test binaries" OFF)
option(ASAN_ENABLED "Whether to enable AddressSanitizer flags" OFF)
option(CCACHE_ENABLED "Whether to enable compiler caching, if available" ON)
option(FETCH_COLMAP "Whether to use COLMAP with FetchContent or with self-installed software" ON)
option(FETCH_POSELIB "Whether to use PoseLib with FetchContent or with self-installed software" ON)

# Propagate options to vcpkg manifest.
if(CUDA_ENABLED)
    list(APPEND VCPKG_MANIFEST_FEATURES "cuda")
endif()

# Initialize the project.
project(glomap VERSION 1.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_NO_CYCLES ON)

include(cmake/FindDependencies.cmake)

if (TESTS_ENABLED)
    enable_testing()
endif()

if(ASAN_ENABLED)
    message(STATUS "Enabling ASan")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -fsanitize-address-use-after-scope)
    add_link_options(-fsanitize=address)
endif()

if(CCACHE_ENABLED)
    find_program(CCACHE ccache)
    if(CCACHE)
        message(STATUS "Enabling ccache support")
        set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE})
        set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
    else()
        message(STATUS "Disabling ccache support")
    endif()
else()
    message(STATUS "Disabling ccache support")
endif()


if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Some fixes for the Glog library.
    add_definitions("-DGLOG_USE_GLOG_EXPORT")
    add_definitions("-DGLOG_NO_ABBREVIATED_SEVERITIES")
    add_definitions("-DGL_GLEXT_PROTOTYPES")
    add_definitions("-DNOMINMAX")
    add_compile_options(/EHsc)
    # Disable warning: 'initializing': conversion from 'X' to 'Y', possible loss of data
    add_compile_options(/wd4244 /wd4267 /wd4305)
    # Enable object level parallel builds in Visual Studio.
    add_compile_options(/MP)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
        add_compile_options(/bigobj)
    endif()
endif()

add_subdirectory(glomap)
