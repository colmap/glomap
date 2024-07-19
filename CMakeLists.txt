cmake_minimum_required(VERSION 3.28)

project(glomap VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_NO_CYCLES ON)

option(OPENMP_ENABLED "Whether to enable OpenMP parallelization" ON)
option(TESTS_ENABLED "Whether to build test binaries" OFF)
option(ASAN_ENABLED "Whether to enable AddressSanitizer flags" OFF)
option(FETCH_COLMAP "Whether to use COLMAP with FetchContent or with self-installed software" ON)
option(FETCH_POSELIB "Whether to use PoseLib with FetchContent or with self-installed software" ON)

include(cmake/FindDependencies.cmake)

if(ASAN_ENABLED)
    message(STATUS "Enabling ASan")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -fsanitize-address-use-after-scope)
    add_link_options(-fsanitize=address)
endif()

add_subdirectory(glomap)
