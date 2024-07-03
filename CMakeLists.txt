cmake_minimum_required(VERSION 3.10)

project(glomap VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)

option(OPENMP_ENABLED "Whether to enable OpenMP parallelization" ON)
option(TESTS_ENABLED "Whether to build test binaries" OFF)

set(LIBRARY_NAME   ${PROJECT_NAME})
set(LIBRARY_FOLDER ${PROJECT_NAME})
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)

include(FetchContent)
FetchContent_Declare(PoseLib
    GIT_REPOSITORY    https://github.com/PoseLib/PoseLib.git
    GIT_TAG           b3691b791bcedccd5451621b2275a1df0d9dcdeb
)
message(STATUS "Configuring PoseLib...")
FetchContent_MakeAvailable(PoseLib)
message(STATUS "Configuring PoseLib... done")

FetchContent_Declare(COLMAP
    GIT_REPOSITORY    https://github.com/colmap/colmap.git
    GIT_TAG           cfdf8da62c9af2269fd965efb33ce8147e8d5d13
)
message(STATUS "Configuring COLMAP...")
set(UNINSTALL_ENABLED OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(COLMAP)
# get_target_property(COLMAP_INCLUDE_DIRECTORIES colmap INCLUDE_DIRECTORIES)
message(STATUS "Configuring COLMAP... done")

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres 2.2 REQUIRED COMPONENTS SuiteSparse)

if(TESTS_ENABLED)
    find_package(GTest REQUIRED)
endif()


if (OPENMP_ENABLED)
    find_package(OpenMP REQUIRED)
endif()

add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)

add_subdirectory(glomap)

if(MSVC)
    target_compile_options(${LIBRARY_NAME} PRIVATE /bigobj)
else()
    target_compile_options(${LIBRARY_NAME} PRIVATE
        -Wall
        -Werror
        -Wno-sign-compare
        -Wno-unused-variable
    )
endif()

add_executable(debug glomap/test_txtinput.cc)
target_link_libraries(debug
    PUBLIC
        colmap
        PoseLib
        Eigen3::Eigen
        glomap::glomap
)
