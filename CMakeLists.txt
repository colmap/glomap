cmake_minimum_required(VERSION 3.10)

project(glomap VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)

option(OPENMP_ENABLED "Whether to enable OpenMP parallelization" ON)
option(TESTS_ENABLED "Whether to build test binaries" OFF)

set(LIBRARY_NAME   ${PROJECT_NAME})
set(LIBRARY_FOLDER ${PROJECT_NAME})
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres 2.2 REQUIRED COMPONENTS SuiteSparse)
find_package(PoseLib REQUIRED)
find_package(colmap 3.10 REQUIRED)
find_package(SuiteSparse REQUIRED)

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
    colmap::colmap
    PoseLib::PoseLib
    Eigen3::Eigen
    glomap::glomap
)
