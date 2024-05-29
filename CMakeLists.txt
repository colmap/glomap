# cmake version to be used
cmake_minimum_required(VERSION 3.10)

# project name
project(glomap VERSION 0.0.1)
# set(CMAKE_FIND_LIBRARY_SUFFIXES .a .so)
set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)
# add_compile_definitions(EIGEN_DONT_VECTORIZE)

set(EIGEN_DISABLE_ALIGN_FLAGS
"-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16 -DEIGEN_DONT_VECTORIZE")
set (AVX2_CXX_FLAGS "-mavx2 -mf16c -mfma")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AVX2_CXX_FLAGS} ${EIGEN_DISABLE_ALIGN_FLAGS}")

# Set variables
set(LIBRARY_NAME   ${PROJECT_NAME})
set(LIBRARY_FOLDER ${PROJECT_NAME})
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)


# external libs
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 3.4 REQUIRED)
find_package(PoseLib REQUIRED)
find_package(colmap REQUIRED)

find_package(SuiteSparse QUIET)

find_package(Theia REQUIRED)




# Include directories for SuiteSparse
include_directories(${SuiteSparse_INCLUDE_DIRS})


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OpenMP
option(USE_OpenMP                    "Use OpenMP"         ON)
if (USE_OpenMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()


include_directories(${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})

# Library sources
add_subdirectory(glomap)

# # python bindings
# add_subdirectory(pybind)


# Compilation options
if(MSVC)
	target_compile_options(${LIBRARY_NAME} PRIVATE /bigobj /fp:fast)
else()
	target_compile_options(${LIBRARY_NAME} PRIVATE
		-march=native -Wall -Werror -Wno-sign-compare
        # (-ffast-math is not good for nan)
	 	# -Wno-unused-variable -ffast-math -fPIC) 
	 	-Wno-unused-variable -fPIC)
endif()

# target

add_executable( debug glomap/test_txtinput.cc )

target_link_libraries(debug
  PUBLIC
    ${CERES_LIBRARIES}
  PUBLIC
    colmap::colmap
  PUBLIC 
    PoseLib::PoseLib
  PUBLIC Eigen3::Eigen
  PUBLIC glomap::glomap
  PUBLIC ${SuiteSparse_LIBRARIES}
  )