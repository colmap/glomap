cmake_minimum_required(VERSION 3.10)

project(glomap VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)

# Set variables
set(LIBRARY_NAME   ${PROJECT_NAME})
set(LIBRARY_FOLDER ${PROJECT_NAME})
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)


# external libs
find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres 2.2 REQUIRED COMPONENTS SuiteSparse)
find_package(PoseLib REQUIRED)
find_package(colmap 3.10 REQUIRED)
find_package(SuiteSparse REQUIRED)



option(USE_OpenMP "Use OpenMP" ON)
if (USE_OpenMP)
    find_package(OpenMP REQUIRED)
endif()

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

add_executable( debug glomap/test_txtinput.cc )
target_link_libraries(debug
  PUBLIC
    colmap::colmap
    PoseLib::PoseLib
    Eigen3::Eigen
    glomap::glomap
)
