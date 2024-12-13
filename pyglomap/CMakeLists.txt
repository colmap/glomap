# setup PYTHON_EXECUTABLE

find_package (Python COMPONENTS Interpreter Development)
message(STATUS "Python_EXECUTABLE: " ${Python_EXECUTABLE})

add_subdirectory(pybind11)
find_package(pybind11 CONFIG REQUIRED)

find_package(colmap REQUIRED)

file(GLOB_RECURSE SOURCE_FILES "*.cc")


pybind11_add_module(pyglomap MODULE ${SOURCE_FILES})
message(STATUS "${SOURCE_FILES}")

target_link_libraries(pyglomap PRIVATE glomap pybind11::module colmap::colmap Eigen3::Eigen ${CERES_LIBRARIES})
target_compile_features(pyglomap PRIVATE cxx_std_17)
