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
    GIT_TAG           b5c381ad71e6a970266a1f1280de523c0b10f107
)
message(STATUS "Configuring COLMAP...")
set(UNINSTALL_ENABLED OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(COLMAP)
message(STATUS "Configuring COLMAP... done")

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres 2.2 REQUIRED COMPONENTS SuiteSparse)

if(TESTS_ENABLED)
    message(STATUS "Enabling tests")
    find_package(GTest REQUIRED)
endif()

if (OPENMP_ENABLED)
    message(STATUS "Enabling OpenMP")
    find_package(OpenMP REQUIRED)
endif()
