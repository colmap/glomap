set(SOURCES
    colmap_migration/camera.cc
    colmap_migration/math.cc
    colmap_migration/models.cc
    colmap_migration/logging.cc
    colmap_migration/reconstruction.cc
    colmap_migration/track.cc
    colmap_migration/misc.cc
    controllers/global_mapper.cc
    controllers/option_manager.cc
    controllers/track_establishment.cc
    controllers/track_retriangulation.cc
    estimators/bundle_adjustment.cc
    estimators/global_positioning.cc
    estimators/global_rotation_averaging.cc
    estimators/gravity_refinement.cc
    estimators/relpose_estimation.cc
    estimators/view_graph_calibration.cc
    io/colmap_converter.cc
    io/colmap_io.cc
    io/gravity_io.cc
    math/gravity.cc
    math/rigid3d.cc
    math/tree.cc
    math/two_view_geometry.cc
    processors/image_pair_inliers.cc
    processors/image_undistorter.cc
    processors/reconstruction_normalizer.cc
    processors/reconstruction_pruning.cc
    processors/relpose_filter.cc
    processors/track_filter.cc
    processors/view_graph_manipulation.cc
    scene/view_graph.cc
)

set(HEADERS
    colmap_migration/eigen_alignment.h
    colmap_migration/types.h
    colmap_migration/camera.h
    colmap_migration/logging.h
    colmap_migration/math.h
    colmap_migration/models.h
    colmap_migration/reconstruction.h
    colmap_migration/track.h
    colmap_migration/enum_to_string.h
    colmap_migration/misc.h
    controllers/global_mapper.h
    controllers/option_manager.h
    controllers/track_establishment.h
    controllers/track_retriangulation.h
    estimators/bundle_adjustment.h
    estimators/cost_function.h
    estimators/global_positioning.h
    estimators/global_rotation_averaging.h
    estimators/gravity_refinement.h
    estimators/relpose_estimation.h
    estimators/optimization_base.h
    estimators/view_graph_calibration.h
    io/colmap_converter.h
    io/colmap_io.h
    io/gravity_io.h
    math/gravity.h
    math/l1_solver.h
    math/rigid3d.h
    math/tree.h
    math/two_view_geometry.h
    math/union_find.h
    processors/image_pair_inliers.h
    processors/image_undistorter.h
    processors/reconstruction_normalizer.h
    processors/reconstruction_pruning.h
    processors/relpose_filter.h
    processors/track_filter.h
    processors/view_graph_manipulation.h
    scene/camera.h
    scene/image_pair.h
    scene/image.h
    scene/track.h
    scene/types_sfm.h
    scene/types.h
)

add_library(glomap ${SOURCES} ${HEADERS})
if(NOT FETCH_COLMAP)
    target_link_libraries(glomap PUBLIC colmap::colmap)
else()
    target_link_libraries(glomap PUBLIC colmap)
endif()

if(NOT FETCH_POSELIB)
    target_link_libraries(glomap PUBLIC PoseLib::PoseLib)
else()
    target_link_libraries(glomap PUBLIC PoseLib)
endif()

set(LIBTORCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/libtorch")
set(CMAKE_PREFIX_PATH ${LIBTORCH_DIR})
find_package(Torch REQUIRED)

target_compile_options(glomap PRIVATE -Wno-restrict)
target_link_libraries(
    glomap
    PUBLIC
        Eigen3::Eigen
        Ceres::ceres
        SuiteSparse::CHOLMOD
        ${TORCH_LIBRARIES}
        ${BOOST_LIBRARIES}
)
target_include_directories(glomap PUBLIC ..)

if(MSVC)
    target_compile_options(glomap PRIVATE /bigobj)
else()
    target_compile_options(glomap PRIVATE
        -Wall
        -Werror
        -Wno-sign-compare
        -Wno-unused-variable
    )
endif()


add_executable(glomap_main
    glomap.cc
    exe/global_mapper.h
    exe/global_mapper.cc)
target_link_libraries(glomap_main glomap)

set_target_properties(glomap_main PROPERTIES OUTPUT_NAME glomap)
install(TARGETS glomap_main DESTINATION bin)


if(TESTS_ENABLED)
    add_executable(glomap_test
        controllers/global_mapper_test.cc
    )
    target_link_libraries(
        glomap_test
        PRIVATE
            glomap
            GTest::gtest
            GTest::gtest_main)
    add_test(NAME glomap_test COMMAND glomap_test)
endif()
