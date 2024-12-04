set(SOURCES
    colmap_migration/camera.cc
    colmap_migration/math.cc
    colmap_migration/models.cc
    colmap_migration/logging.cc
    colmap_migration/reconstruction.cc
    colmap_migration/track.cc
    colmap_migration/misc.cc
    colmap_migration/sim3.cc
    colmap_migration/point2d.cc
    colmap_migration/point3d.cc
    colmap_migration/image.cc
    colmap_migration/rigid3.cc
    colmap_migration/gps.cc
    colmap_migration/database.cc
    colmap_migration/visibility_pyramid.cc
    colmap_migration/two_view_geometry.cc
    colmap_migration/feature_types.cc
    colmap_migration/string.cc
    colmap_migration/pose.cc
    colmap_migration/triangulation.cc
    colmap_migration/essential_matrix.cc
    colmap_migration/two_view_geometry_estimator.cc
    colmap_migration/random_sampler.cc
    colmap_migration/random.cc
    colmap_migration/support_measurement.cc
    colmap_migration/fundamental_matrix.cc
    colmap_migration/homography_matrix.cc
    colmap_migration/homography_matrix_estimator.cc
    colmap_migration/fundamental_matrix_estimator.cc
    colmap_migration/utils_estimator.cc
    colmap_migration/polynomial.cc
    colmap_migration/bitmap.cc
    colmap_migration/correspondence_graph.cc
    colmap_migration/database_cache.cc
    colmap_migration/file.cc
    colmap_migration/ply.cc
    colmap_migration/projection.cc
    colmap_migration/reconstruction_io.cc
    colmap_migration/endian.cc
    colmap_migration/specs.cc
    colmap_migration/database_sensor.cc
    colmap_migration/timer.cc
    colmap_migration/threading.cc
    colmap_migration/bundle_adjustment.cc
    colmap_migration/camera_rig.cc
    colmap_migration/incremental_mapper.cc
    colmap_migration/incremental_triangulator.cc
    colmap_migration/observation_manager.cc
    colmap_migration/alignment.cc
    colmap_migration/triangulation_estimator.cc
    colmap_migration/combination_sampler.cc
    colmap_migration/incremental_pipeline.cc
    colmap_migration/base_controller.cc
    colmap_migration/reconstruction_manager.cc
    colmap_migration/testing.cc
    colmap_migration/synthetic.cc
    colmap_migration/pose_estimator.cc
    colmap_migration/absolute_pose.cc
    colmap_migration/utils.cc
    colmap_migration/essential_matrix_estimator.cc
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
    colmap_migration/sim3.h
    colmap_migration/point2d.h
    colmap_migration/point3d.h
    colmap_migration/image.h
    colmap_migration/rigid3.h
    colmap_migration/gps.h
    colmap_migration/database.h
    colmap_migration/visibility_pyramid.h
    colmap_migration/two_view_geometry.h
    colmap_migration/feature_types.h
    colmap_migration/sqlite3_utils.h
    colmap_migration/string.h
    colmap_migration/pose.h
    colmap_migration/triangulation.h
    colmap_migration/essential_matrix.h
    colmap_migration/matrix.h
    colmap_migration/two_view_geometry_estimator.h
    colmap_migration/sampler.h
    colmap_migration/random_sampler.h
    colmap_migration/random.h
    colmap_migration/ransac.h
    colmap_migration/support_measurement.h
    colmap_migration/fundamental_matrix.h
    colmap_migration/fundamental_matrix_estimator.h
    colmap_migration/homography_matrix.h
    colmap_migration/homography_matrix_estimator.h
    colmap_migration/translation_transform.h
    colmap_migration/loransac.h
    colmap_migration/utils_estimator.h
    colmap_migration/polynomial.h
    colmap_migration/bitmap.h
    colmap_migration/correspondence_graph.h
    colmap_migration/database_cache.h
    colmap_migration/file.h
    colmap_migration/ply.h
    colmap_migration/projection.h
    colmap_migration/reconstruction_io.h
    colmap_migration/endian.h
    colmap_migration/specs.h
    colmap_migration/database_sensor.h
    colmap_migration/timer.h
    colmap_migration/threading.h
    colmap_migration/bundle_adjustment.h
    colmap_migration/camera_rig.h
    colmap_migration/incremental_mapper.h
    colmap_migration/incremental_triangulator.h
    colmap_migration/observation_manager.h
    colmap_migration/alignment.h
    colmap_migration/triangulation_estimator.h
    colmap_migration/combination_sampler.h
    colmap_migration/cost_functions.h
    colmap_migration/manifold.h
    colmap_migration/similarity_transform.h
    colmap_migration/incremental_pipeline.h
    colmap_migration/base_controller.h
    colmap_migration/reconstruction_manager.h
    colmap_migration/testing.h
    colmap_migration/synthetic.h
    colmap_migration/pose_estimator.h
    colmap_migration/absolute_pose.h
    colmap_migration/utils.h
    colmap_migration/essential_matrix_estimator.h
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
    scene/image_pair.h
    scene/image.h
    scene/track.h
    scene/types_sfm.h
    scene/types.h
)

add_library(glomap ${SOURCES} ${HEADERS})
#if(NOT FETCH_COLMAP)
#    target_link_libraries(glomap PUBLIC colmap::colmap)
#else()
#    target_link_libraries(glomap PUBLIC colmap)
#endif()

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
