set(SOURCES
    controllers/global_mapper.cc
    controllers/track_establishment.cc
    controllers/track_retriangulation.cc
    estimators/bundle_adjustment.cc
    estimators/global_positioning.cc
    estimators/global_rotation_averaging.cc
    estimators/view_graph_calibration.cc
    io/colmap_converter.cc
    io/colmap_io.cc
    math/gravity.cc
    math/rigid3d.cc
    math/two_view_geometry.cc
    processors/image_pair_inliers.cc
    processors/image_undistorter.cc
    processors/reconstruction_pruning.cc
    processors/relpose_filter.cc
    processors/track_filter.cc
    processors/view_graph_manipulation.cc
    scene/view_graph.cc
    test/prepare_experiment.cc
)


# Set HEADERS_PUBLIC variabled
set(HEADERS_PUBLIC
    controllers/global_mapper.h
    controllers/track_establishment.h
    controllers/track_retriangulation.h
    estimators/bundle_adjustment.h
    estimators/cost_function.h
    estimators/global_positioning.h
    estimators/global_rotation_averaging.h
    estimators/optimization_base.h
    estimators/view_graph_calibration.h
    io/colmap_converter.h
    io/colmap_io.h
    math/gravity.h
    math/l1_solver.h
    math/rigid3d.h
    math/two_view_geometry.h
    math/union_find.h
    processors/image_pair_inliers.h
    processors/image_undistorter.h
    processors/reconstruction_pruning.h
    processors/relpose_filter.h
    processors/track_filter.h
    processors/view_graph_manipulation.h
    scene/camera.h
    scene/image_pair.h
    scene/image.h
    scene/rigid3d.h
    scene/track.h
    scene/types_sfm.h
    scene/types.h
    test/prepare_experiment.h
)

# library configuration
include(${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake)
target_link_libraries(
    glomap
    PUBLIC
        PoseLib::PoseLib
        Eigen3::Eigen
        Ceres::ceres
        colmap::colmap
        ${SuiteSparse_LIBRARIES}
)

target_compile_features(glomap PRIVATE cxx_std_17)
