set(BOX2D_SOURCE_FILES
    common/b2_logging.cpp
    common/b2_block_allocator.cpp
    collision/b2_obb.cpp
    collision/b2_shape_circle.cpp
    collision/b2_shape_ellipse.cpp
    collision/b2_shape_capsule.cpp
    collision/b2_shape_rectangle.cpp
    collision/b2_shape_polygon.cpp
    collision/b2_fixture.cpp
    collision/b2_body.cpp
    collision/b2_dynamic_tree.cpp
    collision/b2_broad_phase.cpp
    collision/b2_bvh_manager.cpp
    collision/b2_callbacks.cpp
    collision/b2_collide_circle.cpp
    collision/b2_collide_capsule.cpp
    collision/b2_collide_polygon.cpp
    collision/b2_collision.cpp
    collision/b2_distance.cpp)

add_library(box2d_collision ${BOX2D_SOURCE_FILES})
target_include_directories(box2d_collision
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

if (BUILD_SHARED_LIBS)
    target_compile_definitions(box2d_collision PUBLIC B2_SHARED)
endif()

install(
    TARGETS box2d_collision
    EXPORT box2dConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

install(
    EXPORT box2dConfig
    NAMESPACE box2d_collision::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d_collision"
    )

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
    )

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d_collision"
    )
