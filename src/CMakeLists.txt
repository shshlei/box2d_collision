set(BOX2D_SOURCE_FILES
    common/b2_logging.cpp
    common/b2_math.cpp
    common/b2_block_allocator.cpp
    common/b2_stack_allocator.cpp
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

set(BOX2D_HEADER_FILES
    ../include/box2d_collision/b2_api.h
    ../include/box2d_collision/b2_logging.h
    ../include/box2d_collision/b2_scalar.h
    ../include/box2d_collision/b2_common.h
    ../include/box2d_collision/b2_math.h
    ../include/box2d_collision/b2_block_allocator.h
    ../include/box2d_collision/b2_stack_allocator.h
    ../include/box2d_collision/b2_shape.h
    ../include/box2d_collision/b2_shape_circle.h
    ../include/box2d_collision/b2_shape_ellipse.h
    ../include/box2d_collision/b2_shape_capsule.h
    ../include/box2d_collision/b2_shape_rectangle.h
    ../include/box2d_collision/b2_shape_polygon.h
    ../include/box2d_collision/b2_fixture.h
    ../include/box2d_collision/b2_body.h
    ../include/box2d_collision/b2_dynamic_tree.h
    ../include/box2d_collision/b2_broad_phase.h
    ../include/box2d_collision/b2_bvh_manager.h
    ../include/box2d_collision/b2_callbacks.h
    ../include/box2d_collision/b2_collision.h
    ../include/box2d_collision/b2_distance.h)

add_library(box2d_collision ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d_collision
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

set_target_properties(box2d_collision PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    )

if(MSVC)
    set_target_properties(box2d_collision PROPERTIES
        COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        COMPILE_PDB_NAME box2d_collision
        )

    install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/box2d_collision.pdb"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        OPTIONAL
        )
endif()

if (BUILD_SHARED_LIBS)
    target_compile_definitions(box2d_collision
        PUBLIC
        B2_SHARED
        )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_HEADER_FILES})

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
