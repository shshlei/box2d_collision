cmake_minimum_required(VERSION 3.8)
project(box2d_collision VERSION 2.4.1)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

option(BUILD_SHARED_LIBS "Build Box2D_Collision as a shared library" ON)
option(USE_DOUBLE_PRECISION "Build Box2D_Collision in double precision" ON)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

include(GNUInstallDirs)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if (USE_DOUBLE_PRECISION)
	add_compile_definitions(B2_USE_DOUBLE_PRECISION)
endif()

add_subdirectory(src)
add_subdirectory(demos)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/box2d_collision"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
