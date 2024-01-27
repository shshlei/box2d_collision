cmake_minimum_required(VERSION 3.2)
project(box2d_collision VERSION 3.0.0)

add_definitions(-std=c++11)

option(BUILD_SHARED_LIBS "Build Box2D_Collision as a shared library" ON)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)
add_subdirectory(demos)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/box2d_collision"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
