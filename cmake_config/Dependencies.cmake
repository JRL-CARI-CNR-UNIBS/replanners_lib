# CMake policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  cmake_policy(SET CMP0148 NEW)
endif()

# Deps
file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.5/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=c46b876ae3b9f994b4f05a4c15553e0485636862064f1fcc9d8b4f832086bc5d
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME graph_core
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIBS/graph_core
  GIT_TAG master
)

message("\n-----------------------------------------------\n")

###### If ROS is available include graph_display (for debugging) ######

if(USE_GRAPH_DISPLAY AND ("$ENV{ROS_VERSION}" STREQUAL "1" OR "$ENV{ROS_VERSION}" STREQUAL "2"))
CPMFindPackage(
  NAME graph_display
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIB/graph_display
  GIT_TAG master
)

#  find_package(PkgConfig REQUIRED)
#  pkg_check_modules(graph_display graph_display IMPORTED_TARGET)

 if(graph_display_FOUND)
   message("ROS VERSION AVAILABLE: $ENV{ROS_VERSION} -> INCLUDING 'graph_display'")
   list(APPEND PRIVATE_DEPS PkgConfig::graph_display)
   set(GRAPH_DISPLAY_AVAILABLE ON)
 else()
   message(WARNING "'graph_display' not found, setting GRAPH_DISPLAY_AVAILABLE to false")
 endif()
endif()

message("\n-----------------------------------------------\n")
