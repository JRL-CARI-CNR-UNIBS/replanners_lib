cmake_minimum_required(VERSION 3.16)
project(replanners_lib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
  # set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(graph_core REQUIRED)
find_package(cnr_class_loader REQUIRED)

set(DEPS
  graph_core::graph_core
  cnr_logger::cnr_logger
  cnr_param::cnr_param
  cnr_class_loader::cnr_class_loader
  )
set(PRIVATE_DEPS
  )

###### If ROS is available include graph_display (for debugging) ######
#TODO: evaluate only at compilation time
option(USE_GRAPH_DISPLAY "'graph_display' SUPPORT" ON)

if(USE_GRAPH_DISPLAY AND ("$ENV{ROS_VERSION}" STREQUAL "1" OR "$ENV{ROS_VERSION}" STREQUAL "2"))
 find_package(PkgConfig REQUIRED)
 pkg_check_modules(graph_display graph_display IMPORTED_TARGET)

 if(graph_display_FOUND)
   message("ROS VERSION AVAILABLE: $ENV{ROS_VERSION} -> INCLUDING 'graph_display'")
   list(APPEND PRIVATE_DEPS PkgConfig::graph_display)
   set(GRAPH_DISPLAY_AVAILABLE ON)
 else()
   message(WARNING "'graph_display' not found, setting GRAPH_DISPLAY_AVAILABLE to false")
 endif()
endif()
########################################################################

configure_file("${PROJECT_SOURCE_DIR}/include/openmore/replanners/util/replanners_lib_config.h.in"
               "${PROJECT_SOURCE_DIR}/include/openmore/replanners/config/replanners_lib_config.h" @ONLY)

add_library(${PROJECT_NAME} SHARED
  src/replanners/replanner_base.cpp
  src/replanners/MPRRT.cpp
  src/replanners/DRRTStar.cpp
  src/replanners/DRRT.cpp
  src/replanners/anytimeDRRT.cpp
  src/replanners/MARS.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC ${DEPS}
  PRIVATE ${PRIVATE_DEPS}
  )

add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Install
install(DIRECTORY include/openmore
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "share/${PROJECT_NAME}/cmake"
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
  )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

configure_file("${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in"
               "${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake" @ONLY)

install(FILES
  ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION "share/${PROJECT_NAME}/cmake")
