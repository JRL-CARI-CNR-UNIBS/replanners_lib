cmake_minimum_required(VERSION 3.16)
project(replanners_lib)

if(NOT CMAKE_BUILD_TYPE)
  #set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto -O3) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTS "Build the test executables" ON)
# If ROS is available include graph_display
option(USE_GRAPH_DISPLAY "'graph_display' SUPPORT" ON)

# Dependencies
include(cmake_config/Dependencies.cmake)

# Configure RPATH for libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_RPATH "")

message(STATUS "${BLUE}CMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}${RESET}")

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

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

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
include(GNUInstallDirs)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

#configure_file("${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in"
#               "${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake" @ONLY)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION "share/${PROJECT_NAME}/cmake")

# Clang-format target
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
    format
    COMMAND chmod +x cmake_config/clang_format_target.sh
    COMMAND cmake_config/clang_format_target.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
)
