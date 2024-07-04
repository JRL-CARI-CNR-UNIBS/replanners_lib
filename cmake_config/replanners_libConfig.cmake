include(CMakeFindDependencyMacro)

find_dependency(graph_core REQUIRED)
find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)
find_dependency(cnr_class_loader REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/replanners_libTargets.cmake")

if(ON)
  find_dependency(graph_display REQUIRED)
endif()

set(replanners_lib_LIBRARIES replanners_lib)
set(replanners_lib_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(replanners_lib_FOUND True)
