

include(CMakeFindDependencyMacro)

find_dependency(graph_core REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/replanners_libTargets.cmake")

if(ON)
  find_dependency(graph_display REQUIRED)
endif()

set_and_check(replanners_lib_INCLUDE_DIRS "" ${graph_core_INCLUDE_DIRS})
set(replanners_lib_LIBRARIES replanners_lib ${graph_core_LIBRARIES})

check_required_components(replanners_lib)
