

include(CMakeFindDependencyMacro)

find_dependency(graph_core REQUIRED)
find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)
find_dependency(cnr_class_loader REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/replanners_libTargets.cmake")

if(ON)
  find_dependency(graph_display REQUIRED)
endif()

set_and_check(replanners_lib_INCLUDE_DIRS "" ${graph_core_INCLUDE_DIRS} ${cnr_logger_INCLUDE_DIRS} ${cnr_param_INCLUDE_DIRS} ${cnr_class_loader_INCLUDE_DIRS})
set(replanners_lib_LIBRARIES replanners_lib ${graph_core_LIBRARIES} ${cnr_logger_LIBRARIES} ${cnr_param_LIBRARIES} ${cnr_class_loader_LIBRARIES})

check_required_components(replanners_lib)
