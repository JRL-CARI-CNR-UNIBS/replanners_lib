include(CMakeFindDependencyMacro)

find_dependency(graph_core REQUIRED)
find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)
find_dependency(cnr_class_loader REQUIRED)

if(USE_GRAPH_DISPLAY)
    find_dependency(graph_display REQUIRED)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/replanners_libTargets.cmake")
