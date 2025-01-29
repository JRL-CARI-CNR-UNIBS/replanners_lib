set(REPLANNERS_LIB_TESTS_LINK_LIBRARIES
replanners_lib::replanners_lib)

if(${GRAPH_DISPLAY_AVAILABLE})
  list(APPEND REPLANNERS_LIB_TESTS_LINK_LIBRARIES PkgConfig::graph_display)
endif()

add_executable(path_replanning_test src/path_replanning_test.cpp)
target_compile_definitions(path_replanning_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(path_replanning_test PUBLIC
    ${REPLANNERS_LIB_TESTS_LINK_LIBRARIES}
)
install(TARGETS path_replanning_test RUNTIME DESTINATION "bin/replanners_lib/tests/src")
