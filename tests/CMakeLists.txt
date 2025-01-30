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

#GTests
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(run_all_tests src/run_all_tests.cpp)
target_link_libraries(run_all_tests PRIVATE ${GTEST_LIBRARIES} pthread ${REPLANNERS_LIB_TESTS_LINK_LIBRARIES})
install(TARGETS run_all_tests RUNTIME DESTINATION "bin/replanners_lib/tests/src")

# Install files
install(FILES logger_param.yaml logger_param_gtest.yaml
    DESTINATION "bin/replanners_lib/tests")
