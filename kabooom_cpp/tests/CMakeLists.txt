add_executable(pathfinder_tests pathfinder_tests.cpp)

add_test(NAME PathFinderTest COMMAND pathfinder_tests)

target_link_libraries(pathfinder_tests gtest gtest_main libpathfinder)
