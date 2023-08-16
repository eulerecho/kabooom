add_executable(boilerplate_test boilerplate_test.cpp)

add_test(NAME BoilerPlateTest COMMAND boilerplate_test)

target_link_libraries(boilerplate_test gtest gtest_main boilerplate_lib)
