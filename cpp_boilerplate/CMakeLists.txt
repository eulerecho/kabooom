cmake_minimum_required(VERSION 3.10)

project(howdy_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify where the header files are
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add subdirectory for the library
add_subdirectory(lib)

# Add main application executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link the library to the main application
target_link_libraries(${PROJECT_NAME} boilerplate_lib)

# Disable Google Test's build scripts
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(BUILD_GTEST ON CACHE BOOL "" FORCE)

# Specify the Google Test directory
add_subdirectory(googletest)

enable_testing()

# Add subdirectory for the tests
add_subdirectory(tests)
