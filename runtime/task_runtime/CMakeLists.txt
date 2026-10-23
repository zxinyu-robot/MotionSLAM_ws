cmake_minimum_required(VERSION 3.16)
project(motionslam_task_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(task_runtime src/task_runtime.cpp)
target_include_directories(task_runtime PUBLIC include)
target_compile_options(task_runtime PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(task_runtime_tests tests/task_runtime_tests.cpp)
  target_link_libraries(task_runtime_tests PRIVATE task_runtime)
  target_compile_options(task_runtime_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME task_runtime_tests COMMAND task_runtime_tests)
endif()