cmake_minimum_required(VERSION 3.16)
project(motionslam_task_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(task_runtime src/task_runtime.cpp src/plugin_registry.cpp)
target_include_directories(task_runtime PUBLIC include)
target_compile_options(task_runtime PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(task_runtime_tests tests/task_runtime_tests.cpp)
  target_link_libraries(task_runtime_tests PRIVATE task_runtime)
  target_compile_options(task_runtime_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME task_runtime_tests COMMAND task_runtime_tests)

  add_executable(plugin_registry_tests tests/plugin_registry_tests.cpp)
  target_link_libraries(plugin_registry_tests PRIVATE task_runtime)
  target_compile_options(plugin_registry_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME plugin_registry_tests COMMAND plugin_registry_tests)
endif()

# LNR recovery plugin bridge. Pulls legged-navigation-runtime (pinned); set
# FETCHCONTENT_SOURCE_DIR_LEGGED_NAVIGATION_RUNTIME to use a local checkout.
option(MOTIONSLAM_WITH_LNR "Build the LNR RecoveryPlugin bridge" OFF)
if(MOTIONSLAM_WITH_LNR)
  include(FetchContent)
  set(LNR_BUILD_DEMO OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(legged_navigation_runtime
    GIT_REPOSITORY https://github.com/zxinyu-robot/legged-navigation-runtime.git
    GIT_TAG 75a2fbe)
  set(_motionslam_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF)
  FetchContent_MakeAvailable(legged_navigation_runtime)
  set(BUILD_TESTING ${_motionslam_build_testing})

  add_library(recovery_bridge src/recovery_bridge.cpp)
  target_link_libraries(recovery_bridge PUBLIC task_runtime legged_navigation::runtime)
  target_compile_options(recovery_bridge PRIVATE -Wall -Wextra -Wpedantic)

  if(BUILD_TESTING)
    add_executable(recovery_bridge_tests tests/recovery_bridge_tests.cpp)
    target_link_libraries(recovery_bridge_tests PRIVATE recovery_bridge)
    target_compile_options(recovery_bridge_tests PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME recovery_bridge_tests COMMAND recovery_bridge_tests)
  endif()
endif()