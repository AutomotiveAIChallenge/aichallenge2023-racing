cmake_minimum_required(VERSION 3.5)
project(odom2tf)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(odom2tf_node
  src/odom2tf/odom2tf.cpp
)
ament_auto_add_library(odom2tf SHARED
  src/odom2tf/odom2tf.cpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_odom2tf \${test_files})
  target_link_libraries(test_odom2tf
    odom2tf
  )
endif()
ament_auto_package(INSTALL_TO_SHARE
  launch
)
