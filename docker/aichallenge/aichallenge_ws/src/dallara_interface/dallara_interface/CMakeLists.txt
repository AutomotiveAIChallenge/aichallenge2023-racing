cmake_minimum_required(VERSION 3.5)
project(dallara_interface)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(dallara_interface_node
  src/dallara_interface.cpp
)
ament_auto_add_library(dallara_interface SHARED
  src/dallara_interface.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
