cmake_minimum_required(VERSION 3.5)
project(autonoma_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_DEPS std_msgs)

set(MSG_FILES
"msg/PowertrainData.msg"
"msg/RaceControl.msg"
"msg/ToRaptor.msg"
"msg/VehicleData.msg"
"msg/VehicleInputs.msg"
"msg/GroundTruth.msg"
"msg/GroundTruthArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
