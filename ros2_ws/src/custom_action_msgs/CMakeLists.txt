cmake_minimum_required(VERSION 3.5)

project(custom_action_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PROJECT_DEPENDENCIES
  geometry_msgs
  builtin_interfaces
  std_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(ACTION_FILES
  "action/PointAction.action"
  "action/Homebase.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)

ament_package()
