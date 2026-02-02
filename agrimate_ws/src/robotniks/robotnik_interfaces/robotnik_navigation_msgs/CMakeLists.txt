cmake_minimum_required(VERSION 3.5)
project(robotnik_navigation_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotnik_common_msgs REQUIRED)

set(msg_files
)

set(srv_files
)

set(action_files
  "action/Dock.action"
  "action/Move.action"
  "action/Charge.action"
  "action/Uncharge.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    action_msgs
    geometry_msgs
    robotnik_common_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
