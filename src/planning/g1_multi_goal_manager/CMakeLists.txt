cmake_minimum_required(VERSION 3.5)
project(g1_multi_goal_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息和服务文件
set(msg_files
  "msg/GoalInfo.msg"
)

set(srv_files
  "srv/ClearGoals.srv"
  "srv/ListGoals.srv"
  "srv/DeleteGoal.srv"
  "srv/UpdateGoalName.srv"
  "srv/StopNavigation.srv"
  "srv/GetCurrentLocation.srv"
)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# 安装 Python 可执行脚本（仅安装脚本，不将 scripts 作为包）
install(PROGRAMS
  scripts/g1_multi_goal_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
