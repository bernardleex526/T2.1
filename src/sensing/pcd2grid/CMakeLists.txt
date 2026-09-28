cmake_minimum_required(VERSION 3.8)
project(pcd2grid)  # 新包名，与功能包名一致

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找核心依赖（无冲突）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters io common)

# 包含头文件
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 编译节点（可执行文件名改为pcd2grid_node）
add_executable(pcd2grid_node src/pcd2grid.cpp)
ament_target_dependencies(pcd2grid_node
  rclcpp nav_msgs pcl_conversions sensor_msgs
)
# 链接PCL库（关键）
target_link_libraries(pcd2grid_node ${PCL_LIBRARIES})

# 安装可执行文件
install(TARGETS
  pcd2grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
