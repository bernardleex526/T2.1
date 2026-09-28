cmake_minimum_required(VERSION 3.8)
project(lidar_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖（补全所有需要的PCL组件）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# 显式声明所有用到的PCL组件：common+filters+segmentation+search
find_package(PCL REQUIRED COMPONENTS 
  common 
  filters 
  segmentation 
  search
  
)

# 包含PCL头文件目录（解决头文件找不到的终极方案）
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(preprocessor src/preprocessor.cpp)
ament_target_dependencies(preprocessor
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)
# 链接所有PCL库
target_link_libraries(preprocessor
  ${PCL_LIBRARIES}
)

install(TARGETS
  preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
