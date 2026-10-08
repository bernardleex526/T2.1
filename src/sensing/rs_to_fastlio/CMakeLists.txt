cmake_minimum_required(VERSION 3.8)
project(rs_to_fastlio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(rs_to_fastlio_node src/rs_to_fastlio_node.cpp)
target_include_directories(rs_to_fastlio_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rs_to_fastlio_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  rs_to_fastlio_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS rs_to_fastlio_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
