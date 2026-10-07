cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

add_definitions(-DMP_EN)
# OpenMP 线程数可由外部覆盖(迁移到核心数不同的开发板时按需传入 -DMP_PROC_NUM=4),
# 默认 2 以匹配原硬编码值。
set(MP_PROC_NUM 2 CACHE STRING "ikd-Tree/IESKF 并行处理线程数")
add_definitions(-DMP_PROC_NUM=${MP_PROC_NUM})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(interface REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


# Sophus include path: the ROS/apt Sophus package (ros-humble-sophus) ships an
# INTERFACE IMPORTED target `Sophus::Sophus` and does NOT define the legacy
# `Sophus_INCLUDE_DIRS` variable, so the plain variable below expands to nothing
# and `#include <sophus/so3.hpp>` fails with "No such file or directory" even
# though the header is installed in /opt/ros/humble/include.  Prefer the target
# when it exists and fall back to the variable for source builds of Sophus
# (which is what upstream FAST-LIO documents).
set(T21_SOPHUS_INCLUDE_DIRS "${Sophus_INCLUDE_DIRS}")
if(TARGET Sophus::Sophus)
  get_target_property(T21_SOPHUS_TARGET_INCLUDES Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
  if(T21_SOPHUS_TARGET_INCLUDES)
    list(APPEND T21_SOPHUS_INCLUDE_DIRS ${T21_SOPHUS_TARGET_INCLUDES})
  endif()
endif()
if(NOT T21_SOPHUS_INCLUDE_DIRS)
  message(FATAL_ERROR
    "Sophus headers not found: find_package(Sophus) succeeded but exposed neither "
    "Sophus::Sophus nor Sophus_INCLUDE_DIRS. Install ros-humble-sophus or a Sophus "
    "source build.")
endif()
list(REMOVE_DUPLICATES T21_SOPHUS_INCLUDE_DIRS)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${T21_SOPHUS_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


set(SRC_LIST src/map_builder/ieskf.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/commons.cpp
             src/utils.cpp
             src/map_builder/pinhole_camera.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/image_processor.cpp
             src/map_builder/map_builder.cpp
            )

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})
ament_target_dependencies(lio_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs cv_bridge interface)
target_link_libraries(lio_node
  yaml-cpp
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})




if(BUILD_TESTING)
  # Deterministic tests of the static-window IMU init decision (map_builder/imu_init.h):
  # window selection, the max-wait fallback that must fire for a vibrating non-rotating
  # platform, and the window-scoped accel deviation used by the gravity adoption.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_init test/test_imu_init.cpp)
  target_include_directories(test_imu_init PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
                                                   ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_imu_init pcl_conversions)
  target_link_libraries(test_imu_init ${PCL_LIBRARIES})

  # Behavioural regression of the LiDAR frontend converters (src/utils.cpp): the livox2PCL
  # tag/line/range/stride selection and the pcl2_to_PCL decimation phase (C2.3, opt-in
  # `pcl2_filter_phase`) with the per-point time origin of each retained set.  Only the returned
  # clouds are inspected, never source text.
  ament_add_gtest(test_utils_preprocess test/test_utils_preprocess.cpp src/utils.cpp)
  target_include_directories(test_utils_preprocess PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
                                                          ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_utils_preprocess pcl_conversions sensor_msgs livox_ros_driver2)
  target_link_libraries(test_utils_preprocess ${PCL_LIBRARIES})

  # Behavioural regression of the quadruped gait notch cascade
  # (map_builder/gait_filter.h): pass-through when inert, real attenuation at
  # the configured centres, per-axis masking that does not leak filter state
  # between axes, unity DC gain, and config validation.  Header-only and free of
  # ROS/PCL, so this test needs Eigen and gtest only.
  ament_add_gtest(test_gait_filter test/test_gait_filter.cpp)
  target_include_directories(test_gait_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
                                                     ${EIGEN3_INCLUDE_DIRS})

  # Wiring check for the gait compensation inside IMUProcessor: with the feature
  # disabled (the shipped profile) the IMU path must be unchanged, and an enabled
  # but unusable configuration must stay inert instead of building a broken
  # filter.  Needs the map_builder sources, so it links the same set as the node.
  ament_add_gtest(test_gait_integration test/test_gait_integration.cpp
                  src/map_builder/ieskf.cpp
                  src/map_builder/commons.cpp
                  src/map_builder/imu_processor.cpp)
  target_include_directories(test_gait_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
                                                          ${EIGEN3_INCLUDE_DIRS}
                                                          ${PCL_INCLUDE_DIRS}
                                                          ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_gait_integration rclcpp pcl_conversions sensor_msgs)
  # SyncPackage holds a cv::Mat, so the test needs OpenCV linked as well as PCL.
  target_link_libraries(test_gait_integration ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
