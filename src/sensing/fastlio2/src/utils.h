#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

class Utils
{
public:
    // static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2 &msg);
    // static sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0, int max_line = 4);
    // 将通用 PointCloud2 转换为内部 PointXYZINormal。
    // time_field 为空则不填充每点时间(curvature=0，即不做扫描内运动补偿)；
    // time_field 非空时按 time_scale 将其浮点值折算为"相对扫描起始的秒"，curvature 存毫秒。
    // filter_phase 是抽稀相位：取原始索引 i 满足 (i - filter_phase) % filter_num == 0 的点，
    // 即相位 0 与上游 FAST-LIO2 的 `i % point_filter_num == 0` 完全一致(默认值，行为不变)。
    // 相位只影响 PointCloud2 路径；livox2PCL(CustomMsg) 没有该参数(见 C2.3 说明)。
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl2_to_PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0, const std::string &time_field = "", double time_scale = 1.0, int filter_phase = 0);
    static builtin_interfaces::msg::Time getTime(const double& sec);
};