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
    // curvature 以本帧保留点中最早的时间为零点(点不一定按时间排序)。
    // scan_t0_sec 非空时写入该最早点的时间(按 time_scale 折算后的秒；无有效时间字段时写 NaN)，
    // 调用方据此判断帧头时间戳是扫描起点还是终点(RoboSense SDK 默认 ts_first_point=false)。
    // NaN / Inf 坐标的点被丢弃(驱动 dense_points=false 时无效点为 NaN)。
    // 字段按其 datatype 用 memcpy 读取(不要求对齐)；x/y/z 必须是 FLOAT32 或 FLOAT64，且每个字段
    // 必须落在 point_step 内，否则返回空云；data 比 width*height*point_step 短时只解析完整的点。
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl2_to_PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0, const std::string &time_field = "", double time_scale = 1.0, int filter_phase = 0, double *scan_t0_sec = nullptr);
    static builtin_interfaces::msg::Time getTime(const double& sec);

    // 扫描起始时刻(秒)：pcl2_to_PCL 把 curvature 以最早保留点 t0 为零点，所以起始时刻必须对应 t0。
    //   t0 非有限(无时间字段)              -> 帧头时间
    //   |t0 - 帧头| < 1 s (绝对时间, 如 RoboSense timestamp) -> t0
    //   |t0| < 1 s (相对帧头的偏移, 如 Ouster t / Velodyne time) -> 帧头 + t0
    //   其它(时钟域不一致)                  -> 帧头, *inconsistent = true
    static double scanStartTime(double header_sec, double point_t0, bool *inconsistent = nullptr);
};