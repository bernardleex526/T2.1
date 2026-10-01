#include "utils.h"
#include <sensor_msgs/msg/point_field.hpp>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range, int max_line)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < max_line) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pcl2_to_PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range, const std::string &time_field, double time_scale, int filter_phase)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 按字段名定位字节偏移
    auto find_offset = [&](const std::string &name) -> int
    {
        for (const auto &f : msg->fields)
            if (f.name == name)
                return static_cast<int>(f.offset);
        return -1;
    };
    int off_x = find_offset("x");
    int off_y = find_offset("y");
    int off_z = find_offset("z");
    if (off_x < 0 || off_y < 0 || off_z < 0)
        return cloud; // 不含 xyz 字段，无法转换
    int off_int = find_offset("intensity");
    // 时间字段需按其 datatype 解析(FLOAT32/FLOAT64)，否则禁用每点时间
    int off_time = -1, time_datatype = 0;
    if (!time_field.empty() && time_scale > 0.0)
    {
        for (const auto &f : msg->fields)
        {
            if (f.name == time_field)
            {
                off_time = static_cast<int>(f.offset);
                time_datatype = static_cast<int>(f.datatype);
                break;
            }
        }
        if (time_datatype != sensor_msgs::msg::PointField::FLOAT32 &&
            time_datatype != sensor_msgs::msg::PointField::FLOAT64)
            off_time = -1; // 仅支持浮点时间字段(如 Velodyne time / RoboSense timestamp)
    }

    std::size_t point_step = msg->point_step;
    std::size_t point_num = msg->width * msg->height;
    const int stride_i = std::max(1, filter_num);
    const std::size_t stride = static_cast<std::size_t>(stride_i);
    // Sampling phase (C2.3): start the stride at `filter_phase` instead of 0, i.e. keep the raw
    // indices i with (i - phase) % stride == 0.  phase 0 (the default) is byte-identical to the
    // historical/upstream `i % point_filter_num == 0`.  The offset is folded into [0, stride) so
    // any configured value is well defined; it changes only WHICH points of the same message are
    // kept (and, because t0 below is the first RETAINED point's time, by how much the per-point
    // times are rebased).
    const int phase_folded = ((filter_phase % stride_i) + stride_i) % stride_i;
    const std::size_t start = static_cast<std::size_t>(phase_folded);
    cloud->reserve(point_num / stride + 1);
    double t0_sec = 0.0;
    bool t0_set = false;
    auto read_float = [&](std::size_t base, int off) -> float
    {
        if (off < 0)
            return 0.0f;
        const uint8_t *p = msg->data.data() + base + off;
        return *reinterpret_cast<const float *>(p);
    };
    auto read_time = [&](std::size_t base) -> double
    {
        const uint8_t *p = msg->data.data() + base + off_time;
        if (time_datatype == sensor_msgs::msg::PointField::FLOAT64)
            return (*reinterpret_cast<const double *>(p)) * time_scale;
        return static_cast<double>((*reinterpret_cast<const float *>(p)) * static_cast<float>(time_scale));
    };

    for (std::size_t i = start; i < point_num; i += stride)
    {
        std::size_t base = i * point_step;
        float x = read_float(base, off_x);
        float y = read_float(base, off_y);
        float z = read_float(base, off_z);
        if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
            continue;
        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = read_float(base, off_int);
        if (off_time >= 0)
        {
            double t = read_time(base); // 秒
            if (!t0_set)
            {
                t0_sec = t;
                t0_set = true;
            }
            p.curvature = static_cast<float>((t - t0_sec) * 1000.0); // 毫秒，相对扫描起始
        }
        else
        {
            p.curvature = 0.0f; // 无每点时间，扫描内不做运动补偿
        }
        cloud->push_back(p);
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
