#include "utils.h"
#include <sensor_msgs/msg/point_field.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
using PF = sensor_msgs::msg::PointField;

// 字节宽度；未知 datatype 返回 0
std::size_t datatypeSize(int datatype)
{
    switch (datatype)
    {
    case PF::INT8:
    case PF::UINT8:
        return 1;
    case PF::INT16:
    case PF::UINT16:
        return 2;
    case PF::INT32:
    case PF::UINT32:
    case PF::FLOAT32:
        return 4;
    case PF::FLOAT64:
        return 8;
    default:
        return 0;
    }
}

template <typename T>
double readAs(const uint8_t *p)
{
    T v;
    std::memcpy(&v, p, sizeof(T)); // 字段不保证对齐，不能 reinterpret_cast 解引用
    return static_cast<double>(v);
}

double readScalar(const uint8_t *p, int datatype)
{
    switch (datatype)
    {
    case PF::INT8: return readAs<int8_t>(p);
    case PF::UINT8: return readAs<uint8_t>(p);
    case PF::INT16: return readAs<int16_t>(p);
    case PF::UINT16: return readAs<uint16_t>(p);
    case PF::INT32: return readAs<int32_t>(p);
    case PF::UINT32: return readAs<uint32_t>(p);
    case PF::FLOAT32: return readAs<float>(p);
    case PF::FLOAT64: return readAs<double>(p);
    default: return std::numeric_limits<double>::quiet_NaN();
    }
}

struct FieldRef
{
    int offset = -1;
    int datatype = 0;
    bool valid() const { return offset >= 0; }
};

// 按名字查字段，并要求它完整落在一个点内
FieldRef findField(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
    FieldRef ref;
    for (const auto &f : msg.fields)
    {
        if (f.name != name)
            continue;
        const std::size_t sz = datatypeSize(f.datatype);
        if (sz == 0 || static_cast<std::size_t>(f.offset) + sz > msg.point_step)
            return ref;
        ref.offset = static_cast<int>(f.offset);
        ref.datatype = static_cast<int>(f.datatype);
        return ref;
    }
    return ref;
}
} // namespace

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range, int max_line)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // point_num 是驱动声明的点数，不保证等于 points.size()；filter_num <= 0 时按 1 处理(否则除零/死循环)
    const std::size_t point_num = std::min<std::size_t>(msg->point_num, msg->points.size());
    const std::size_t stride = static_cast<std::size_t>(std::max(1, filter_num));
    cloud->reserve(point_num / stride + 1);
    for (std::size_t i = 0; i < point_num; i += stride)
    {
        if ((msg->points[i].line < max_line) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
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

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pcl2_to_PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range, const std::string &time_field, double time_scale, int filter_phase, double *scan_t0_sec)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (scan_t0_sec)
        *scan_t0_sec = std::numeric_limits<double>::quiet_NaN();
    const FieldRef fx = findField(*msg, "x");
    const FieldRef fy = findField(*msg, "y");
    const FieldRef fz = findField(*msg, "z");
    auto is_float = [](const FieldRef &f) { return f.datatype == PF::FLOAT32 || f.datatype == PF::FLOAT64; };
    if (!fx.valid() || !fy.valid() || !fz.valid() || !is_float(fx) || !is_float(fy) || !is_float(fz))
        return cloud; // 不含(浮点) xyz 字段，无法转换
    const FieldRef fi = findField(*msg, "intensity");
    // 时间字段需按其 datatype 解析(FLOAT32/FLOAT64/UINT32/INT32)，否则禁用每点时间
    FieldRef ft;
    if (!time_field.empty() && time_scale > 0.0)
    {
        ft = findField(*msg, time_field);
        if (ft.datatype != PF::FLOAT32 && ft.datatype != PF::FLOAT64 &&
            ft.datatype != PF::UINT32 && ft.datatype != PF::INT32)
            ft = FieldRef(); // 支持浮点与整型时间字段(如 Velodyne time / RoboSense timestamp)
    }

    const std::size_t point_step = msg->point_step;
    if (point_step == 0)
        return cloud;
    // 声明的点数不能超过 data 实际承载的完整点数(畸形消息不得越界读)
    const std::size_t point_num = std::min<std::size_t>(static_cast<std::size_t>(msg->width) * msg->height,
                                                        msg->data.size() / point_step);
    const int stride_i = std::max(1, filter_num);
    const std::size_t stride = static_cast<std::size_t>(stride_i);
    // Sampling phase (C2.3): start the stride at `filter_phase` instead of 0, i.e. keep the raw
    // indices i with (i - phase) % stride == 0.  phase 0 (the default) is byte-identical to the
    // historical/upstream `i % point_filter_num == 0`.  The offset is folded into [0, stride) so
    // any configured value is well defined; it changes only WHICH points of the same message are
    // kept (and, because t0 below is the earliest RETAINED point's time, by how much the
    // per-point times are rebased).
    const int phase_folded = ((filter_phase % stride_i) + stride_i) % stride_i;
    const std::size_t start = static_cast<std::size_t>(phase_folded);
    cloud->reserve(point_num / stride + 1);
    const uint8_t *data = msg->data.data();

    std::vector<double> times;
    if (ft.valid())
        times.reserve(point_num / stride + 1);
    for (std::size_t i = start; i < point_num; i += stride)
    {
        const uint8_t *base = data + i * point_step;
        const double x = readScalar(base + fx.offset, fx.datatype);
        const double y = readScalar(base + fy.offset, fy.datatype);
        const double z = readScalar(base + fz.offset, fz.datatype);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        const double r2 = x * x + y * y + z * z;
        if (r2 < min_range * min_range || r2 > max_range * max_range)
            continue;
        pcl::PointXYZINormal p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        p.intensity = fi.valid() ? static_cast<float>(readScalar(base + fi.offset, fi.datatype)) : 0.0f;
        p.curvature = 0.0f; // 无每点时间，扫描内不做运动补偿
        if (ft.valid())
        {
            const double t = readScalar(base + ft.offset, ft.datatype) * time_scale; // 秒
            if (!std::isfinite(t))
                continue;
            times.push_back(t);
        }
        cloud->push_back(p);
    }
    double t0_sec = std::numeric_limits<double>::quiet_NaN();
    if (!times.empty())
    {
        // Rebase in double on the EARLIEST retained point: points need not be stored in time order,
        // and absolute epoch seconds would lose ~0.1 ms precision if stored as float directly.
        t0_sec = *std::min_element(times.begin(), times.end());
        for (std::size_t k = 0; k < cloud->size(); ++k)
            cloud->points[k].curvature = static_cast<float>((times[k] - t0_sec) * 1000.0); // 毫秒，相对扫描起始
    }
    if (scan_t0_sec)
        *scan_t0_sec = t0_sec;
    return cloud;
}

double Utils::scanStartTime(double header_sec, double point_t0, bool *inconsistent)
{
    if (inconsistent)
        *inconsistent = false;
    if (!std::isfinite(point_t0))
        return header_sec;
    if (std::fabs(point_t0 - header_sec) < 1.0)
        return point_t0; // 绝对时间：最早点即扫描起点，与驱动把帧头打在首点还是末点无关
    if (std::fabs(point_t0) < 1.0)
        return header_sec + point_t0; // 相对帧头的偏移：起点 = 帧头 + 最早偏移(可为负，如帧头=末点)
    if (inconsistent)
        *inconsistent = true;
    return header_sec;
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
