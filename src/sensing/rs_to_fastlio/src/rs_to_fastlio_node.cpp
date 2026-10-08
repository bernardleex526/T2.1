#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace rs_to_fastlio
{

// Standard Point type expected by FAST-LIO2
struct PointXYZINormalTime
{
    PCL_ADD_POINT4D;
    float intensity;
    float normal_x; // ring or normal
    float normal_y;
    float normal_z;
    float curvature; // relative time offset in milliseconds from scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace rs_to_fastlio

POINT_CLOUD_REGISTER_POINT_STRUCT(rs_to_fastlio::PointXYZINormalTime,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)

class RsToFastLioNode : public rclcpp::Node
{
public:
    explicit RsToFastLioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("rs_to_fastlio_node", options), frame_count_(0), total_points_(0)
    {
        // Parameters
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/rslidar_points");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/sensing/lidar/pointcloud");
        target_frame_ = this->declare_parameter<std::string>("target_frame", "");
        time_field_name_ = this->declare_parameter<std::string>("time_field_name", "timestamp");
        time_scale_ = this->declare_parameter<double>("time_scale", 1.0e-9);
        min_range_ = this->declare_parameter<double>("min_range", 0.3);
        max_range_ = this->declare_parameter<double>("max_range", 50.0);

        RCLCPP_INFO(this->get_logger(),
            "RsToFastLioNode initialized. Subscribing: %s, Publishing: %s",
            input_topic_.c_str(), output_topic_.c_str());

        auto qos = rclcpp::SensorDataQoS();
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&RsToFastLioNode::pointCloudCallback, this, std::placeholders::_1));

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        frame_count_++;
        auto find_offset = [&](const std::string &name, int &datatype) -> int
        {
            for (const auto &f : msg->fields)
            {
                if (f.name == name)
                {
                    datatype = static_cast<int>(f.datatype);
                    return static_cast<int>(f.offset);
                }
            }
            return -1;
        };

        int dt = 0;
        int off_x = find_offset("x", dt);
        int off_y = find_offset("y", dt);
        int off_z = find_offset("z", dt);
        if (off_x < 0 || off_y < 0 || off_z < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Incoming point cloud missing x, y, or z coordinates.");
            return;
        }

        int off_intensity = find_offset("intensity", dt);
        int off_ring = find_offset("ring", dt);

        int time_datatype = 0;
        int off_time = find_offset(time_field_name_, time_datatype);
        if (off_time < 0)
        {
            // Fallback candidate names: time, t, offset_time
            off_time = find_offset("time", time_datatype);
            if (off_time < 0) off_time = find_offset("t", time_datatype);
            if (off_time < 0) off_time = find_offset("offset_time", time_datatype);
        }

        std::size_t point_step = msg->point_step;
        std::size_t total_num = msg->width * msg->height;

        auto read_float = [&](std::size_t base, int off) -> float
        {
            if (off < 0) return 0.0f;
            const uint8_t *p = msg->data.data() + base + off;
            return *reinterpret_cast<const float *>(p);
        };

        auto read_raw_time = [&](std::size_t base) -> double
        {
            if (off_time < 0) return 0.0;
            const uint8_t *p = msg->data.data() + base + off_time;
            if (time_datatype == sensor_msgs::msg::PointField::FLOAT64)
                return *reinterpret_cast<const double *>(p);
            if (time_datatype == sensor_msgs::msg::PointField::FLOAT32)
                return static_cast<double>(*reinterpret_cast<const float *>(p));
            if (time_datatype == sensor_msgs::msg::PointField::UINT32)
                return static_cast<double>(*reinterpret_cast<const uint32_t *>(p));
            if (time_datatype == sensor_msgs::msg::PointField::INT32)
                return static_cast<double>(*reinterpret_cast<const int32_t *>(p));
            return 0.0;
        };

        double min_t = 1e18;
        double max_t = -1e18;
        bool has_time = (off_time >= 0);

        // First pass: scan start time determination if timestamps present
        if (has_time)
        {
            for (std::size_t i = 0; i < total_num; ++i)
            {
                std::size_t base = i * point_step;
                float x = read_float(base, off_x);
                float y = read_float(base, off_y);
                float z = read_float(base, off_z);
                float dist_sq = x * x + y * y + z * z;
                if (dist_sq >= min_range_ * min_range_ && dist_sq <= max_range_ * max_range_)
                {
                    double t = read_raw_time(base);
                    if (t < min_t) min_t = t;
                    if (t > max_t) max_t = t;
                }
            }
            if (min_t >= max_t) min_t = 0.0;
        }

        pcl::PointCloud<rs_to_fastlio::PointXYZINormalTime> out_cloud;
        out_cloud.reserve(total_num);

        // Auto-scale detection: if raw delta between max_t and min_t is large (>1e6), it's nanoseconds
        double eff_scale = time_scale_;
        if (has_time && (max_t - min_t) > 1e6 && eff_scale >= 1.0)
        {
            eff_scale = 1.0e-9;
        }

        for (std::size_t i = 0; i < total_num; ++i)
        {
            std::size_t base = i * point_step;
            float x = read_float(base, off_x);
            float y = read_float(base, off_y);
            float z = read_float(base, off_z);
            float dist_sq = x * x + y * y + z * z;
            if (dist_sq < min_range_ * min_range_ || dist_sq > max_range_ * max_range_)
                continue;

            rs_to_fastlio::PointXYZINormalTime pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = read_float(base, off_intensity);
            pt.normal_x = (off_ring >= 0) ? read_float(base, off_ring) : 0.0f;
            pt.normal_y = 0.0f;
            pt.normal_z = 0.0f;

            if (has_time)
            {
                double raw_t = read_raw_time(base);
                // curvature in milliseconds
                pt.curvature = static_cast<float>((raw_t - min_t) * eff_scale * 1000.0);
            }
            else
            {
                pt.curvature = 0.0f;
            }
            out_cloud.push_back(pt);
        }

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(out_cloud, out_msg);
        out_msg.header = msg->header;
        if (!target_frame_.empty())
        {
            out_msg.header.frame_id = target_frame_;
        }
        pub_cloud_->publish(out_msg);

        total_points_ += out_cloud.size();
        if (frame_count_ % 100 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "[rs_to_fastlio] Processed %zu frames. Output pts=%zu, time span=%.2f ms, has_time=%s",
                frame_count_, out_cloud.size(),
                has_time ? (max_t - min_t) * eff_scale * 1000.0 : 0.0,
                has_time ? "true" : "false");
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    std::string time_field_name_;
    double time_scale_;
    double min_range_;
    double max_range_;

    std::size_t frame_count_;
    std::size_t total_points_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RsToFastLioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
