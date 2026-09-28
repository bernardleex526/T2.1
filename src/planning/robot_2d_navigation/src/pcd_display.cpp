/**
 * @file pcd_display.cpp
 * @author xuankeng he (carlos_hxk@outlook.com)
 * @brief 读取并显示彩色点云到rviz2中（保留原始颜色和数据，无滤波）
 *
 * @version 0.2
 * @date 2025-09-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class ColorPointCloudDisplayNode : public rclcpp::Node
{
public:
    ColorPointCloudDisplayNode() : Node("color_pointcloud_loader")
    {
        // 声明参数并设置默认值
        declare_parameter<std::string>("publish_topic", "color_map_point_cloud");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<std::string>("file_path", "");
        declare_parameter<float>("pub_rate", 0.5f); // 发布频率

        // 获取参数值
        std::string publish_topic = get_parameter("publish_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        std::string file_path = get_parameter("file_path").as_string();
        double pub_rate = get_parameter("pub_rate").as_double();

        // 检查文件路径是否有效
        if (file_path.empty())
        {
            RCLCPP_WARN(get_logger(), "文件路径未指定，无法显示点云");
            return;
        }

        // 加载彩色点云文件
        if (!loadColorPointCloud(file_path))
        {
            RCLCPP_ERROR(get_logger(), "点云处理失败，节点将退出");
            return;
        }

        // 创建发布者
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic, 10);

        // 创建定时器，按指定频率发布点云
        auto period = std::chrono::duration<float>(1.0f / pub_rate);
        timer_ = create_wall_timer(
            period,
            std::bind(&ColorPointCloudDisplayNode::publishPointCloud, this));

        RCLCPP_INFO(get_logger(), "彩色点云显示节点初始化完成");
    }

private:
    // 加载彩色点云
    bool loadColorPointCloud(const std::string &file_path)
    {
        // 使用带颜色的点云类型
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 加载PCD文件
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud) == -1)
        {
            RCLCPP_ERROR(get_logger(), "无法读取点云文件: %s", file_path.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "加载原始彩色点云数量: %zu", cloud->points.size());

        // 直接转换为ROS2消息（保留颜色信息）
        pcl::toROSMsg(*cloud, point_cloud_msg_);
        point_cloud_msg_.header.frame_id = frame_id_;

        return true;
    }

    void publishPointCloud()
    {
        if (publisher_->get_subscription_count() > 0)
        {
            point_cloud_msg_.header.stamp = now();
            publisher_->publish(point_cloud_msg_);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 point_cloud_msg_;
    std::string frame_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorPointCloudDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
