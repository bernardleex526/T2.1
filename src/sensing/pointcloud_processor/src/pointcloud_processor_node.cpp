#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <memory>
#include <cmath>

class ProcessedPointCloudPublisher : public rclcpp::Node {
public:
    ProcessedPointCloudPublisher() : Node("processed_pointcloud_publisher") {
        // 参数
        this->declare_parameter("ground_threshold", 0.1);    // 去除地面阈值 (米)
        this->declare_parameter("voxel_size", 0.02);        // 降采样体素大小
        this->declare_parameter("scale_factor", 1.0);       // 放大倍数
        this->declare_parameter("statistical_mean_k", 50);  // 统计滤波邻域点数
        this->declare_parameter("statistical_stddev", 1.0); // 统计滤波标准差
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapping/body_cloud", 10,
            std::bind(&ProcessedPointCloudPublisher::callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/visualization/processed_cloud", 10);
        
        RCLCPP_INFO(this->get_logger(), "点云处理节点启动");
        RCLCPP_INFO(this->get_logger(), "配置: 地面阈值=%.2fm, 体素大小=%.3f, 放大倍数=%.1f", 
                   this->get_parameter("ground_threshold").as_double(),
                   this->get_parameter("voxel_size").as_double(),
                   this->get_parameter("scale_factor").as_double());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        try {
            // 记录原始点云信息
            size_t original_points = msg->width * msg->height;
            RCLCPP_INFO(this->get_logger(), "接收原始点云: %zu 点", original_points);
            
            if (original_points == 0) {
                RCLCPP_WARN(this->get_logger(), "原始点云为空");
                return;
            }
            
            // 1. 转换为PCL点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
            
            RCLCPP_INFO(this->get_logger(), "转换后点云: %ld 点", cloud->size());
            
            // 2. 去除地面 (简单Z轴阈值法)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
            float ground_threshold = this->get_parameter("ground_threshold").as_double();
            
            for (const auto& point : *cloud) {
                // 保留Z大于阈值的点（假设Z向上）
                if (!std::isnan(point.z) && point.z > ground_threshold) {
                    cloud_no_ground->push_back(point);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "去除地面后: %ld 点 (移除了 %.1f%%)", 
                       cloud_no_ground->size(),
                       (cloud->size() - cloud_no_ground->size()) * 100.0 / cloud->size());
            
            if (cloud_no_ground->empty()) {
                RCLCPP_WARN(this->get_logger(), "去除地面后点云为空，发布原始点云");
                // 发布原始点云但设置正确的frame_id
                sensor_msgs::msg::PointCloud2 output_msg = *msg;
                output_msg.header.frame_id = "body";
                output_msg.header.stamp = this->now();
                publisher_->publish(output_msg);
                return;
            }
            
            // 3. 降采样 (体素滤波)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            float voxel_size = this->get_parameter("voxel_size").as_double();
            
            if (voxel_size > 0 && cloud_no_ground->size() > 1000) {
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(cloud_no_ground);
                voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
                voxel_filter.filter(*cloud_downsampled);
                RCLCPP_INFO(this->get_logger(), "降采样后: %ld 点", cloud_downsampled->size());
            } else {
                *cloud_downsampled = *cloud_no_ground;
            }
            
            // 4. 去噪 (统计滤波)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>);
            
            if (cloud_downsampled->size() > 10) {
                try {
                    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                    sor.setInputCloud(cloud_downsampled);
                    sor.setMeanK(this->get_parameter("statistical_mean_k").as_int());
                    sor.setStddevMulThresh(this->get_parameter("statistical_stddev").as_double());
                    sor.filter(*cloud_denoised);
                    RCLCPP_INFO(this->get_logger(), "去噪后: %ld 点", cloud_denoised->size());
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "去噪失败: %s，跳过此步骤", e.what());
                    *cloud_denoised = *cloud_downsampled;
                }
            } else {
                *cloud_denoised = *cloud_downsampled;
            }
            
            // 5. 放大点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scaled(new pcl::PointCloud<pcl::PointXYZ>);
            *cloud_scaled = *cloud_denoised;
            
            float scale_factor = this->get_parameter("scale_factor").as_double();
            if (std::abs(scale_factor - 1.0) > 0.001) {
                for (auto& point : *cloud_scaled) {
                    point.x *= scale_factor;
                    point.y *= scale_factor;
                    point.z *= scale_factor;
                }
                RCLCPP_INFO(this->get_logger(), "放大 %.1f 倍", scale_factor);
            }
            
            // 6. 转换回ROS消息
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_scaled, output_msg);
            
            // 设置正确的header
            output_msg.header.stamp = this->now();
            output_msg.header.frame_id = "body";
            
            // 7. 发布处理后的点云
            publisher_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "发布处理后的点云: %d 点 (原始: %zu, 处理率: %.1f%%)", 
                       output_msg.width * output_msg.height, original_points,
                       (output_msg.width * output_msg.height) * 100.0 / original_points);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理点云时出错: %s", e.what());
            
            // 出错时转发原始点云
            sensor_msgs::msg::PointCloud2 output_msg = *msg;
            output_msg.header.frame_id = "body";
            output_msg.header.stamp = this->now();
            publisher_->publish(output_msg);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessedPointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}