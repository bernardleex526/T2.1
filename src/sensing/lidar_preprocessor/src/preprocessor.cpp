#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <vector>
#include <cmath>

class PointCloudToCmdVel : public rclcpp::Node
{
public:
    PointCloudToCmdVel() : Node("point_cloud_to_cmd_vel")
    {
        // ====================== 参数声明 ======================
        // 基础话题
        this->declare_parameter<std::string>("input_cloud_topic", "/mapping/body_cloud");
        this->declare_parameter<std::string>("output_cloud_topic", "/preprocessed_cloud");
        this->declare_parameter<std::string>("obstacle_cloud_topic", "/obstacle_cloud");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_test");

        // 点云预处理参数
        this->declare_parameter<double>("min_dist", 0.2);
        this->declare_parameter<double>("max_dist", 1.0);
        this->declare_parameter<int>("stat_k", 10);
        this->declare_parameter<double>("std_dev_mult", 2.0);
        this->declare_parameter<bool>("use_voxel_filter", false);      // 是否启用体素滤波
        this->declare_parameter<double>("voxel_leaf_size", 0.05);       // 体素大小

        // 3D聚类参数
        this->declare_parameter<double>("cluster_tolerance", 0.08);
        this->declare_parameter<int>("min_cluster_size", 30);
        this->declare_parameter<int>("max_cluster_size", 1500);

        // 势场避障参数
        this->declare_parameter<double>("safety_distance", 0.7);        // 触发避障的距离阈值（米）
        this->declare_parameter<double>("linear_gain", 1.0);            // 斥力对线速度的增益
        this->declare_parameter<double>("max_linear_speed", 0.3);       // 最大线速度（m/s）
        this->declare_parameter<double>("min_drive_threshold", 0.2);    // 最小驱动速度阈值（m/s）

        // 日志控制
        this->declare_parameter<double>("log_throttle_sec", 1.0);

        // 加载参数
        get_params();

        // ROS接口初始化
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params_.input_topic, 10,
            std::bind(&PointCloudToCmdVel::cloud_callback, this, std::placeholders::_1)
        );
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.output_topic, 10);
        obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.obstacle_cloud_topic, 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(params_.cmd_vel_topic, 10);

        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "【全向势场避障】基于欧几里得聚类");
        RCLCPP_INFO(this->get_logger(), "安全距离: %.2fm | 最小驱动: %.2fm/s", params_.safety_distance, params_.min_drive_threshold);
        RCLCPP_INFO(this->get_logger(), "最大速度: %.2fm/s | 体素滤波: %s", params_.max_linear_speed, params_.use_voxel_filter ? "启用" : "禁用");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }

private:
    struct Params {
        std::string input_topic, output_topic, obstacle_cloud_topic, cmd_vel_topic;
        double min_dist, max_dist;
        int stat_k;
        double std_dev_mult;
        bool use_voxel_filter;
        double voxel_leaf_size;
        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        double safety_distance, linear_gain, max_linear_speed, min_drive_threshold;
        double log_throttle;
    };

    Params params_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void get_params()
    {
        this->get_parameter("input_cloud_topic", params_.input_topic);
        this->get_parameter("output_cloud_topic", params_.output_topic);
        this->get_parameter("obstacle_cloud_topic", params_.obstacle_cloud_topic);
        this->get_parameter("cmd_vel_topic", params_.cmd_vel_topic);
        this->get_parameter("min_dist", params_.min_dist);
        this->get_parameter("max_dist", params_.max_dist);
        this->get_parameter("stat_k", params_.stat_k);
        this->get_parameter("std_dev_mult", params_.std_dev_mult);
        this->get_parameter("use_voxel_filter", params_.use_voxel_filter);
        this->get_parameter("voxel_leaf_size", params_.voxel_leaf_size);
        this->get_parameter("cluster_tolerance", params_.cluster_tolerance);
        this->get_parameter("min_cluster_size", params_.min_cluster_size);
        this->get_parameter("max_cluster_size", params_.max_cluster_size);
        this->get_parameter("safety_distance", params_.safety_distance);
        this->get_parameter("linear_gain", params_.linear_gain);
        this->get_parameter("max_linear_speed", params_.max_linear_speed);
        this->get_parameter("min_drive_threshold", params_.min_drive_threshold);
        this->get_parameter("log_throttle_sec", params_.log_throttle);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // ====================== 1. 预处理 ======================
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud_raw);
        size_t raw_count = cloud_raw->size();

        // 距离裁剪
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dist(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : *cloud_raw) {
            double dist = std::hypot(point.x, point.y);
            if (dist > params_.min_dist && dist < params_.max_dist) {
                cloud_dist->points.push_back(point);
            }
        }
        cloud_dist->width = cloud_dist->size();
        cloud_dist->height = 1;
        cloud_dist->is_dense = true;

        // 统计滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (cloud_dist->size() > static_cast<size_t>(params_.stat_k)) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_dist);
            sor.setMeanK(params_.stat_k);
            sor.setStddevMulThresh(params_.std_dev_mult);
            sor.filter(*cloud_filtered);
        } else {
            *cloud_filtered = *cloud_dist;
        }

        // 可选体素滤波
        if (params_.use_voxel_filter) {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud_filtered);
            vg.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size, params_.voxel_leaf_size);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            vg.filter(*cloud_downsampled);
            cloud_filtered = cloud_downsampled;
        }

        size_t final_count = cloud_filtered->size();

        // 发布预处理点云（可选）
        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(*cloud_filtered, cloud_out);
        cloud_out.header = cloud_msg->header;
        cloud_pub_->publish(cloud_out);

        // ====================== 2. 欧几里得聚类 ======================
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (final_count >= static_cast<size_t>(params_.min_cluster_size)) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filtered);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(params_.cluster_tolerance);
            ec.setMinClusterSize(params_.min_cluster_size);
            ec.setMaxClusterSize(params_.max_cluster_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);
        }

        // ====================== 3. 计算斥力 ======================
        double repulsion_x = 0.0, repulsion_y = 0.0;
        int valid_clusters = 0;

        for (const auto &indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &idx : indices.indices) {
                cluster->points.push_back(cloud_filtered->points[idx]);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;

            // 寻找簇中距离机器人最近的点（xy平面）
            double min_dist_xy = std::numeric_limits<double>::max();
            pcl::PointXYZ closest_point;
            for (const auto &p : cluster->points) {
                double d = std::hypot(p.x, p.y);
                if (d < min_dist_xy) {
                    min_dist_xy = d;
                    closest_point = p;
                }
            }

            // 如果最近点小于安全距离，则累加斥力
            if (min_dist_xy < params_.safety_distance) {
                valid_clusters++;
                // 权重线性衰减：距离越近，权重越大
                double weight = (params_.safety_distance - min_dist_xy) / params_.safety_distance;
                // 斥力方向：从障碍物指向机器人（-closest_point.x, -closest_point.y）归一化
                double norm = std::hypot(closest_point.x, closest_point.y);
                if (norm > 1e-6) {
                    repulsion_x += (-closest_point.x / norm) * weight;
                    repulsion_y += (-closest_point.y / norm) * weight;
                }
            }

            // 收集所有障碍物点云用于可视化（可选）
            *obstacle_cloud += *cluster;
        }

        // ====================== 4. 合成全向速度 ======================
        geometry_msgs::msg::Twist cmd;

        // 初始速度由斥力乘增益得到
        double vx = params_.linear_gain * repulsion_x;
        double vy = params_.linear_gain * repulsion_y;

        // 检查是否需要放大以满足最小驱动要求
        double max_abs = std::max(std::abs(vx), std::abs(vy));
        if (max_abs > 1e-6 && max_abs < params_.min_drive_threshold) {
            double scale = params_.min_drive_threshold / max_abs;
            vx *= scale;
            vy *= scale;
        }

        // 限幅：确保合速度不超过最大线速度
        double speed = std::hypot(vx, vy);
        if (speed > params_.max_linear_speed) {
            vx = vx * params_.max_linear_speed / speed;
            vy = vy * params_.max_linear_speed / speed;
        }

        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = 0.0;  // 全向移动不需要旋转

        cmd_vel_pub_->publish(cmd);

        // ====================== 5. 发布障碍物点云（可选） ======================
        sensor_msgs::msg::PointCloud2 obstacle_out;
        pcl::toROSMsg(*obstacle_cloud, obstacle_out);
        obstacle_out.header = cloud_msg->header;
        obstacle_cloud_pub_->publish(obstacle_out);

        // ====================== 6. 日志 ======================
        auto &clk = *this->get_clock();
        // RCLCPP_INFO_THROTTLE(this->get_logger(), clk, static_cast<int>(params_.log_throttle * 1000),
        //     "原始点数: %zu | 聚类数: %zu | 有效障碍簇: %d | 斥力(x=%.2f, y=%.2f) | 速度(%.2f, %.2f) m/s",
        //     raw_count, cluster_indices.size(), valid_clusters, repulsion_x, repulsion_y, vx, vy);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToCmdVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}