#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/conditional_removal.h>         
#include <pcl/filters/passthrough.h>                 
#include <pcl/filters/radius_outlier_removal.h>      
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/filters/voxel_grid.h>                  
#include <pcl/point_types.h>

std::string file_directory;
std::string file_name;
std::string pcd_file;
std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::msg::OccupancyGrid map_topic_msg;
double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double map_resolution = 0.05;
double thre_radius = 0.1;
int thres_point_count = 10;

pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void pass_through_filter(const double &thre_low, const double &thre_high,
                       const bool &flag_in);
void radius_outlier_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
void set_map_topic_msg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pcl_filters");

  node->declare_parameter<std::string>("file_directory", "/home/");
  node->declare_parameter<std::string>("file_name", "map");
  node->declare_parameter<double>("thre_z_min", 0.2);
  node->declare_parameter<double>("thre_z_max", 2.0);
  node->declare_parameter<int>("flag_pass_through", 0);
  node->declare_parameter<double>("thre_radius", 0.5);
  node->declare_parameter<double>("map_resolution", 0.05);
  node->declare_parameter<int>("thres_point_count", 10);
  node->declare_parameter<std::string>("map_topic_name", "map");

  node->get_parameter("file_directory", file_directory);
  node->get_parameter("file_name", file_name);
  node->get_parameter("thre_z_min", thre_z_min);
  node->get_parameter("thre_z_max", thre_z_max);
  node->get_parameter("flag_pass_through", flag_pass_through);
  node->get_parameter("thre_radius", thre_radius);
  node->get_parameter("map_resolution", map_resolution);
  node->get_parameter("thres_point_count", thres_point_count);
  node->get_parameter("map_topic_name", map_topic_name);

  pcd_file = file_directory + file_name + pcd_format;

  rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1))  // 队列深度为1（地图只需最新状态）
                             .transient_local()                 // 持久性策略：TRANSIENT_LOCAL
                             .reliable();  

  auto map_topic_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_name, map_qos);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    RCLCPP_ERROR(node->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
    return (-1);
  }
  RCLCPP_INFO(node->get_logger(), "初始点云数据点数：%ld", pcd_cloud->points.size());

  pass_through_filter(thre_z_min, thre_z_max, static_cast<bool>(flag_pass_through));
  radius_outlier_filter(cloud_after_pass_through, thre_radius, thres_point_count);
  
  set_map_topic_msg(cloud_after_radius, map_topic_msg);

  rclcpp::Rate loop_rate(1.0);
  while (rclcpp::ok()) {
    map_topic_pub->publish(map_topic_msg);
    loop_rate.sleep();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}

void pass_through_filter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  // 修正：替换过时的setFilterLimitsNegative为setNegative
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_pass_through);

  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + file_name + "_filter.pcd",
                                      *cloud_after_pass_through);
  RCLCPP_INFO(rclcpp::get_logger("pcl_filters"), "直通滤波后点云数据点数：%ld",
              cloud_after_pass_through->points.size());
}

void radius_outlier_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  radiusoutlier.setInputCloud(pcd_cloud0);
  radiusoutlier.setRadiusSearch(radius);
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_radius);

  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + file_name + "_radius_filter.pcd",
                                      *cloud_after_radius);
  RCLCPP_INFO(rclcpp::get_logger("pcl_filters"), "半径滤波后点云数据点数：%ld",
              cloud_after_radius->points.size());
}

void set_map_topic_msg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg) {
  // 修正：删除ROS2中不存在的seq赋值
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = rclcpp::Clock().now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("pcl_filters"), "pcd is empty!");
    return;
  }

  for (size_t i = 0; i < cloud->points.size(); i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    x_min = std::min(x_min, x);
    x_max = std::max(x_max, x);
    y_min = std::min(y_min, y);
    y_max = std::max(y_max, y);
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = static_cast<int>((x_max - x_min) / map_resolution);
  msg.info.height = static_cast<int>((y_max - y_min) / map_resolution);
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  RCLCPP_INFO(rclcpp::get_logger("pcl_filters"), "data size = %ld", msg.data.size());

  for (size_t iter = 0; iter < cloud->points.size(); iter++) {
    int i = static_cast<int>((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = static_cast<int>((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height)
      continue;

    msg.data[i + j * msg.info.width] = 100;
  }
}
