#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>

#include "interface/srv/save_colored_pcd.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string image_topic = "/camera2/camera/color/image_raw";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    // PointCloud2 模式下的每点时间字段适配
    std::string pcl2_time_field = "";   // 为空则不做扫描内运动补偿
    double pcl2_time_scale = 1.0;       // 字段值×该系数 = 偏移秒(Velodyne time=1.0)
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::mutex image_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    double last_image_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    std::deque<std::pair<double, cv::Mat>> image_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started - Multi-threaded Version");
        
        // 创建回调组
        m_timer_callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        m_sensor_callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);  // 使用Reentrant允许多个传感器回调同时执行
        
        m_service_callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        loadParameters();
        
        // 初始化ROS2组件 - 使用回调组
        initializeSubscribersWithCallbackGroups();
        initializePublishers();
        initializeServicesWithCallbackGroups();
        initializeTimerWithCallbackGroup();
        initializeTFBroadcaster();
        
        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        // 初始化IESKF和MapBuilder
        m_kf = std::make_shared<IESKF>(this->get_logger());
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_builder->setLogger(this->get_logger());
        
        m_global_colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>());
            
        RCLCPP_INFO(this->get_logger(), "LIO Node initialized with multi-threaded callbacks");
    }

private:
    // ==================== 回调组初始化 ====================
    void initializeSubscribersWithCallbackGroups()
    {
        // 为每个订阅者设置回调组
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = m_sensor_callback_group;
        
        // 初始化传感器订阅者
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LIONode::imuCB, this, std::placeholders::_1),
            sub_options);

        // 根据雷达类型订阅 CustomMsg(Livox) 或通用 PointCloud2
        if (m_builder_config.lidar_type == "livox")
        {
            m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                m_node_config.lidar_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&LIONode::lidarCB, this, std::placeholders::_1),
                sub_options);
            RCLCPP_INFO(this->get_logger(), "LiDAR type: livox (CustomMsg), max_line=%d", m_builder_config.lidar_max_line);
        }
        else
        {
            m_pcl_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                m_node_config.lidar_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&LIONode::lidarPcl2CB, this, std::placeholders::_1),
                sub_options);
            RCLCPP_INFO(this->get_logger(), "LiDAR type: pointcloud2 (generic), time_field='%s'", m_node_config.pcl2_time_field.c_str());
        }

        m_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            m_node_config.image_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LIONode::imageCB, this, std::placeholders::_1),
            sub_options);
            
        RCLCPP_INFO(this->get_logger(), "Subscribers initialized with callback group");
    }

    void initializeServicesWithCallbackGroups()
    {
        // 正确的方法：直接传递回调组指针
        m_save_colored_pcd_srv = this->create_service<interface::srv::SaveColoredPcd>(
            "/mapping/save_colored_pcd",
            std::bind(&LIONode::saveColoredPcdCB, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(),
            m_service_callback_group);
            
        RCLCPP_INFO(this->get_logger(), "Service initialized with dedicated callback group");
    }

    void initializeTimerWithCallbackGroup()
    {
        // 定时器使用独立的回调组，避免阻塞传感器回调
        m_timer = this->create_wall_timer(
            20ms,
            std::bind(&LIONode::timerCB, this),
            m_timer_callback_group);
            
        RCLCPP_INFO(this->get_logger(), "Timer initialized with dedicated callback group (20ms)");
    }

    void initializePublishers()
    {
        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_color_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("color_world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    }

    void initializeTFBroadcaster()
    {
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    // ==================== 修改后的loadParameters函数 ====================
    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        try {
            YAML::Node config = YAML::LoadFile(config_path);
            if (!config || config.IsNull()) {
                RCLCPP_WARN(this->get_logger(), "YAML file is empty or does not exist: %s", config_path.c_str());
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

            // 安全地读取配置参数，使用默认值
            m_node_config.imu_topic = config["imu_topic"] ? config["imu_topic"].as<std::string>() : m_node_config.imu_topic;
            m_node_config.lidar_topic = config["lidar_topic"] ? config["lidar_topic"].as<std::string>() : m_node_config.lidar_topic;
            m_node_config.image_topic = config["image_topic"] ? config["image_topic"].as<std::string>() : m_node_config.image_topic;
            m_node_config.body_frame = config["body_frame"] ? config["body_frame"].as<std::string>() : m_node_config.body_frame;
            m_node_config.world_frame = config["world_frame"] ? config["world_frame"].as<std::string>() : m_node_config.world_frame;
            m_node_config.print_time_cost = config["print_time_cost"] ? config["print_time_cost"].as<bool>() : m_node_config.print_time_cost;
            m_node_config.pcl2_time_field = config["pcl2_time_field"] ? config["pcl2_time_field"].as<std::string>() : m_node_config.pcl2_time_field;
            m_node_config.pcl2_time_scale = config["pcl2_time_scale"] ? config["pcl2_time_scale"].as<double>() : m_node_config.pcl2_time_scale;

            // MapBuilder配置
            if (config["lidar_filter_num"]) m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
            if (config["lidar_min_range"]) m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
            if (config["lidar_max_range"]) m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
            if (config["scan_resolution"]) m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
            if (config["map_resolution"]) m_builder_config.map_resolution = config["map_resolution"].as<double>();
            if (config["cube_len"]) m_builder_config.cube_len = config["cube_len"].as<double>();
            if (config["det_range"]) m_builder_config.det_range = config["det_range"].as<double>();
            if (config["move_thresh"]) m_builder_config.move_thresh = config["move_thresh"].as<double>();
            if (config["na"]) m_builder_config.na = config["na"].as<double>();
            if (config["ng"]) m_builder_config.ng = config["ng"].as<double>();
            if (config["nba"]) m_builder_config.nba = config["nba"].as<double>();
            if (config["nbg"]) m_builder_config.nbg = config["nbg"].as<double>();

            if (config["imu_init_num"]) m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
            if (config["near_search_num"]) m_builder_config.near_search_num = config["near_search_num"].as<int>();
            if (config["ieskf_max_iter"]) m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
            if (config["gravity_align"]) m_builder_config.gravity_align = config["gravity_align"].as<bool>();
            if (config["esti_il"]) m_builder_config.esti_il = config["esti_il"].as<bool>();
            if (config["point_quality_thresh"]) m_builder_config.point_quality_thresh = config["point_quality_thresh"].as<double>();

            // 雷达/IMU 适配配置
            if (config["lidar_type"]) m_builder_config.lidar_type = config["lidar_type"].as<std::string>();
            if (config["lidar_max_line"]) m_builder_config.lidar_max_line = config["lidar_max_line"].as<int>();
            if (config["imu_acc_scale"]) m_builder_config.imu_acc_scale = config["imu_acc_scale"].as<double>();
            
            // 外参配置 - 更安全的处理
            if (config["ext_il"] && config["ext_il"].IsSequence() && config["ext_il"].size() >= 7) {
                std::vector<double> ext_il_vec = config["ext_il"].as<std::vector<double>>();
                V3D t_il(ext_il_vec[0], ext_il_vec[1], ext_il_vec[2]);
                Eigen::Quaterniond q_il(ext_il_vec[6], ext_il_vec[3], ext_il_vec[4], ext_il_vec[5]);
                // The extrinsic is documented and consumed as a ROTATION, so q_il must be
                // unit-norm before toRotationMatrix(): Eigen's conversion assumes |q| = 1 and
                // otherwise returns |q|^2 * Rot(q), i.e. a matrix that is NOT orthogonal.
                // A yaml-rounded quaternion (e.g. (0,-0.7071068,0,0.7071068): |q|^2 = 1+5.3e-8)
                // then makes r_il violate Sophus' isOrthogonal() tolerance (1e-10) and the
                // estimator aborts in State::operator- (SO3d(r_il^T r_il)). Normalise here so
                // every r_il consumer sees an exact rotation, whatever the config precision.
                if (q_il.norm() > 0.0) q_il.normalize();
                m_builder_config.r_il = q_il.toRotationMatrix();
                m_builder_config.t_il = t_il;
            } else {
                RCLCPP_WARN(this->get_logger(), "Missing or invalid ext_il parameter, using defaults");
            }

            if (config["ext_lc"] && config["ext_lc"].IsSequence() && config["ext_lc"].size() >= 7) {
                std::vector<double> ext_lc_vec = config["ext_lc"].as<std::vector<double>>();
                V3D t_lc(ext_lc_vec[0], ext_lc_vec[1], ext_lc_vec[2]);
                Eigen::Quaterniond q_lc(ext_lc_vec[6], ext_lc_vec[3], ext_lc_vec[4], ext_lc_vec[5]);
                if (q_lc.norm() > 0.0) q_lc.normalize();   // same reason as q_il above
                m_builder_config.r_cl = q_lc.toRotationMatrix().transpose();
                m_builder_config.t_cl = -m_builder_config.r_cl * t_lc;
            } else {
                RCLCPP_WARN(this->get_logger(), "Missing or invalid ext_lc parameter, using defaults");
            }

            // 相机参数
            if (config["cam_width"]) m_builder_config.cam_width = config["cam_width"].as<double>();
            if (config["cam_height"]) m_builder_config.cam_height = config["cam_height"].as<double>();
            if (config["cam_fx"]) m_builder_config.cam_fx = config["cam_fx"].as<double>();
            if (config["cam_fy"]) m_builder_config.cam_fy = config["cam_fy"].as<double>();
            if (config["cam_cx"]) m_builder_config.cam_cx = config["cam_cx"].as<double>();
            if (config["cam_cy"]) m_builder_config.cam_cy = config["cam_cy"].as<double>();
            
            // 相机畸变参数
            if (config["cam_d"] && config["cam_d"].IsSequence()) {
                m_builder_config.cam_d = Vec<double>(config["cam_d"].as<std::vector<double>>());
            }

            if (config["lidar_cov_inv"]) m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

            // 状态约束参数 - 使用默认值
            m_builder_config.max_bias_gyro = config["max_bias_gyro"] ? config["max_bias_gyro"].as<double>() : 0.1;
            m_builder_config.max_bias_accel = config["max_bias_accel"] ? config["max_bias_accel"].as<double>() : 0.2;
            m_builder_config.max_velocity = config["max_velocity"] ? config["max_velocity"].as<double>() : 10.0;

            // 设置到State类中
            State::max_bias_gyro = m_builder_config.max_bias_gyro;
            State::max_bias_accel = m_builder_config.max_bias_accel;
            State::max_velocity = m_builder_config.max_velocity;
            
            RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Using default parameters due to YAML error");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading parameters: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Using default parameters");
        }
    }

    // ==================== 传感器回调函数保持不变 ====================
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * m_builder_config.imu_acc_scale,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range, m_builder_config.lidar_max_line);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    // 通用 PointCloud2 雷达接入(Velodyne / Ouster / RoboSense 等发布 PointCloud2 的雷达)
    void lidarPcl2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::pcl2_to_PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range, m_node_config.pcl2_time_field, m_node_config.pcl2_time_scale);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }
    
    void imageCB(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.image_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_image_time)
        {
            RCLCPP_WARN(this->get_logger(), "Image Message is out of order");
            std::deque<std::pair<double, cv::Mat>>().swap(m_state_data.image_buffer);
        }
        cv::Mat image_mat = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        m_state_data.image_buffer.emplace_back(timestamp, image_mat);
        m_state_data.last_image_time = timestamp;
    }

    // ==================== 同步和处理函数保持不变 ====================
    bool syncPackage()
    {
        // 取当前帧激光(若尚未取出)。所有 lidar_buffer 访问均加锁，避免与 lidarCB/乱序清空竞争。
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
            if (m_state_data.lidar_buffer.empty())
                return false;
            if (!m_state_data.lidar_pushed)
            {
                m_package.cloud = m_state_data.lidar_buffer.front().second;
                std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                          { return p1.curvature < p2.curvature; });
                m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
                m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / double(1000.0);
                m_package.lidar_end = false;
                m_state_data.lidar_pushed = true;
            }
        }

        // 取对齐的图像状态(若存在)。
        bool has_image = false;
        double image_start_time = 0.0;
        bool image_before_cloud_end = false;
        {
            std::lock_guard<std::mutex> lock(m_state_data.image_mutex);
            if (!m_state_data.image_buffer.empty())
            {
                has_image = true;
                image_start_time = m_state_data.image_buffer.front().first;
                image_before_cloud_end = (image_start_time <= m_package.cloud_end_time);
            }
        }

        double last_imu_time;
        {
            std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
            last_imu_time = m_state_data.last_imu_time;
        }

        // 无图像或图像尚未到达本帧末尾 -> 纯激光分支
        if (!has_image || !image_before_cloud_end)
        {
            if (last_imu_time < m_package.cloud_end_time)
                return false;
            m_package.imus.clear();
            {
                std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
                if (m_state_data.imu_buffer.empty())
                    return false; // 缓冲被乱序清空，本周期重试，避免空 deque front() UB
                while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
                {
                    m_package.imus.push_back(m_state_data.imu_buffer.front());
                    m_state_data.imu_buffer.pop_front();
                }
            }
            {
                std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
                if (!m_state_data.lidar_buffer.empty())
                    m_state_data.lidar_buffer.pop_front();
            }
            m_package.lidar_end = true;
            m_state_data.lidar_pushed = false;
            return true;
        }

        // 图像已在本帧时间窗内 -> 视觉+激光融合分支
        if (image_start_time < m_package.cloud_start_time)
        {
            std::lock_guard<std::mutex> lock(m_state_data.image_mutex);
            if (!m_state_data.image_buffer.empty())
                m_state_data.image_buffer.pop_front();
            return false;
        }

        if (last_imu_time < image_start_time)
            return false;

        m_package.imus.clear();
        m_package.image_time = image_start_time;
        {
            std::lock_guard<std::mutex> lock(m_state_data.image_mutex);
            if (!m_state_data.image_buffer.empty())
                m_package.image = m_state_data.image_buffer.front().second;
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
            if (m_state_data.imu_buffer.empty())
                return false; // 避免空 deque front() UB
            while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < image_start_time)
            {
                m_package.imus.push_back(m_state_data.imu_buffer.front());
                m_state_data.imu_buffer.pop_front();
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.image_mutex);
            if (!m_state_data.image_buffer.empty())
                m_state_data.image_buffer.pop_front();
        }
        m_package.lidar_end = false;
        return true;
    }

    // ==================== 发布函数保持不变 ====================
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        if (cloud == nullptr || cloud->empty())
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    // ==================== timerCB保持不变 ====================
    void timerCB()
    {
        if (!syncPackage())
            return;

        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.lidar_end ? m_package.cloud_end_time : m_package.image_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.lidar_end ? m_package.cloud_end_time : m_package.image_time);

        if (m_package.lidar_end)
        {
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

            CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

            publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

            publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        }
        else
        {
            if (m_builder->colorRendered())
            {
                // 获取当前帧彩色点云（单帧）
                auto current_colored_cloud = m_builder->coloredCloud();
                if (current_colored_cloud && !current_colored_cloud->empty())
                {
                    // 避免与saveColoredPcdCB并发访问全局点云
                    std::lock_guard<std::mutex> lock(m_global_cloud_mutex);
                    // 3. 累加当前帧到全局点云（+= 是PCL库重载运算符，直接合并点云）
                    *m_global_colored_cloud += *current_colored_cloud;
                    RCLCPP_DEBUG(this->get_logger(), "Added %d points to global colored cloud (total: %d)",
                                 current_colored_cloud->size(), m_global_colored_cloud->size());
                }

                // 发布单帧彩色点云
                publishCloud(m_color_world_cloud_pub, current_colored_cloud,
                             m_node_config.world_frame, m_package.image_time);
                m_builder->colorRendered() = false;
            }
        }
    }

    // ==================== saveColoredPcdCB保持不变 ====================
    void saveColoredPcdCB(const std::shared_ptr<interface::srv::SaveColoredPcd::Request> req,
                          const std::shared_ptr<interface::srv::SaveColoredPcd::Response> res)
    {
        // 1. 处理保存路径（原有逻辑不变）
        std::string save_path = req->save_path;
        const char *home_dir = std::getenv("HOME");
        if (home_dir != nullptr && save_path.find("$HOME") != std::string::npos)
        {
            size_t pos = save_path.find("$HOME");
            save_path.replace(pos, 5, home_dir);
        }
        if (save_path.substr(save_path.find_last_of(".") + 1) != "pcd")
        {
            save_path += ".pcd";
            RCLCPP_WARN(this->get_logger(), "Added .pcd suffix to path: %s", save_path.c_str());
        }

        // 2. 检查MapBuilder是否初始化（原有逻辑不变）
        if (!m_builder)
        {
            res->success = false;
            res->message = "MapBuilder is not initialized!";
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        // 3. 加锁读取全局彩色点云（而非单帧）
        std::lock_guard<std::mutex> lock(m_global_cloud_mutex);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = m_global_colored_cloud;

        // 4. 检查全局点云是否为空（提示信息改为"全局点云"）
        if (!colored_cloud || colored_cloud->empty())
        {
            res->success = false;
            res->message = "Global colored point cloud is empty! (No frames accumulated yet)";
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        // 5. 创建保存目录（原有逻辑不变）
        fs::path pcd_path(save_path);
        fs::path parent_dir = pcd_path.parent_path();
        if (!fs::exists(parent_dir))
        {
            try
            {
                fs::create_directories(parent_dir);
                RCLCPP_INFO(this->get_logger(), "Created parent directory: %s", parent_dir.c_str());
            }
            catch (const fs::filesystem_error &e)
            {
                res->success = false;
                res->message = "Failed to create parent directory: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
                return;
            }
        }

        // 6. 保存全局点云（原有逻辑不变，但点云来源已改为全局）
        try
        {
            int save_result = pcl::io::savePCDFileBinary(save_path, *colored_cloud);
            if (save_result == 0)
            {
                res->success = true;
                // 日志中显示"全局点云总点数"
                res->message = "Global colored PCD saved successfully! Path: " + save_path +
                               " (Total point count: " + std::to_string(colored_cloud->size()) + ")";
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
            }
            else
            {
                res->success = false;
                res->message = "PCL save failed! Error code: " + std::to_string(save_result);
                RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            }
        }
        catch (const std::exception &e)
        {
            res->success = false;
            res->message = "Exception during save: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
        }
    }

private:
    // ==================== 成员变量 ====================
    // 回调组
    rclcpp::CallbackGroup::SharedPtr m_timer_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_sensor_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_service_callback_group;
    
    // ROS2订阅者
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcl_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;

    // ROS2发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_color_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    // ROS2定时器和广播器
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    // 数据状态
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    
    // 核心算法组件
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;

    // 保存着色点云
    rclcpp::Service<interface::srv::SaveColoredPcd>::SharedPtr m_save_colored_pcd_srv;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_global_colored_cloud;
    std::mutex m_global_cloud_mutex;
};

// ==================== 主函数 - 使用多线程执行器 ====================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 创建多线程执行器（推荐使用CPU核心数或根据需求指定）
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // 创建LIONode实例
    auto node = std::make_shared<LIONode>();
    
    // 将节点添加到执行器
    executor->add_node(node);
    
    // 运行执行器
    RCLCPP_INFO(node->get_logger(), "Starting LIONode with MultiThreadedExecutor");
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}