#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <memory>
#include <string>
#include <future>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include "interface/srv/save_colored_pcd.hpp"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using SaveColoredPcd = interface::srv::SaveColoredPcd;

// 节点配置结构体
struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    int processing_threads = 2;  // 处理线程数，根据开发板调整
    int max_queue_size = 100;    // 最大队列大小
};

// 线程安全的队列
template<typename T>
class ThreadSafeQueue
{
private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::size_t max_size_;

public:
    explicit ThreadSafeQueue(std::size_t max_size = 100) : max_size_(max_size) {}

    bool push(T&& item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_)
        {
            // 队列满时丢弃最旧的数据
            queue_.pop();
            RCLCPP_WARN(rclcpp::get_logger("pgo_node"), 
                       "Queue full, dropping oldest data. Current size: %lu", queue_.size());
        }
        queue_.push(std::move(item));
        lock.unlock();
        cond_.notify_one();
        return true;
    }

    bool pop(T& item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty())
        {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool try_pop(T& item)
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || queue_.empty())
        {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty())
        {
            queue_.pop();
        }
    }
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node"),
                m_processing(false),
                m_shutdown(false)
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        
        // 加载参数
        loadParameters();
        
        // 初始化PGO
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        
        // 初始化回调组 - 使用重入回调组允许多个回调并行执行
        m_callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = m_callback_group;
        
        // 初始化订阅者
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, 
                             rclcpp::SensorDataQoS().get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, 
                            rclcpp::SensorDataQoS().get_rmw_qos_profile());
        
        // 初始化时间同步器
        m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(50), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(
            std::bind(&PGONode::syncCB, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 初始化发布者
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pgo/loop_markers", 10);
        
        // 初始化TF广播器
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        
        // 初始化服务
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
            "/pgo/save_maps",
            std::bind(&PGONode::saveMapsCB, this, 
                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_callback_group);
        
        // 初始化客户端
        m_save_colored_client = this->create_client<SaveColoredPcd>(
            "/mapping/save_colored_pcd", rmw_qos_profile_services_default,
            m_callback_group);
        
        // 启动处理线程
        startProcessingThreads();
        
        // 初始化状态监控定时器
        m_monitor_timer = this->create_wall_timer(
            1000ms, std::bind(&PGONode::monitorTimerCB, this));
        
        RCLCPP_INFO(this->get_logger(), 
                   "PGO node initialized with %d processing threads", 
                   m_node_config.processing_threads);
    }
    
    ~PGONode()
    {
        stopProcessingThreads();
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    
    // 配置和状态
    NodeConfig m_node_config;
    Config m_pgo_config;
    std::shared_ptr<SimplePGO> m_pgo;
    
    // ROS2组件
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Client<SaveColoredPcd>::SharedPtr m_save_colored_client;
    
    // 订阅器和同步器
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> m_sync;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr m_monitor_timer;
    
    // 线程安全队列
    ThreadSafeQueue<CloudWithPose> m_cloud_queue;
    
    // 处理线程
    std::vector<std::thread> m_processing_threads;
    std::atomic<bool> m_processing;
    std::atomic<bool> m_shutdown;
    std::mutex m_pgo_mutex;  // 保护PGO对象
    
    // 加载参数
    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
        
        // 节点配置
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();
        
        // 性能相关配置
        if (config["processing_threads"])
            m_node_config.processing_threads = config["processing_threads"].as<int>();
        if (config["max_queue_size"])
            m_node_config.max_queue_size = config["max_queue_size"].as<int>();
        
        // PGO配置
        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
    }
    
    // 启动处理线程
    void startProcessingThreads()
    {
        m_processing = true;
        m_shutdown = false;
        
        for (int i = 0; i < m_node_config.processing_threads; ++i)
        {
            m_processing_threads.emplace_back(&PGONode::processingThread, this, i);
        }
    }
    
    // 停止处理线程
    void stopProcessingThreads()
    {
        m_shutdown = true;
        m_processing = false;
        
        for (auto& thread : m_processing_threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
        m_processing_threads.clear();
    }
    
    // 处理线程主函数
    void processingThread(int thread_id)
    {
        RCLCPP_INFO(this->get_logger(), "Processing thread %d started", thread_id);
        
        while (!m_shutdown)
        {
            CloudWithPose cp;
            if (m_cloud_queue.try_pop(cp))
            {
                processCloudWithPose(cp, thread_id);
            }
            else
            {
                // 队列为空时短暂休眠
                std::this_thread::sleep_for(1ms);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Processing thread %d stopped", thread_id);
    }
    
    // 处理单个点云-位姿对
    void processCloudWithPose(const CloudWithPose& cp, int thread_id)
    {
        auto start_time = std::chrono::steady_clock::now();
        
        // 构造当前时间戳
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        
        bool is_keyframe = false;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            is_keyframe = m_pgo->addKeyPose(cp);
        }
        
        // 无论是否关键帧，都发布TF
        sendBroadCastTF(cur_time);
        
        if (!is_keyframe)
        {
            // 非关键帧，只更新TF，不进行PGO处理
            return;
        }
        
        // 关键帧处理
        bool has_loop = false;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            m_pgo->searchForLoopPairs();
            has_loop = !m_pgo->historyPairs().empty();
            
            if (has_loop)
            {
                m_pgo->smoothAndUpdate();
                // 优化后需要再次发布TF
                sendBroadCastTF(cur_time);
            }
        }
        
        // 发布回环标记（如果有回环）
        if (has_loop)
        {
            publishLoopMarkers(cur_time);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        if (duration.count() > 50)  // 处理时间超过50ms时警告
        {
            RCLCPP_WARN(this->get_logger(), 
                       "Thread %d: Processing took %ld ms (queue size: %lu)", 
                       thread_id, duration.count(), m_cloud_queue.size());
        }
    }
    
    // 同步回调 - 快速接收数据，放入队列
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        try
        {
            // 构造点云-位姿对
            CloudWithPose cp;
            cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
            
            // 解析位姿
            cp.pose.r = Eigen::Quaterniond(
                            odom_msg->pose.pose.orientation.w,
                            odom_msg->pose.pose.orientation.x,
                            odom_msg->pose.pose.orientation.y,
                            odom_msg->pose.pose.orientation.z)
                            .toRotationMatrix();
            cp.pose.t = V3D(
                odom_msg->pose.pose.position.x,
                odom_msg->pose.pose.position.y,
                odom_msg->pose.pose.position.z);
            
            // 解析点云
            cp.cloud = CloudType::Ptr(new CloudType);
            pcl::fromROSMsg(*cloud_msg, *cp.cloud);
            
            // 放入队列
            if (!m_cloud_queue.push(std::move(cp)))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Cloud queue full, dropping data");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in syncCB: %s", e.what());
        }
    }
    
    // 广播TF（map -> local_frame） - 恢复原始机制，每次处理都发布
    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;

        // 从PGO获取偏移位姿
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();

        // 填充TF数据
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        m_tf_broadcaster->sendTransform(transformStamped);
    }
    
    // 发布回环标记
    void publishLoopMarkers(const builtin_interfaces::msg::Time& time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        
        std::lock_guard<std::mutex> lock(m_pgo_mutex);
        const auto& loop_pairs = m_pgo->historyPairs();
        if (loop_pairs.empty())
            return;
        
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker, edges_marker;
        
        // 节点标记
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;
        
        // 边标记
        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;
        
        // 填充标记数据
        const auto& key_poses = m_pgo->keyPoses();
        for (const auto& pair : loop_pairs)
        {
            size_t i1 = pair.first, i2 = pair.second;
            geometry_msgs::msg::Point p1, p2;
            
            p1.x = key_poses[i1].t_global.x();
            p1.y = key_poses[i1].t_global.y();
            p1.z = key_poses[i1].t_global.z();
            
            p2.x = key_poses[i2].t_global.x();
            p2.y = key_poses[i2].t_global.y();
            p2.z = key_poses[i2].t_global.z();
            
            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }
        
        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }
    
    // 状态监控定时器
    void monitorTimerCB()
    {
        auto queue_size = m_cloud_queue.size();
        if (queue_size > m_node_config.max_queue_size * 0.8)
        {
            RCLCPP_WARN(this->get_logger(), 
                       "High queue load: %lu/%d", 
                       queue_size, m_node_config.max_queue_size);
        }
        
        static size_t last_queue_size = 0;
        static auto last_time = this->now();
        auto current_time = this->now();
        auto dt = (current_time - last_time).seconds();
        
        if (dt > 1.0)
        {
            if (queue_size > last_queue_size)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Queue growing: +%lu/s", 
                           (queue_size - last_queue_size) / static_cast<size_t>(dt));
            }
            last_queue_size = queue_size;
            last_time = current_time;
        }
    }
    
    // 彩色点云保存回调函数
    void colorSaveCallback(rclcpp::Client<SaveColoredPcd>::SharedFuture future)
    {
        try
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "彩色点云保存成功: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "彩色点云保存失败: %s", response->message.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "彩色点云服务调用异常: %s", e.what());
        }
    }
    
    // 保存地图服务回调
    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request,
                    std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received save maps request");
        
        // 1. 创建根目录
        std::filesystem::path root_dir(request->file_path);
        try
        {
            std::filesystem::create_directories(root_dir);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", root_dir.string().c_str());
        }
        catch (const std::filesystem::filesystem_error& e)
        {
            response->success = false;
            response->message = "Failed to create directory: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 2. 检查数据：锁内拷出关键帧快照，所有耗时的文件 IO 与点云变换放到锁外执行，
        //    避免 processingThread 长时间拿不到 m_pgo_mutex 导致丢帧。
        std::vector<KeyPoseWithCloud> key_poses_snapshot;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (!m_pgo || m_pgo->keyPoses().empty())
            {
                response->success = false;
                response->message = "No key poses to save";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            key_poses_snapshot = m_pgo->keyPoses(); // 拷贝(共享 body_cloud 指针，不受后续优化影响)
        }

        // 3. 定义路径
        std::filesystem::path patches_dir = root_dir / "patches";
        std::filesystem::path poses_txt_path = root_dir / "poses.txt";
        std::filesystem::path mono_map_path = root_dir / "map.pcd";
        std::filesystem::path color_map_path = root_dir / "colored_map.pcd";

        // 4. 异步调用彩色点云服务(无需持锁)
        std::string color_status = "Not requested";

        if (m_save_colored_client && m_save_colored_client->service_is_ready())
        {
            auto color_req = std::make_shared<SaveColoredPcd::Request>();
            color_req->save_path = color_map_path.string();

            // 使用异步回调，不等待结果
            auto future = m_save_colored_client->async_send_request(
                color_req,
                std::bind(&PGONode::colorSaveCallback, this, std::placeholders::_1));

            color_status = "Request sent (async)";
            RCLCPP_INFO(this->get_logger(), "Sent async color map save request: %s",
                       color_map_path.string().c_str());
        }
        else
        {
            color_status = "Service not available";
            RCLCPP_WARN(this->get_logger(), "Color map service not ready");
        }

        // 5. 处理分块点云
        std::ofstream poses_txt_file;
        if (request->save_patches)
        {
            try
            {
                if (std::filesystem::exists(patches_dir))
                {
                    std::filesystem::remove_all(patches_dir);
                }
                std::filesystem::create_directories(patches_dir);

                poses_txt_file.open(poses_txt_path);
                if (!poses_txt_file.is_open())
                {
                    throw std::runtime_error("Failed to open poses.txt");
                }
            }
            catch (const std::exception& e)
            {
                response->success = false;
                response->message = "Failed to setup patches: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
        }

        // 6. 处理关键帧(锁外，使用快照)
        CloudType::Ptr merged_mono_cloud(new CloudType);
        size_t total_points = 0;

        try
        {
            for (size_t i = 0; i < key_poses_snapshot.size(); ++i)
            {
                const auto& key_pose = key_poses_snapshot[i];

                if (!key_pose.body_cloud || key_pose.body_cloud->empty())
                {
                    continue;
                }

                // 保存分块
                if (request->save_patches && poses_txt_file.is_open())
                {
                    std::string patch_filename = std::to_string(i) + ".pcd";
                    std::filesystem::path patch_path = patches_dir / patch_filename;

                    if (pcl::io::savePCDFileBinary(patch_path.string(), *key_pose.body_cloud) == 0)
                    {
                        Eigen::Quaterniond pose_quat(key_pose.r_global);
                        Eigen::Vector3d pose_trans(key_pose.t_global);

                        poses_txt_file << patch_filename << " "
                                     << pose_trans.x() << " " << pose_trans.y() << " " << pose_trans.z() << " "
                                     << pose_quat.w() << " " << pose_quat.x() << " " << pose_quat.y() << " " << pose_quat.z() << "\n";
                    }
                }

                // 合并点云
                CloudType::Ptr world_cloud(new CloudType);
                pcl::transformPointCloud(
                    *key_pose.body_cloud,
                    *world_cloud,
                    key_pose.t_global,
                    Eigen::Quaterniond(key_pose.r_global));

                *merged_mono_cloud += *world_cloud;
                total_points += world_cloud->size();

                // 定期报告进度
                if ((i + 1) % 50 == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Processed %lu/%lu keyframes",
                               i + 1, key_poses_snapshot.size());
                }
            }
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "Error processing keyframes: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            if (poses_txt_file.is_open()) poses_txt_file.close();
            return;
        }

        // 7. 关闭文件
        if (poses_txt_file.is_open())
        {
            poses_txt_file.close();
        }

        // 8. 保存单色点云地图
        if (merged_mono_cloud->empty())
        {
            response->success = false;
            response->message = "No points to save";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (pcl::io::savePCDFileBinary(mono_map_path.string(), *merged_mono_cloud) != 0)
        {
            response->success = false;
            response->message = "Failed to save mono map";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 9. 返回结果
        response->success = true;
        response->message = "Map saved successfully. Mono map: " +
                           std::to_string(total_points) + " points. " +
                           "Colored map: " + color_status;

        RCLCPP_INFO(this->get_logger(), "Save maps completed: %s", response->message.c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 使用多线程执行器
    auto node = std::make_shared<PGONode>();
    
    // 配置多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),  // 默认选项
        4  // 线程数，根据开发板调整
    );
    
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}