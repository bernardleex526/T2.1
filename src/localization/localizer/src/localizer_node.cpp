#include <queue>
#include <mutex>
#include <filesystem>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/outlier_gate.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/mapping/body_cloud";
    std::string odom_topic = "/mapping/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    std::string pcd_path = ""; // PCD路径
    OffsetGateConfig gate;     // physical-consistency gate of the ICP updates
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    rclcpp::Time last_send_tf_time; // initialized in constructor via node clock
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        m_state.last_send_tf_time = this->now();
        loadParameters();
        m_gate = OffsetOutlierGate(m_config.gate);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // 添加initial_pose订阅
        m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1));

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        // 声明并获取命令行参数
        this->declare_parameter("config_path", "");
        this->declare_parameter("pcd_path", "");

        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        // 优先从命令行获取pcd_path
        this->get_parameter<std::string>("pcd_path", m_config.pcd_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();

        // 如果命令行没有提供，则使用YAML中的配置
        if (m_config.pcd_path.empty())
        {
            // The key used to be absent from localizer.yaml entirely, so this line threw
            // yaml-cpp YAML::TypedBadConversion and the node aborted with an opaque message.
            if (config["pcd_path"])
                m_config.pcd_path = config["pcd_path"].as<std::string>();
        }

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        // Optional: correspondence radii [m] and inlier-ratio floors of the two ICP stages,
        // and the physical-consistency gate of the accepted updates.  Absent keys keep the
        // defaults in icp_localizer.h / outlier_gate.h, so existing configs stay valid.
        if (config["rough_max_corr_dist"]) m_localizer_config.rough_max_corr_dist = config["rough_max_corr_dist"].as<double>();
        if (config["refine_max_corr_dist"]) m_localizer_config.refine_max_corr_dist = config["refine_max_corr_dist"].as<double>();
        if (config["rough_min_inlier_ratio"]) m_localizer_config.rough_min_inlier_ratio = config["rough_min_inlier_ratio"].as<double>();
        if (config["refine_min_inlier_ratio"]) m_localizer_config.refine_min_inlier_ratio = config["refine_min_inlier_ratio"].as<double>();
        if (config["gate_max_translation_m"]) m_config.gate.max_translation_m = config["gate_max_translation_m"].as<double>();
        if (config["gate_max_angle_rad"]) m_config.gate.max_angle_rad = config["gate_max_angle_rad"].as<double>();
        if (config["gate_max_consecutive_rejects"]) m_config.gate.max_consecutive_rejects = config["gate_max_consecutive_rejects"].as<int>();
        if (config["gate_recovery_accepts"]) m_config.gate.recovery_accepts = config["gate_recovery_accepts"].as<int>();
        if (config["gate_ema_alpha"]) m_config.gate.ema_alpha = config["gate_ema_alpha"].as<double>();
        RCLCPP_INFO(this->get_logger(),
                    "correspondence: rough %.2fm/%.2f refine %.2fm/%.2f | gate: %.3fm %.3frad %d rejects/%d recovery alpha %.2f",
                    m_localizer_config.rough_max_corr_dist, m_localizer_config.rough_min_inlier_ratio,
                    m_localizer_config.refine_max_corr_dist, m_localizer_config.refine_min_inlier_ratio,
                    m_config.gate.max_translation_m, m_config.gate.max_angle_rad,
                    m_config.gate.max_consecutive_rejects, m_config.gate.recovery_accepts,
                    m_config.gate.ema_alpha);

        RCLCPP_INFO(this->get_logger(), "Using PCD path: %s", m_config.pcd_path.c_str());

        // Fail fast and human-readably when no prior map was configured. Without a map the
        // localizer cannot relocalize anything, and the previous behaviour was an opaque
        // yaml-cpp YAML::TypedBadConversion backtrace.
        if (m_config.pcd_path.empty())
        {
            throw std::runtime_error(
                "localizer_node: no prior PCD map configured. Set the 'pcd_path' key in the "
                "config file passed via -p config_path:=<file> (see localizer/config/localizer.yaml, "
                "where it defaults to an empty string) or pass it on the command line, e.g. "
                "'ros2 run localizer localizer_node --ros-args -p config_path:=<file> -p pcd_path:=/path/to/map.pcd'. "
                "The 'relocalize' service can also carry a path per request, but one of the two "
                "config sources must be set at startup.");
        }

        if (!std::filesystem::exists(m_config.pcd_path))
        {
            RCLCPP_WARN(this->get_logger(),
                        "configured PCD map does not exist yet: %s (relocalization will fail until it does)",
                        m_config.pcd_path.c_str());
        }
    }

    // 处理RViz发布的初始位姿
    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // 提取位置
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float z = msg->pose.pose.position.z;

        // 四元数转欧拉角
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 创建服务请求
        auto request = std::make_shared<interface::srv::Relocalize::Request>();
        request->pcd_path = m_config.pcd_path; // 使用默认PCD路径
        request->x = x;
        request->y = y;
        request->z = z;
        request->roll = roll;
        request->pitch = pitch;
        request->yaw = yaw;

        // 创建服务响应
        auto response = std::make_shared<interface::srv::Relocalize::Response>();

        // 调用重定位服务
        relocCB(request, response);

        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Relocalization triggered via initial_pose");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Relocalization failed: %s", response->message.c_str());
        }
    }

    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = this->now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = this->now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            // Seed the ICP from the gate's tracking reference (the last accepted ICP estimate,
            // or the re-seeded one while the lock is degraded).  Not from the published
            // transform: that is deliberately frozen at the last trusted value while the lock is
            // invalid, and seeding a stale value would keep the tracker from ever re-locking.
            // The published map<-odom output is not affected by this choice.
            const M3D seed_r = m_gate.engaged() ? m_gate.reference_r() : m_state.last_offset_r;
            const V3D seed_t = m_gate.engaged() ? m_gate.reference_t() : m_state.last_offset_t;
            initial_guess.block<3, 3>(0, 0) = (seed_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (seed_r * m_state.last_t + seed_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            M3D cand_offset_r = map_body_r * current_local_r.transpose();
            V3D cand_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;

            // The gate owns the whole lock lifecycle (see outlier_gate.h).  It is entered on
            // EVERY successful ICP result: the only unconditional-adopt cases are the very first
            // lock and an operator relocalize request - never "the lock is currently degraded",
            // which would leave the gate bypassed forever (review NB1).
            const OffsetGateOutcome outcome =
                m_gate.update(cand_offset_r, cand_offset_t, current_local_t, m_state.service_received);
            m_state.last_offset_r = m_gate.published_r();
            m_state.last_offset_t = m_gate.published_t();
            if (m_state.service_received)
            {
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.service_received = false;
                RCLCPP_INFO(this->get_logger(), "localization lock adopted at t=[%.3f %.3f %.3f]",
                            m_gate.published_t().x(), m_gate.published_t().y(),
                            m_gate.published_t().z());
            }
            else if (outcome.recovered)
            {
                RCLCPP_INFO(this->get_logger(),
                            "localization lock recovered after %d consecutive accepted ICP updates (valid again)",
                            m_config.gate.recovery_accepts);
            }
            else if (outcome.lost)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "outlier gate: %d consecutive ICP updates rejected; declaring the localization INVALID and re-seeding on the current candidate (gating continues, %d accepted updates re-validate the lock)",
                    m_config.gate.max_consecutive_rejects, m_config.gate.recovery_accepts);
            }
            else if (outcome.accepted)
            {
                RCLCPP_DEBUG(this->get_logger(), "ICP update accepted: innovation %.4fm %.4frad",
                             outcome.innovation_m, outcome.innovation_rad);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "ICP update rejected by outlier gate: innovation=%.3fm %.3frad (%d consecutive)",
                    outcome.innovation_m, outcome.innovation_rad, outcome.consecutive_rejects);
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            RCLCPP_ERROR(this->get_logger(), "PCD file not found: %s", pcd_path.c_str());
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", pcd_path.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            // Until the next adopted candidate the lock is NOT valid: the operator is re-seeding
            // it and relocalize_check must not keep answering valid in the meantime.
            m_gate.requestRelock();
        }

        RCLCPP_INFO(this->get_logger(), "Relocalization started with initial pose: [%.2f, %.2f, %.2f] RPY: [%.2f, %.2f, %.2f]",
                    x, y, z, roll, pitch, yaw);

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_gate.valid();
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    OffsetOutlierGate m_gate;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<LocalizerNode>());
    }
    catch (const std::exception &e)
    {
        // e.g. a missing/unset pcd_path: report it as a plain human-readable error instead of
        // an uncaught-exception backtrace from yaml-cpp.
        RCLCPP_FATAL(rclcpp::get_logger("localizer_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}