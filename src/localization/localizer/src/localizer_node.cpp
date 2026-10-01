#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <utility>
#include <filesystem>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/lock_validity.h"
#include "localizers/outlier_gate.h"
#include "localizers/pair_cache.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// Stamp -> seconds, the domain every timing decision of this node is made in.
static double toSeconds(const builtin_interfaces::msg::Time &t)
{
    return double(t.sec) + double(t.nanosec) * 1e-9;
}

// Seconds -> stamp, for the TF/cloud outputs.
static builtin_interfaces::msg::Time toStamp(double t)
{
    builtin_interfaces::msg::Time out;
    out.sec = static_cast<int32_t>(std::floor(t));
    out.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    return out;
}

// Depth of the per-frame pairing cache and how far apart a cloud stamp and an odometry stamp may
// be and still be the same scan (half a 10 Hz scan period).
static constexpr size_t kPairCacheDepth = 40;
static constexpr double kPairToleranceS = 0.05;

// update_hz is a FLOOR on the update interval, with this much slack so that a sensor running
// exactly at update_hz (a 10 Hz lidar with update_hz: 10.0) is never decimated by
// sub-millisecond stamp jitter: without it, a 0.099999 s frame gap would be rejected and every
// following frame would be rejected too, halving the output rate for good.
static constexpr double kRateLimitSlack = 0.95;

struct NodeConfig
{
    std::string cloud_topic = "/mapping/body_cloud";
    std::string odom_topic = "/mapping/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    std::string pcd_path = ""; // PCD路径
    OffsetGateConfig gate;     // physical-consistency gate of the ICP updates
    // How long the last CORROBORATED correction stays evidence of a live lock (an accepted
    // candidate under a valid lock, an explicit relock, or a completed first/post-loss
    // qualification; a failed attempt and a rejected candidate renew nothing).  relocalize_check
    // answers from the gate's state, and the gate only changes when an update arrives, so
    // without this bound a node whose intake has stalled (no synced cloud/odom pair, a dead
    // subscriber, a wedged map) keeps answering "valid" forever.  Ten nominal update periods.
    double validity_timeout_s = 1.0;
    // Periodic availability report: achieved update rate and the age of the last accepted
    // update, so a stalled or CPU-bound node is visible in its own log.  0 disables it.
    double report_period_s = 10.0;
};

struct NodeState
{
    // Guards the pair cache: the subscription callbacks write it, the timer reads it.
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool service_received = false;
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    // Whether a VALID map<-odom correction has ever been published.  The first one is broadcast
    // even when it is exactly the identity (the identity is a legitimate correction, and a
    // consumer that started after the localizer must still learn that the lock exists); after
    // that only a published value that actually moved is stamped.
    bool valid_correction_published = false;

    // Per-frame pairing cache.  lio_node stamps one scan's body cloud and its odometry with the
    // same scan-end time (one scan -> one cloud + one odom), so the localizer needs the newest
    // (cloud, odom) pair, not a general N-way approximate-time policy: see pair_cache.h for why
    // the previous message_filters intake was replaced (it was measured to stop delivering
    // pairs permanently while both topics kept arriving at 10 Hz).
    FramePairCache<sensor_msgs::msg::PointCloud2::ConstSharedPtr, std::pair<M3D, V3D>> pairs;
    std::string odom_frame_id;
    std::atomic<size_t> clouds_received{0};
    std::atomic<size_t> odoms_received{0};
    // Freshness bound of the lock-validity answer (see lock_validity.h).
    LockValidity validity;
    // Stamp (seconds) of the last frame an ICP update was ATTEMPTED for.  A frame is measured
    // at most once: re-aligning the same buffered cloud burns the CPU that the sync intake
    // needs, without producing a new sample.
    double last_processed_stamp = -1.0;
    std::atomic<size_t> sync_pairs{0};   // DISTINCT cloud+odom frames paired (intake health)
    double last_paired_stamp = -1.0;     // timer-thread only
    std::atomic<size_t> updates{0};      // ICP updates that produced a gate outcome
    std::atomic<size_t> icp_failures{0}; // frames whose ICP/score test returned no result
    // Cost of one align() call, summed over the process.  This is what decides the achievable
    // output rate on a given prior map, so it is reported instead of guessed at.
    std::atomic<double> align_ms_total{0.0};
    rclcpp::Time last_report;            // node clock at the previous periodic report
    size_t updates_at_report = 0;
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        m_state.last_report = this->now();
        loadParameters();
        m_gate = OffsetOutlierGate(m_config.gate);
        // LATEST-SAMPLE intake.  This node is a freshest-frame consumer, so the transport must
        // drop what it cannot deliver instead of queueing it.  Measured on the corrected fixed3
        // bag with a deep RELIABLE history (depth 50): the executor is busy ~0.15 s per ICP
        // update while the two streams deliver 20 messages/s, so it serviced the callbacks more
        // slowly than they arrived and the samples the pairing cache ever saw were the OLDEST
        // queued ones - the published correction fell further and further behind the live stream
        // (odometry lag constant 3.41 s, correction lag growing 3.6 s -> 8.5 s over 54 s, i.e.
        // ~5 s of self-inflicted backlog).
        //
        // The canonical sensor-data profile (best-effort, KEEP_LAST 5, volatile) is used rather
        // than a reliable reader with a tiny history: a reliable reader whose history overflows
        // constantly was measured to stop delivering odometry altogether about 50 s into a run
        // while the cloud stream continued, which silently freezes the map->odom output.  With
        // best-effort there is no retransmission machinery to get stuck, the backlog is bounded
        // by the history depth (at most ~0.25 s at 10 Hz + 10 Hz), and the pairing always takes
        // the newest complete pair, so a slow consumer degrades to missed frames.
        const auto qos = rclcpp::SensorDataQoS();
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_config.cloud_topic, qos, std::bind(&LocalizerNode::cloudCB, this, std::placeholders::_1));
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            m_config.odom_topic, qos, std::bind(&LocalizerNode::odomCB, this, std::placeholders::_1));

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

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
        if (config["validity_timeout_s"]) m_config.validity_timeout_s = config["validity_timeout_s"].as<double>();
        if (config["report_period_s"]) m_config.report_period_s = config["report_period_s"].as<double>();
        RCLCPP_INFO(this->get_logger(),
                    "correspondence: rough %.2fm/%.2f refine %.2fm/%.2f | gate: %.3fm %.3frad %d rejects/%d recovery alpha %.2f | validity timeout %.2fs report %.0fs",
                    m_localizer_config.rough_max_corr_dist, m_localizer_config.rough_min_inlier_ratio,
                    m_localizer_config.refine_max_corr_dist, m_localizer_config.refine_min_inlier_ratio,
                    m_config.gate.max_translation_m, m_config.gate.max_angle_rad,
                    m_config.gate.max_consecutive_rejects, m_config.gate.recovery_accepts,
                    m_config.gate.ema_alpha, m_config.validity_timeout_s, m_config.report_period_s);

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

    // Fill in the counters of the periodic availability report and log it.  This is what makes
    // a CPU-bound or stalled localizer visible in its own log instead of only in the timing of
    // its output stream.
    void reportAvailability(const char *why)
    {
        const double dt = (this->now() - m_state.last_report).seconds();
        const size_t done = m_state.updates.load() - m_state.updates_at_report;
        const double rate = dt > 0.0 ? double(done) / dt : 0.0;
        const double last = m_state.validity.lastUpdateS();
        const double age = m_state.validity.hasUpdate() ? this->now().seconds() - last : -1.0;
        const size_t attempts = m_state.updates.load() + m_state.icp_failures.load();
        const double mean_align_ms = attempts
                                         ? m_state.align_ms_total.load() / double(attempts)
                                         : 0.0;
        const auto &st = m_localizer->stageTimings();
        const double calls = st.align_calls ? double(st.align_calls) : 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "availability report (%s): %.2f ICP updates/s over %.1fs (%zu updates, %zu "
                    "synced frames, %zu ICP failures, mean align %.1f ms [rough %.1f + fit %.1f "
                    "+ refine %.1f + fit %.1f]) | last corroborated correction %.2fs ago | valid=%d | "
                    "intake: %zu clouds, %zu odom | update_hz=%.1f",
                    why, rate, dt, m_state.updates.load(), m_state.sync_pairs.load(),
                    m_state.icp_failures.load(), mean_align_ms,
                    calls ? st.rough_align_ms / calls : 0.0,
                    calls ? st.rough_fitness_ms / calls : 0.0,
                    calls ? st.refine_align_ms / calls : 0.0,
                    calls ? st.refine_fitness_ms / calls : 0.0, age,
                    int(m_gate.valid() && last >= 0.0 &&
                        age <= m_config.validity_timeout_s),
                    m_state.clouds_received.load(), m_state.odoms_received.load(),
                    m_config.update_hz);
        m_state.last_report = this->now();
        m_state.updates_at_report = m_state.updates.load();
    }

    void timerCB()
    {
        if (m_config.report_period_s > 0.0 &&
            (this->now() - m_state.last_report).seconds() >= m_config.report_period_s)
            reportAvailability(m_state.validity.hasUpdate() ? "periodic" : "no lock yet");

        // PER-FRAME consumer.  One (cloud, odom) pair is measured per ICP update and the TF
        // sample carries that frame's stamp and nothing else: the previous code re-broadcast the
        // held offset with the newest received cloud stamp on every wall-clock tick, which
        // fabricated freshness (a consumer could not tell a 5 Hz measurement stream from a 10 Hz
        // one, and a stalled intake kept looking fresh) and re-aligned the same buffered cloud
        // over and over, burning the CPU the intake needs.
        double frame_stamp;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr frame_cloud_msg;
        M3D frame_local_r;
        V3D frame_local_t;
        if (!newestPair(frame_stamp, frame_cloud_msg, frame_local_r, frame_local_t))
            return; // no complete (cloud, odom) pair yet
        // timer-thread only, so no lock is needed here
        if (m_state.last_processed_stamp >= 0.0 &&
            frame_stamp - m_state.last_processed_stamp < (1.0 / m_config.update_hz) * kRateLimitSlack)
            return; // same frame, or the configured update interval has not elapsed
        m_state.last_processed_stamp = frame_stamp;
        CloudType::Ptr frame_cloud = std::make_shared<CloudType>();
        pcl::fromROSMsg(*frame_cloud_msg, *frame_cloud);
        const builtin_interfaces::msg::Time frame_time = toStamp(frame_stamp);

        M4F initial_guess = M4F::Identity();
        bool service_request = false;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            service_request = m_state.service_received;
            if (service_request)
            {
                initial_guess = m_state.initial_guess;
            }
            else
            {
                // Seed the ICP from the gate's tracking reference (the last accepted ICP
                // estimate, or the re-seeded one while the lock is degraded).  Not from the
                // published transform: that is deliberately frozen at the last trusted value
                // while the lock is invalid, and seeding a stale value would keep the tracker
                // from ever re-locking.  The published map<-odom output is not affected.
                const M3D seed_r = m_gate.engaged() ? m_gate.reference_r() : m_state.last_offset_r;
                const V3D seed_t = m_gate.engaged() ? m_gate.reference_t() : m_state.last_offset_t;
                initial_guess.block<3, 3>(0, 0) = (seed_r * frame_local_r).cast<float>();
                initial_guess.block<3, 1>(0, 3) = (seed_r * frame_local_t + seed_t).cast<float>();
            }
        }

        m_localizer->setInput(frame_cloud);
        const auto align_t0 = std::chrono::steady_clock::now();
        const bool result = m_localizer->align(initial_guess);
        m_state.align_ms_total = m_state.align_ms_total.load() +
                                 std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - align_t0).count();
        if (!result)
        {
            // No measurement for this frame: no sample is published for it at all, so the
            // consumer holds the previous one and its age tells the truth.  The freshness bound
            // of the lock answer is NOT renewed here either: a failed attempt is no evidence
            // about the lock, and renewing on it let a stream of failures keep a dead lock
            // answering "valid" forever (see the corroborated() renewal below).
            ++m_state.icp_failures;
            return;
        }
        ++m_state.updates;

        M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
        V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
        M3D cand_offset_r = map_body_r * frame_local_r.transpose();
        V3D cand_offset_t = -map_body_r * frame_local_r.transpose() * frame_local_t + map_body_t;

        // The gate owns the whole lock lifecycle (see outlier_gate.h).  It is entered on
        // EVERY successful ICP result: the only unconditional-adopt cases are the first lock and
        // an operator relocalize request - the first lock only seeds the tracking reference (it
        // neither publishes nor validates anything), and neither case means "the lock is
        // currently degraded", which would leave the gate bypassed forever (review NB1).
        OffsetGateOutcome outcome;
        bool published_moved = false;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            const M3D prev_r = m_gate.published_r();
            const V3D prev_t = m_gate.published_t();
            outcome = m_gate.update(cand_offset_r, cand_offset_t, frame_local_t, service_request);
            m_state.last_offset_r = m_gate.published_r();
            m_state.last_offset_t = m_gate.published_t();
            // The published correction is frozen while the lock is degraded, and a REJECTED
            // candidate never becomes the published value.  Emitting a sample anyway would
            // re-stamp that frozen value with this frame's stamp, i.e. it would fabricate
            // freshness - the exact defect this node must not have.  So a sample is emitted
            // exactly when the published map<-odom correction actually moved (adopt, accepted
            // EMA update, or the recovery snap); otherwise the consumer keeps holding the older
            // sample and the recorder measures its age.
            // The FIRST valid lock publishes one sample even if the correction is exactly the
            // identity, so a consumer that starts later still learns the lock exists; afterwards
            // only a published correction that actually MOVED is stamped.
            const bool first_valid_lock = !m_state.valid_correction_published && m_gate.valid();
            if (first_valid_lock)
                m_state.valid_correction_published = true;
            published_moved =
                first_valid_lock || (m_gate.published_t() - prev_t).norm() > 1e-9 ||
                Eigen::Quaterniond(prev_r).angularDistance(
                    Eigen::Quaterniond(m_gate.published_r())) > 1e-9;
            // Renew the freshness bound of the relocalize_check answer ONLY for a corroborated
            // correction: an accepted candidate under a valid lock, an explicit relock, or a
            // completed (first or post-lost) qualification.  A failed ICP attempt, a rejected
            // candidate and a not-yet-qualified first lock renew nothing, so they expire into
            // "invalid" instead of keeping a dead lock looking alive.
            if (outcome.corroborated())
                m_state.validity.onUpdate(this->now().seconds());
            if (service_request)
                m_state.service_received = false;
        }
        if (service_request)
        {
            RCLCPP_INFO(this->get_logger(), "localization lock adopted at t=[%.3f %.3f %.3f]",
                        m_state.last_offset_t.x(), m_state.last_offset_t.y(),
                        m_state.last_offset_t.z());
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
        // exactly one sample per MOVED published correction, stamped with the frame it came from
        if (published_moved)
        {
            sendBroadCastTF(frame_time);
            publishMapCloud(frame_time);
        }
    }
    void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        m_state.pairs.addCloud(toSeconds(cloud_msg->header.stamp), cloud_msg);
        ++m_state.clouds_received;
    }

    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        const M3D r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                         odom_msg->pose.pose.orientation.x,
                                         odom_msg->pose.pose.orientation.y,
                                         odom_msg->pose.pose.orientation.z)
                          .toRotationMatrix();
        const V3D t(odom_msg->pose.pose.position.x,
                    odom_msg->pose.pose.position.y,
                    odom_msg->pose.pose.position.z);
        m_state.pairs.addOdom(toSeconds(odom_msg->header.stamp), std::make_pair(r, t));
        m_state.odom_frame_id = odom_msg->header.frame_id;
        ++m_state.odoms_received;
    }

    // Newest scan stamp present in BOTH stores, with the odometry nearest to it within
    // kPairToleranceS, plus the raw cloud message.  NO cloud conversion happens here: this runs
    // on every timer tick, and pcl::fromROSMsg of a full scan at 100 Hz would burn exactly the
    // CPU the ICP needs.  Self-healing by construction: nothing is queued across calls, so a
    // slow or blocked executor costs missed frames but can never wedge the intake.
    bool newestPair(double &stamp, sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                    M3D &r, V3D &t)
    {
        typename FramePairCache<sensor_msgs::msg::PointCloud2::ConstSharedPtr,
                                std::pair<M3D, V3D>>::Pair pair;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (!m_state.pairs.newest(kPairToleranceS, pair))
                return false;
            if (m_state.sync_pairs.load() == 0)
                // first pair: the odometry frame the localizer publishes map<-odom against
                m_config.local_frame = m_state.odom_frame_id;
        }
        stamp = pair.stamp;
        cloud_msg = pair.cloud;
        r = pair.pose.first;
        t = pair.pose.second;
        // count DISTINCT paired frames: this is the intake-health signal (it grows at the
        // stream rate and freezes if pairing ever stops), not a count of timer ticks
        if (stamp != m_state.last_paired_stamp)
        {
            m_state.last_paired_stamp = stamp;
            ++m_state.sync_pairs;
        }
        return true;
    }

    void sendBroadCastTF(const builtin_interfaces::msg::Time &time)
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
        // The request's `code` field is kept for message compatibility and is IGNORED: every
        // code answers from the same state machine.  code==1 used to return valid=true
        // unconditionally, which let any caller assert a lock that no ICP update backed - the
        // same defect class (an answer with no evidence behind it) the freshness bound below
        // exists to prevent, and it bypassed the operator relocalize too.
        (void)request;
        // The gate only changes when an ICP update arrives, so its state alone is stale
        // evidence: an intake that stopped delivering frames (dead subscription, wedged map,
        // sync policy that no longer pairs) used to keep answering valid forever.  The lock is
        // reported valid only while the last CORROBORATED correction is younger than
        // validity_timeout_s (see the renewal in timerCB).
        response->valid = m_state.validity.valid(m_gate.valid(), this->now().seconds(),
                                                m_config.validity_timeout_s);
        return;
    }
    void publishMapCloud(const builtin_interfaces::msg::Time &time)
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
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