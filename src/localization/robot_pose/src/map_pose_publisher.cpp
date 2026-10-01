#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "interface/srv/is_valid.hpp"

#include "map_pose_current.hpp"
#include "map_pose_stamp.hpp"

class MapPosePublisher : public rclcpp::Node
{
public:
    MapPosePublisher() : Node("map_pose_publisher")
    {
        // TF2 相关初始化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // EXPLICIT, OPT-IN current-pose mode.  Default false: the node keeps publishing the chain
        // tf2 resolves (truthful resolved stamp).  When true it publishes the last accepted map
        // correction propagated over the newest RECEIVED odometry, stamped with the odometry's own
        // measurement time - never the node clock - and reports the correction's age/validity on a
        // diagnostic status topic.  No extrapolation, no future transform: stale odometry is
        // unavailable.
        this->declare_parameter<bool>("current_pose_mode", false);
        this->declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
        this->declare_parameter<double>("odom_age_limit_s", 1.0);
        this->declare_parameter<double>("correction_validity_timeout_s", 1.0);
        this->get_parameter("current_pose_mode", current_pose_mode_);
        std::string odom_topic;
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("odom_age_limit_s", cfg_.odom_age_limit_s);
        this->get_parameter("correction_validity_timeout_s", cfg_.correction_validity_timeout_s);

        // BOUNDED prediction, only meaningful for the current-pose service.  The bounds are the
        // pre-registered short window (see map_pose_current.hpp); they are parameters so the
        // experiment can record which ones ran, never so an acceptance case can be widened.
        this->declare_parameter<bool>("predict_current_pose", false);
        this->declare_parameter<double>("max_prediction_s", cfg_.max_prediction_s);
        this->declare_parameter<double>("min_odom_interval_s", cfg_.min_odom_interval_s);
        this->declare_parameter<double>("max_odom_interval_s", cfg_.max_odom_interval_s);
        this->get_parameter("predict_current_pose", predict_current_pose_);
        this->get_parameter("max_prediction_s", cfg_.max_prediction_s);
        this->get_parameter("min_odom_interval_s", cfg_.min_odom_interval_s);
        this->get_parameter("max_odom_interval_s", cfg_.max_odom_interval_s);

        if (predict_current_pose_ && !current_pose_mode_)
        {
            RCLCPP_FATAL(this->get_logger(),
                         "predict_current_pose=true requires current_pose_mode=true: prediction is a "
                         "current-pose service, a stamp/legacy mode has nothing to predict for");
            throw std::invalid_argument(
                "predict_current_pose=true requires current_pose_mode=true");
        }
        if (!(cfg_.max_prediction_s >= 0.0) || !(cfg_.min_odom_interval_s > 0.0) ||
            !(cfg_.max_odom_interval_s >= cfg_.min_odom_interval_s))
        {
            RCLCPP_FATAL(this->get_logger(),
                         "invalid prediction bounds (max_prediction_s=%f, min_odom_interval_s=%f, "
                         "max_odom_interval_s=%f): refusing to start",
                         cfg_.max_prediction_s, cfg_.min_odom_interval_s, cfg_.max_odom_interval_s);
            throw std::invalid_argument("invalid prediction interval parameters");
        }

        // 创建发布器
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_pose_map",
            rclcpp::QoS(10));
        status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/robot_pose_map/status", rclcpp::QoS(10));
        if (current_pose_mode_)
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, rclcpp::SensorDataQoS(),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });
            // Validate the gate: the localizer's own lock-validity answer is the authoritative
            // "is this correction trustworthy" signal, and current_pose_mode is STRICT by default -
            // a correction the gate has not vouched for is never propagated silently.
            try {
                using IsValid = interface::srv::IsValid;
                gate_client_ = this->create_client<IsValid>("/localizer/relocalize_check");
            } catch (...) {
                gate_client_ = nullptr;
            }
        }

        // 创建定时器，定期发布位姿
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&MapPosePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(),
                    "Map Pose Publisher 已启动 (current_pose_mode=%s, predict_current_pose=%s)",
                    current_pose_mode_ ? "true" : "false",
                    predict_current_pose_ ? "true" : "false");
    }

private:
    // One query tick's status payload (predict mode only; the legacy modes pass nullptr).
    struct QueryStatus
    {
        uint64_t seq = 0;
        int64_t stamp_ns = 0;
        bool predicted = false;
        double prediction_dt_s = -1.0;
        bool odom_ready = false;
        bool tf_ready = false;
        std::string reject_reason;
    };

    static double monotonicSeconds()
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static builtin_interfaces::msg::Time nsToTime(int64_t ns)
    {
        builtin_interfaces::msg::Time t;
        int64_t sec = ns / 1000000000LL;
        int64_t nsec = ns % 1000000000LL;
        if (nsec < 0) { nsec += 1000000000LL; --sec; }
        t.sec = static_cast<int32_t>(sec);
        t.nanosec = static_cast<uint32_t>(nsec);
        return t;
    }

    // A number that survives being written into a JSON document: the diagnostic KeyValue is a
    // string, and a NaN/Inf literal is NOT legal JSON, so every non-finite value becomes "n/a".
    static std::string numOrNa(double v)
    {
        return std::isfinite(v) ? std::to_string(v) : std::string("n/a");
    }

    // Odometry arrives here.  The legacy current mode only needs the newest sample; the predicting
    // mode keeps AT MOST TWO strictly increasing samples (previous, latest) so the angular rate has
    // a window, ignores repeated stamps, and treats an out-of-order sample / ROS-time jump-back as
    // an epoch change: every cached input is dropped and the node re-waits for a fresh stream.
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!predict_current_pose_)
        {
            odom_ = msg;
            return;
        }
        const double t = stampSeconds(msg->header.stamp);
        if (!std::isfinite(t))
            return; // a non-finite stamp can never seed a window
        if (odom_samples_.empty())
        {
            odom_samples_.push_back(msg);
            return;
        }
        const double t_last = stampSeconds(odom_samples_.back()->header.stamp);
        if (t == t_last)
            return; // repeated stamp: ignore, it carries no new information
        if (t < t_last)
        {
            invalidateInputs("out-of-order odometry / ROS time jump-back");
            return; // re-wait for input; the late sample does not seed the new window
        }
        odom_samples_.push_back(msg);
        while (odom_samples_.size() > 2)
            odom_samples_.erase(odom_samples_.begin());
    }

    // Forget every cached input.  For the TF chain a fresh buffer + listener is created instead of
    // calling tf2::BufferCore::clear(): that clears the StaticCache too, and a latched /tf_static
    // sample is never re-delivered to an existing subscription, so the child->base_link lever arm
    // would be lost for the rest of the process.  Recreating the subscription re-acquires the
    // static transform and drops the pre-reset dynamic cache with it.
    void invalidateInputs(const char *reason)
    {
        odom_samples_.clear();
        gate_trust_.reset();
        gate_pending_ = false;
        gate_future_ = std::shared_future<interface::srv::IsValid::Response::SharedPtr>();
        gate_valid_ = false;
        gate_answer_age_ = -1.0;
        tf_listener_.reset();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_WARN(this->get_logger(),
                    "input caches flushed (%s): re-waiting for odometry and TF", reason);
    }

    // Predict mode.  The clock is captured ONCE per tick; only a STRICTLY INCREASING query clock
    // produces a new query, so a paused clock cannot re-publish (or re-reject) a stale query, and a
    // jump-back first flushes every cached input.  Exactly one status is emitted per query and at
    // most one pose - a rejected query still reports its reason under the same seq/stamp.
    void predictQueryCallback()
    {
        const rclcpp::Time now = this->now();
        const int64_t now_ns = now.nanoseconds();
        if (now_ns <= 0)
            return; // ROS time not initialized yet: there is no query to answer

        if (last_query_valid_)
        {
            if (now_ns < last_query_ns_)
            {
                invalidateInputs("ROS time jump-back");
                last_query_ns_ = now_ns; // resync the query epoch; publish nothing this tick
                return;
            }
            if (now_ns == last_query_ns_)
                return; // clock paused: never re-flush a stale query
        }
        last_query_ns_ = now_ns;
        last_query_valid_ = true;

        const builtin_interfaces::msg::Time query_stamp = nsToTime(now_ns);
        QueryStatus qs;
        qs.seq = ++query_seq_;
        qs.stamp_ns = now_ns;

        // The gate answer is evidence only for a bounded time after it was RECEIVED (monotonic).
        pollGate();
        const double mono_s = monotonicSeconds();
        gate_answer_age_ = gate_trust_.age(mono_s);
        const bool gate_valid = gate_trust_.trusted(mono_s, cfg_.correction_validity_timeout_s);
        gate_valid_ = gate_valid;
        if (gate_valid)
            gate_ever_valid_ = true; // first-lock evidence for the post-lock denominator

        double corr_age_s = -1.0;
        double odom_age_s = -1.0;
        bool correction_valid = false;

        const bool odom_ready = odom_samples_.size() >= 2;
        qs.odom_ready = odom_ready;

        geometry_msgs::msg::TransformStamped map_odom;
        bool have_correction = false;
        if (odom_ready)
        {
            try {
                // the latest RECEIVED correction, on the correction link only
                map_odom = tf_buffer_->lookupTransform(cfg_.map_frame,
                                                       odom_samples_.back()->header.frame_id,
                                                       tf2::TimePointZero);
                have_correction = true;
            } catch (tf2::TransformException &) {
            }
        }

        geometry_msgs::msg::TransformStamped child_base;
        bool have_lever = false;
        if (have_correction)
        {
            try {
                // the mounting extrinsic at the EXACT query stamp; a future TF is never consulted
                child_base = tf_buffer_->lookupTransform(odom_samples_.back()->child_frame_id,
                                                         cfg_.base_frame,
                                                         tf2::timeFromSec(stampSeconds(query_stamp)));
                have_lever = true;
            } catch (tf2::TransformException &) {
            }
        }
        // "two chains ready": the odometry pair AND both TF links (correction + lever arm)
        qs.tf_ready = have_correction && have_lever;

        std::string why;
        if (!odom_ready)
            why = map_pose_reason::kOdomNotReady;
        else if (!have_correction)
            why = map_pose_reason::kNoCorrection;
        else if (!have_lever)
            why = map_pose_reason::kNoChildBaseTf;
        else
        {
            const auto r = predictCurrentPose(map_odom, *odom_samples_.front(),
                                              *odom_samples_.back(), child_base, query_stamp, cfg_,
                                              gate_valid, gate_answer_age_);
            corr_age_s = r.correction_age_s;
            odom_age_s = r.odom_age_s;
            correction_valid = r.correction_valid;
            qs.prediction_dt_s = r.prediction_dt_s;
            if (r.available)
            {
                qs.predicted = true;
                qs.reject_reason.clear();
                publishStatus(true, "ok", corr_age_s, odom_age_s, correction_valid, &qs);
                publishPose(query_stamp, r.pose);
                return;
            }
            why = r.reject_reason;
        }
        qs.reject_reason = why;
        publishStatus(false, why, corr_age_s, odom_age_s, correction_valid, &qs);
    }

    // Legacy current-pose mode: propagate the last accepted correction over the newest received
    // odometry.  Semantics are unchanged (odometry stamp, no query counter, no extrapolation).
    void currentPoseCallback()
    {
        if (!odom_)
        {
            publishStatus(false, "no odometry received yet", -1.0, -1.0, false, nullptr);
            return;
        }
        geometry_msgs::msg::TransformStamped map_odom;
        try {
            // the latest RECEIVED correction (TimePointZero on the correction link only)
            map_odom = tf_buffer_->lookupTransform("map", odom_->header.frame_id,
                                                   tf2::TimePointZero);
        } catch (tf2::TransformException &) {
            publishStatus(false, "no map correction received yet", -1.0, -1.0, false, nullptr);
            return;
        }
        // the odometry is odom<-child_frame_id, NOT odom<-base_link: the message's own child frame
        // is carried through and the lever arm to base_link is looked up separately
        geometry_msgs::msg::TransformStamped odom_child;
        odom_child.header = odom_->header;
        odom_child.child_frame_id = odom_->child_frame_id;
        odom_child.transform.translation.x = odom_->pose.pose.position.x;
        odom_child.transform.translation.y = odom_->pose.pose.position.y;
        odom_child.transform.translation.z = odom_->pose.pose.position.z;
        odom_child.transform.rotation = odom_->pose.pose.orientation;

        // child_frame_id <- base_link at the EXACT odometry stamp (the mounting extrinsic; a real
        // quadruped calibration is never identity).  Missing TF -> unavailable, never a body pose
        // silently labelled base_link.
        geometry_msgs::msg::TransformStamped child_base;
        try {
            child_base = tf_buffer_->lookupTransform(odom_->child_frame_id, cfg_.base_frame,
                                                     tf2::timeFromSec(stampSeconds(odom_->header.stamp)));
        } catch (tf2::TransformException &) {
            publishStatus(false, "odometry child frame -> base_link TF unavailable at the odometry stamp",
                          -1.0, -1.0, false, nullptr);
            return;
        }

        pollGate();
        const double now_s = this->now().seconds();          // message clock: pose/odom ages
        const double mono_s = monotonicSeconds();            // monotonic: gate-answer age
        gate_answer_age_ = gate_trust_.age(mono_s);          // derived, never cached as 0
        const bool gate_valid = gate_trust_.trusted(mono_s, cfg_.correction_validity_timeout_s);
        gate_valid_ = gate_valid;
        const auto r = currentPose(map_odom, odom_child, child_base, now_s, cfg_,
                                   gate_valid, gate_answer_age_);
        std::string why = "ok";
        if (!r.available)
        {
            const bool frames_ok = map_odom.header.frame_id == cfg_.map_frame &&
                                   map_odom.child_frame_id == odom_child.header.frame_id &&
                                   odom_child.child_frame_id == child_base.header.frame_id &&
                                   child_base.child_frame_id == cfg_.base_frame;
            why = !frames_ok
                      ? "frame chain does not line up (map<-odom<-child<-base_link)"
                      : (r.odom_age_s < 0.0 || r.odom_age_s > cfg_.odom_age_limit_s
                             ? "odometry older than the declared limit"
                             : (!r.correction_valid
                                    ? "correction older than the declared validity bound"
                                    : "gate has not vouched for the lock"));
        }
        publishStatus(r.available, why, r.correction_age_s, r.odom_age_s, r.correction_valid, nullptr);
        if (!r.available)
            return; // unavailable, never extrapolated
        publishPose(r.stamp, r.pose);
    }

    void publishPose(const builtin_interfaces::msg::Time &stamp,
                     const geometry_msgs::msg::Pose &pose)
    {
        geometry_msgs::msg::PoseStamped pose_map;
        pose_map.header.stamp = stamp;
        pose_map.header.frame_id = cfg_.map_frame;
        pose_map.pose = pose;
        pose_pub_->publish(pose_map);
    }

    // Ask the localizer's own gate, asynchronously, at most every 0.5 s; the answer carries its
    // own MONOTONIC receipt time so a stale answer is not treated as evidence, and a request that
    // never completes cannot extend the trust.
    void pollGate()
    {
        if (!gate_client_)
            return;
        if (gate_pending_ && gate_future_.valid() &&
            gate_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto res = gate_future_.get();
            // the answer is evidence from the moment it was RECEIVED; its age is derived on every
            // query, so a request that never completes cannot keep an old "valid" alive
            gate_trust_.onAnswer(res ? res->valid : false, monotonicSeconds());
            gate_pending_ = false;
        }
        if (gate_pending_ || (this->now() - gate_last_request_).seconds() < 0.5)
            return;
        if (!gate_client_->service_is_ready())
        {
            // fail closed: an unreachable gate is not a valid gate (the stored trust still ages
            // out on its own, so nothing is refreshed here)
            return;
        }
        auto req = std::make_shared<interface::srv::IsValid::Request>();
        req->code = 0;
        gate_future_ = gate_client_->async_send_request(req).future.share();
        gate_pending_ = true;
        gate_last_request_ = this->now();
    }

    // The diagnostic status.  Behavioural/diagnostic keys are always present; the predict-mode
    // query contract (query_seq / query_stamp_ns / predicted / prediction_dt_s / reject_reason and
    // the chain-readiness flags) is added only when `q` is given, so the legacy modes keep exactly
    // the key set their existing recordings captured.
    void publishStatus(bool available, const std::string &message, double correction_age_s,
                       double odom_age_s, bool correction_valid, const QueryStatus *q)
    {
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = "map_pose_publisher";
        st.hardware_id = "localization";
        st.level = available ? diagnostic_msgs::msg::DiagnosticStatus::OK
                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        st.message = message;
        auto kv = [](const std::string &k, const std::string &v) {
            diagnostic_msgs::msg::KeyValue out;
            out.key = k;
            out.value = v;
            return out;
        };
        st.values.push_back(kv("current_pose_mode", current_pose_mode_ ? "true" : "false"));
        st.values.push_back(kv("available", available ? "true" : "false"));
        st.values.push_back(kv("correction_valid", correction_valid ? "true" : "false"));
        st.values.push_back(kv("gate_valid", gate_valid_ ? "true" : "false"));
        st.values.push_back(kv("gate_answer_age_s",
                               gate_answer_age_ < 0 ? "n/a" : numOrNa(gate_answer_age_)));
        st.values.push_back(kv("correction_age_s",
                               correction_age_s < 0 ? "n/a" : numOrNa(correction_age_s)));
        st.values.push_back(kv("odom_age_s",
                               odom_age_s < 0 ? "n/a" : numOrNa(odom_age_s)));
        st.values.push_back(kv("odom_age_limit_s", numOrNa(cfg_.odom_age_limit_s)));
        st.values.push_back(kv("correction_validity_timeout_s",
                               numOrNa(cfg_.correction_validity_timeout_s)));
        if (q)
        {
            st.values.push_back(kv("predict_current_pose", "true"));
            st.values.push_back(kv("query_seq", std::to_string(q->seq)));
            st.values.push_back(kv("query_stamp_ns", std::to_string(q->stamp_ns)));
            st.values.push_back(kv("predicted", q->predicted ? "true" : "false"));
            st.values.push_back(kv("prediction_dt_s",
                                   q->prediction_dt_s < 0 ? "n/a" : numOrNa(q->prediction_dt_s)));
            st.values.push_back(kv("reject_reason", q->reject_reason));
            st.values.push_back(kv("odom_ready", q->odom_ready ? "true" : "false"));
            st.values.push_back(kv("tf_ready", q->tf_ready ? "true" : "false"));
            st.values.push_back(kv("gate_ever_valid", gate_ever_valid_ ? "true" : "false"));
            st.values.push_back(kv("max_prediction_s", numOrNa(cfg_.max_prediction_s)));
            st.values.push_back(kv("min_odom_interval_s", numOrNa(cfg_.min_odom_interval_s)));
            st.values.push_back(kv("max_odom_interval_s", numOrNa(cfg_.max_odom_interval_s)));
        }
        status_pub_->publish(st);
    }

    void timer_callback()
    {
        if (predict_current_pose_)
        {
            predictQueryCallback();
            return;
        }
        if (current_pose_mode_)
        {
            currentPoseCallback();
            return;
        }
        try {
            // 直接查询从 map 到 base_link 的变换
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
                "map",           // 目标坐标系
                "base_link",     // 源坐标系（机器人本体）
                tf2::TimePointZero);
            
            // 将变换转换为 PoseStamped
            geometry_msgs::msg::PoseStamped pose_map;
            // PUBLISH THE RESOLVED STAMP, NOT now().  TimePointZero resolves the chain at its
            // latest COMMON time, bounded by the older of the two transforms (here the localizer's
            // map->odom broadcast, measured ~0.24 s behind the clock).  Stamping that pose with
            // now() claims the robot is where it already left: measured as a 12.6 cm error against
            // the ground truth at the published stamp on a 0.5 m/s route, while the pose itself was
            // accurate to 2.2-2.8 cm for the time it describes.  Publishing the resolved stamp
            // keeps the interface truthful (subscribers can see the true age) and never invents a
            // future pose; a consumer that needs its own clock must compose/extrapolate explicitly
            // under its own accepted-correction contract.
            pose_map.header.stamp = map_pose_stamp(transform);
            pose_map.header.frame_id = "map";
            
            // 位置
            pose_map.pose.position.x = transform.transform.translation.x;
            pose_map.pose.position.y = transform.transform.translation.y;
            pose_map.pose.position.z = transform.transform.translation.z;
            
            // 朝向
            pose_map.pose.orientation = transform.transform.rotation;
            
            // 发布
            pose_pub_->publish(pose_map);
            
        } catch (tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "TF 变换查找失败: %s", ex.what());
        }
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    bool current_pose_mode_ = false;
    bool predict_current_pose_ = false;
    CurrentPoseConfig cfg_;
    std::vector<nav_msgs::msg::Odometry::SharedPtr> odom_samples_; // previous, latest (predict)
    uint64_t query_seq_ = 0;
    int64_t last_query_ns_ = 0;
    bool last_query_valid_ = false;
    bool gate_ever_valid_ = false; // latched first-lock evidence; never reset
    rclcpp::Client<interface::srv::IsValid>::SharedPtr gate_client_;
    std::shared_future<interface::srv::IsValid::Response::SharedPtr> gate_future_;
    bool gate_pending_ = false;
    bool gate_valid_ = false;
    double gate_answer_age_ = -1.0;
    GateTrust gate_trust_;
    rclcpp::Time gate_last_request_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MapPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("map_pose_publisher"), "startup failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
