#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapPosePublisher : public rclcpp::Node
{
public:
    MapPosePublisher() : Node("map_pose_publisher")
    {
        // TF2 相关初始化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建发布器
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_pose_map",
            rclcpp::QoS(10));
        
        // 创建定时器，定期发布位姿
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&MapPosePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Map Pose Publisher 已启动");
    }

private:
    void timer_callback()
    {
        try {
            // 直接查询从 map 到 base_link 的变换
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
                "map",           // 目标坐标系
                "base_link",     // 源坐标系（机器人本体）
                tf2::TimePointZero);
            
            // 将变换转换为 PoseStamped
            geometry_msgs::msg::PoseStamped pose_map;
            pose_map.header.stamp = this->now();
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
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}