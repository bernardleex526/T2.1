#include "cmd_vel_smoother/cmd_vel_smoother.hpp"

namespace cmd_vel_smoother
{

    Smoother::Smoother(double alpha)
        : Node("cmd_vel_smoother"),
          alpha_(alpha), 
          prev_vx_(0.0), 
          prev_vy_(0.0), 
          prev_vyaw_(0.0)
    {
        // 创建发布者和订阅者
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smooth", 10);
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Smoother::cmdCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cmd_vel_smoother node started with alpha: %f", alpha_);
    }

    // 平滑处理
    void Smoother::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double target_vx = msg->linear.x;
        double target_vy = msg->linear.y;
        double target_vyaw = msg->angular.z;

        double vx = alpha_ * target_vx + (1.0 - alpha_) * prev_vx_;
        double vy = alpha_ * target_vy + (1.0 - alpha_) * prev_vy_;
        double vyaw = alpha_ * target_vyaw + (1.0 - alpha_) * prev_vyaw_;

        prev_vx_ = vx;
        prev_vy_ = vy;
        prev_vyaw_ = vyaw;

        geometry_msgs::msg::Twist smoothed;
        smoothed.linear.x = vx;
        smoothed.linear.y = vy;
        smoothed.angular.z = vyaw;

        pub_->publish(smoothed);
    }

} // namespace cmd_vel_smoother

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 创建平滑器节点，可以传入alpha参数
    auto smoother = std::make_shared<cmd_vel_smoother::Smoother>(0.2); // alpha=0.2
    
    rclcpp::spin(smoother);
    rclcpp::shutdown();
    
    return 0;
}