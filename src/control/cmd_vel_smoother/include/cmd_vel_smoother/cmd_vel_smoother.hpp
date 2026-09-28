#ifndef CMD_VEL_SMOOTHER_HPP
#define CMD_VEL_SMOOTHRT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

namespace cmd_vel_smoother
{
    class Smoother:public rclcpp::Node
    {
        public:
           explicit Smoother(double alpha = 0.2);
        private:
           void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

           rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
           rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
           double alpha_;


           double prev_vx_;
           double prev_vy_;
           double prev_vyaw_;

           
    };

}

#endif