// Regression for the map_pose_publisher interface defect measured on 2026-09-30.
//
// The node resolved the chain with tf2::TimePointZero - which returns the transform at the chain's
// latest COMMON time, bounded by the older of the two transforms (the localizer's map->odom
// broadcast, measured ~0.24 s behind the clock) - and then stamped the published pose with
// this->now().  On a 0.5 m/s route that claims the robot is ~12 cm further along than the pose
// says: the measured /robot_pose_map error against the ground truth AT THE PUBLISHED STAMP was
// 0.1263 m, while the pose was accurate to 0.0223-0.0284 m for the time it actually described.
//
// The interface rule these tests pin:
//   * tf2 TimePointZero returns the transform with its OWN (past) stamp, not the query time;
//   * the node must publish that stamp verbatim (mapPoseStamp), so subscribers can see the true
//     age, and must never substitute the node clock.
#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "map_pose_stamp.hpp"

namespace
{
geometry_msgs::msg::TransformStamped makeTransform(const std::string &frame,
                                                   const std::string &child,
                                                   double stamp_s)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = frame;
    t.child_frame_id = child;
    t.header.stamp.sec = static_cast<int32_t>(stamp_s);
    t.header.stamp.nanosec = static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1e9);
    t.transform.translation.x = 1.0;
    t.transform.rotation.w = 1.0;
    return t;
}
} // namespace

// A known PAST transform must come back with its exact stamp, and the node must publish that stamp.
TEST(MapPoseStamp, KnownPastStampIsPreservedExactly)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("map_pose_stamp_test");
    tf2_ros::Buffer buffer(node->get_clock());
    const double past = 1000.25;
    buffer.setTransform(makeTransform("map", "odom", past), "test");
    buffer.setTransform(makeTransform("odom", "base_link", past), "test");

    const auto res = buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(res.header.stamp.sec + res.header.stamp.nanosec * 1e-9, past);
    EXPECT_EQ(res.header.frame_id, "map");

    // what the node publishes: the transform's own stamp, verbatim
    const auto out = map_pose_stamp(res);
    EXPECT_EQ(out.sec, res.header.stamp.sec);
    EXPECT_EQ(out.nanosec, res.header.stamp.nanosec);
    // and NOT the node's current time
    const double now_s = node->now().seconds();
    EXPECT_NE(out.sec + out.nanosec * 1e-9, now_s);
    EXPECT_LT(out.sec + out.nanosec * 1e-9, now_s);

    node.reset();
    rclcpp::shutdown();
}
