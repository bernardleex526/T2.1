#pragma once
// The published stamp of a map pose: the stamp of the transform tf2 actually resolved.
//
// TimePointZero resolves the chain at its latest COMMON time, so the transform carries a PAST
// stamp.  Publishing that value verbatim is what makes the interface truthful (a subscriber can
// see the age) and avoids claiming the robot is at a pose it has already left.  Measured on the
// T3 route: stamping the same pose with the node clock instead produced a 12.6 cm error against
// the ground truth at the published stamp (0.242 s of travel at 0.5 m/s), while the pose was
// accurate to 2.2-2.8 cm for the time it described.  Kept as a function so the rule is testable.
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

inline builtin_interfaces::msg::Time map_pose_stamp(
    const geometry_msgs::msg::TransformStamped &transform)
{
    return transform.header.stamp;
}
