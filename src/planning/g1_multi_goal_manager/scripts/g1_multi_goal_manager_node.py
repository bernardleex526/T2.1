#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import yaml
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal

from g1_multi_goal_manager.srv import (
    ClearGoals, ListGoals, DeleteGoal, UpdateGoalName,
    StopNavigation, GetCurrentLocation
)
from g1_multi_goal_manager.msg import GoalInfo

DIST_THRESH = 0.2

class G1MultiGoalManager(Node):
    def __init__(self):
        super().__init__('g1_multi_goal_manager')
        
        # 全局状态变量
        self.goal_queue = []
        self.waiting_for_robot = False
        self.current_pose = None
        self.goal_counter = 1
        self.goal_file_path = ""
        self.MAP = None
        self.paused = False
        self.navigation_mode = "single"  # "single" 或 "sequence"
        self.current_index = 0
        self.auto_navigation = False

        # 发布器
        self.nav_goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # 订阅器
        self.create_subscription(PoseStamped, "/multi_nav/goal", self.rviz_goal_callback, 10)
        self.create_subscription(Int32, "/navigation_start", self.navigation_start_callback, 10)
        self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)
        self.create_subscription(PoseStamped, "/robot_pose_map", self.current_pose_callback, 10)

        # 服务
        self.create_service(ClearGoals, "/multi_nav/clear", self.clear_goals_service)
        self.create_service(ListGoals, "/multi_nav/list", self.list_goals_service)
        self.create_service(DeleteGoal, "/multi_nav/delete", self.delete_goal_service)
        self.create_service(UpdateGoalName, "/multi_nav/update_name", self.update_goal_name_service)
        self.create_service(StopNavigation, "/multi_nav/pause", self.pause_navigation_service)
        self.create_service(StopNavigation, "/multi_nav/resume", self.resume_navigation_service)
        self.create_service(GetCurrentLocation, "/nav/location", self.get_current_location_service)

        # 取消导航用的客户端(只创建一次，避免每次回调泄漏)。
        # nav2 取消：NavigateToPose action 取消服务，goal_id 全 0 表示取消该 action 所有活动目标
        self.nav2_cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel')
        # 控制接口取消：G1 运动网关
        self.control_cancel_client = self.create_client(SetBool, '/goal_cancel')

        # 获取参数
        self.declare_parameter('map', '')
        self.MAP = self.get_parameter('map').get_parameter_value().string_value
        
        if not self.MAP:
            self.get_logger().error("[multi_goal_manager] Missing required parameter: map")
            return

        # 数据根目录可通过环境变量 FLYOS_DATA_DIR 配置，默认与原硬编码路径一致(便于迁移到其他机器/用户)
        data_dir = os.environ.get("FLYOS_DATA_DIR", "/home/flyos-universe-ros2/data")
        self.goal_file_path = os.path.join(data_dir, self.MAP, "goals.yaml")
        self.load_goals_from_file(self.goal_file_path)
        self.get_logger().info("[multi_goal_manager] Node started, waiting for NAVIGATION POINTS and SERVICE calls")

    def quaternion_to_euler(self, quaternion):
        """
        将四元数转换为欧拉角 (roll, pitch, yaw)
        输入: geometry_msgs.msg.Quaternion 对象
        返回: (roll, pitch, yaw) 弧度值
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # 计算roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # 计算pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # 使用90度，如果超出范围
        else:
            pitch = math.asin(sinp)

        # 计算yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def pose_to_dict(self, pose):
        return {
            "header": {
                "frame_id": pose.header.frame_id
            },
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w
            }
        }

    def dict_to_pose(self, d):
        pose = PoseStamped()
        pose.header.frame_id = d.get("header", {}).get("frame_id", "")
        pose.pose.position.x = d["position"]["x"]
        pose.pose.position.y = d["position"]["y"]
        pose.pose.position.z = d["position"]["z"]
        pose.pose.orientation.x = d["orientation"]["x"]
        pose.pose.orientation.y = d["orientation"]["y"]
        pose.pose.orientation.z = d["orientation"]["z"]
        pose.pose.orientation.w = d["orientation"]["w"]
        return pose

    def load_goals_from_file(self, filepath):
        if os.path.exists(filepath):
            with open(filepath, "r") as f:
                data = yaml.safe_load(f)
                if data and "goals" in data:
                    for item in data["goals"]:
                        self.goal_queue.append({
                            "goal_id": item["goal_id"],
                            "name": item["name"],
                            "pose": self.dict_to_pose(item["pose"])
                        })
                        self.goal_counter = max(self.goal_counter, item["goal_id"])
                    self.get_logger().info(f"[multi_goal_manager] Loaded {len(self.goal_queue)} goals from {filepath}")
                else:
                    self.get_logger().info(f"[multi_goal_manager] File {filepath} exists but contains no goals")
        else:
            self.get_logger().info(f"[multi_goal_manager] Goal file {filepath} not found, creating a new empty one with initial point.")
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            # 创建初始点数据
            initial_goal = {
                "goal_id": 1,
                "name": "初始点",
                "pose": {
                    "header": { "frame_id": "map" },
                    "orientation": {
                        "w": 1.0,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0
                    },
                    "position": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0
                    }
                }
            }
            
            # 将初始点添加到文件
            with open(filepath, "w") as f:
                yaml.dump({"goals": [initial_goal]}, f)
                
            # 将初始点添加到goal_queue
            self.goal_queue.append({
                "goal_id": initial_goal["goal_id"],
                "name": initial_goal["name"],
                "pose": self.dict_to_pose(initial_goal["pose"])
            })
            self.goal_counter = initial_goal["goal_id"]
            self.get_logger().info(f"[multi_goal_manager] Added initial goal to queue")

    def save_goals_to_file(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as f:
            yaml.dump({
                "goals": [{
                    "goal_id": item["goal_id"],
                    "name": item["name"],
                    "pose": self.pose_to_dict(item["pose"])
                } for item in self.goal_queue]
            }, f)
        self.get_logger().info(f"[multi_goal_manager] Saved {len(self.goal_queue)} goals to {filepath}")

    def rviz_goal_callback(self, msg):
        self.goal_counter += 1
        name = f"导航点{self.goal_counter}"
        goal = {
            "goal_id": self.goal_counter,
            "pose": msg,
            "name": name
        }
        self.goal_queue.append(goal)
        
        # 使用自定义的四元数转欧拉角函数
        _, _, yaw = self.quaternion_to_euler(msg.pose.orientation)
        
        self.get_logger().info(f"[multi_goal_manager] Receive navigation point #{self.goal_counter}: "
                              f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, yaw={yaw:.3f}), name: {name}")
        if self.goal_file_path:
            self.save_goals_to_file(self.goal_file_path)

    def navigation_start_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn("[multi_goal_manager] Localization UNKNOWN!")
            return

        data = msg.data

        # 模式解析
        if data >= 0:
            # 单点模式
            self.navigation_mode = "single"
            self.auto_navigation = False
            target_id = data
        elif data == -1:
            # 顺序模式，从第一个点开始
            self.navigation_mode = "sequence"
            self.auto_navigation = True
            target_id = self.goal_queue[0]["goal_id"] if self.goal_queue else -1
        elif data < -1:
            # 顺序模式，从第N个点开始（data = -N）
            self.navigation_mode = "sequence"
            self.auto_navigation = True
            start_index = abs(data) - 1
            if 0 <= start_index < len(self.goal_queue):
                target_id = self.goal_queue[start_index]["goal_id"]
            else:
                self.get_logger().warn(f"[multi_goal_manager] Invalid start index: {-data}")
                return
        else:
            self.get_logger().warn(f"[multi_goal_manager] Invalid data: {data}")
            return

        # 查找目标点索引
        matched_index = next((i for i, g in enumerate(self.goal_queue) if g["goal_id"] == target_id), -1)

        if matched_index != -1:
            self.current_index = matched_index
            goal = self.goal_queue[self.current_index]["pose"]
            dx = goal.pose.position.x - self.current_pose.pose.position.x
            dy = goal.pose.position.y - self.current_pose.pose.position.y
            dist = math.hypot(dx, dy)

            # 使用自定义的四元数转欧拉角函数
            _, _, yaw = self.quaternion_to_euler(goal.pose.orientation)

            if dist < DIST_THRESH:
                self.get_logger().info(f"[multi_goal_manager] Navigation goal_id={target_id} is close ({dist:.3f}m), SKIPPED")
                self.waiting_for_robot = False
                # 不再发布 goal_reached，而是直接调用到达处理逻辑
                self.handle_goal_skipped()
            else:
                self.get_logger().info(f"[multi_goal_manager] Starting navigation at ID={target_id}, Mode={self.navigation_mode}")
                self.nav_goal_pub.publish(goal)
                self.waiting_for_robot = True
        else:
            self.get_logger().warn(f"[multi_goal_manager] Input goal_id={target_id} is invalid (not found in goal list)")

    def handle_goal_skipped(self):
        """处理跳过近距离航点的逻辑"""
        # 当航点距离很近被跳过时，模拟到达行为
        if self.navigation_mode == "sequence" and self.auto_navigation:
            if self.paused:
                self.get_logger().info("[multi_goal_manager] Navigation is paused, skipping current goal.")
                return
            if self.current_index + 1 < len(self.goal_queue):
                self.current_index += 1
                next_goal = self.goal_queue[self.current_index]
                self.get_logger().info(f"[multi_goal_manager] Auto navigating to next point: ID={next_goal['goal_id']}, Name={next_goal['name']}")
                self.nav_goal_pub.publish(next_goal["pose"])
                self.waiting_for_robot = True
            else:
                self.get_logger().info("[multi_goal_manager] Sequence navigation finished")
                self.auto_navigation = False

    def goal_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info("[multi_goal_manager] Navigation point REACHED")
            self.waiting_for_robot = False

            if self.navigation_mode == "sequence" and self.auto_navigation:
                if self.paused:
                    self.get_logger().info("[multi_goal_manager] Navigation is paused, waiting for resume.")
                    return
                if self.current_index + 1 < len(self.goal_queue):
                    self.current_index += 1
                    next_goal = self.goal_queue[self.current_index]
                    self.get_logger().info(f"[multi_goal_manager] Auto navigating to next point: ID={next_goal['goal_id']}, Name={next_goal['name']}")
                    self.nav_goal_pub.publish(next_goal["pose"])
                    self.waiting_for_robot = True
                else:
                    self.get_logger().info("[multi_goal_manager] Sequence navigation finished")
                    self.auto_navigation = False

    def current_pose_callback(self, msg):
        self.current_pose = msg

    # 服务回调函数
    def clear_goals_service(self, request, response):
        self.goal_queue.clear()
        self.waiting_for_robot = False
        self.get_logger().info("[multi_goal_manager] All navigation points CLEARED")
        if self.goal_file_path:
            self.save_goals_to_file(self.goal_file_path)
        response.success = True
        response.message = "All goals cleared."
        return response

    def list_goals_service(self, request, response):
        if not self.goal_queue:
            self.get_logger().info("[multi_goal_manager] No navigation point")
            return response
        
        self.get_logger().info("[multi_goal_manager] Current navigation points:")
        for i, item in enumerate(self.goal_queue, start=1):
            name = item["name"]
            gid = item["goal_id"]
            self.get_logger().info(f"  {i}: ID={gid}, {name}")
            goal_info = GoalInfo()
            goal_info.goal_id = gid
            goal_info.name = name
            response.goals.append(goal_info)
        return response

    def delete_goal_service(self, request, response):
        target_id = request.index

        index_to_delete = next((i for i, g in enumerate(self.goal_queue) if g["goal_id"] == target_id), -1)

        if index_to_delete != -1:
            removed = self.goal_queue.pop(index_to_delete)
            pose = removed["pose"]
            name = removed["name"]
            gid = removed["goal_id"]
            position = pose.pose.position
            self.get_logger().info(f"[multi_goal_manager] Deleted goal_id={gid} ({name}) "
                                  f"({position.x:.2f}, {position.y:.2f})")
            if self.goal_file_path:
                self.save_goals_to_file(self.goal_file_path)
            response.success = True
            response.message = "Deleted successfully."
        else:
            msg = f"Delete FAILED: goal_id {target_id} not found in the list"
            self.get_logger().warn(f"[multi_goal_manager] {msg}")
            response.success = False
            response.message = msg
        return response

    def update_goal_name_service(self, request, response):
        target_id = request.index

        for i, g in enumerate(self.goal_queue):
            if g["goal_id"] == target_id:
                old_name = g["name"]
                self.goal_queue[i]["name"] = request.new_name
                self.get_logger().info(f"[multi_goal_manager] Renamed goal_id={target_id} from \"{old_name}\" to \"{request.new_name}\"")
                if self.goal_file_path:
                    self.save_goals_to_file(self.goal_file_path)
                response.success = True
                response.message = "Name updated successfully."
                return response

        msg = f"Update FAILED: goal_id {target_id} not found in the list"
        self.get_logger().warn(f"[multi_goal_manager] {msg}")
        response.success = False
        response.message = msg
        return response

    def pause_navigation_service(self, request, response):
        self.paused = True
        self.waiting_for_robot = False
        # 同时取消 nav2 当前导航目标与控制接口的目标。
        try:
            # 1. 取消 nav2 导航：调用 NavigateToPose action 取消服务，
            #    goal_id 全 0 的 CancelGoal 请求会取消该 action server 上的全部活动目标。
            if self.nav2_cancel_client.wait_for_service(timeout_sec=1.0):
                cancel_req = CancelGoal.Request()
                cancel_req.goal_info.goal_id.uuid = [0] * 16  # 全 0 = 取消全部活动目标
                self.nav2_cancel_client.call_async(cancel_req)
            else:
                self.get_logger().warn("[multi_goal_manager] /navigate_to_pose/_action/cancel 不可用，nav2 当前目标可能未被取消")

            # 2. 取消控制接口(G1 运动网关立即停止运动)
            if self.control_cancel_client.wait_for_service(timeout_sec=1.0):
                control_req = SetBool.Request()
                control_req.data = True
                self.control_cancel_client.call_async(control_req)
            else:
                self.get_logger().warn("[multi_goal_manager] /goal_cancel 不可用，控制节点目标可能未被取消")

        except Exception as e:
            self.get_logger().warn(f"[multi_goal_manager] Error during pause: {str(e)}")

        self.get_logger().info("[multi_goal_manager] Navigation PAUSED")
        response.success = True
        response.message = "Navigation paused."
        return response

    def resume_navigation_service(self, request, response):
        if not self.paused:
            response.success = False
            response.message = "Navigation is not paused."
            return response

        self.paused = False
        if self.navigation_mode == "sequence" and self.auto_navigation and self.current_index < len(self.goal_queue):
            next_goal = self.goal_queue[self.current_index]
            self.get_logger().info(f"[multi_goal_manager] Resuming navigation to: ID={next_goal['goal_id']}, Name={next_goal['name']}")
            self.nav_goal_pub.publish(next_goal["pose"])
            self.waiting_for_robot = True
            response.success = True
            response.message = "Navigation resumed."
        else:
            response.success = False
            response.message = "Cannot resume, invalid state."
        return response

    def get_current_location_service(self, request, response):
        if self.current_pose is None:
            self.get_logger().warn("[multi_goal_manager] Current pose is unknown!")
            response.x = 0.0
            response.y = 0.0
            response.yaw = 0.0
            response.goal_id = -1
            response.name = ""
            return response
        
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        # 使用自定义的四元数转欧拉角函数
        _, _, yaw = self.quaternion_to_euler(self.current_pose.pose.orientation)

        closest_goal_id = -1
        closest_name = ""
        
        for g in self.goal_queue:
            gx = g["pose"].pose.position.x
            gy = g["pose"].pose.position.y
            dist = math.hypot(gx - x, gy - y)
            if dist < DIST_THRESH:
                closest_goal_id = g["goal_id"]
                closest_name = g["name"]
                break

        response.x = x
        response.y = y
        response.yaw = yaw
        response.goal_id = closest_goal_id
        response.name = closest_name
        return response


def main(args=None):
    rclpy.init(args=args)
    node = G1MultiGoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()