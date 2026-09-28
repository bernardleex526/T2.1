#!/usr/bin/env python3

"""
宇树 G1机器人 运动控制节点 - ROS2版本（严格状态机）
新增功能：根据 spin 恢复行为的开始次数切换速度源
- 第1次 spin 开始 → 切换到 /cmd_vel_test
- 当 /cmd_vel_smooth 的线速度 x > 0.2 时 → 切换回 /cmd_vel_smooth，重置计数器
- 新目标 → 重置计数器，强制使用 /cmd_vel_smooth
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatusArray, GoalStatus

def quaternion_to_yaw(q):
    """四元数到偏航角(yaw)转换"""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class G1MovementGate(Node):
    def __init__(self):
        super().__init__("g1_movement_gate")
        
        # 参数声明与读取
        self.declare_parameter('dist_thresh', 0.15)
        self.declare_parameter('yaw_thresh', 0.20)
        self.declare_parameter('yaw_align_speed', 1.20)
        self.declare_parameter('stable_count_req', 3)
        self.declare_parameter('yaw_align_count_req', 3)
        self.declare_parameter('lan_port', 'eth0')
        self.declare_parameter('min_effective_yaw', 0.30)  # 最小有效角速度
        
        self.DIST_THRESH = self.get_parameter('dist_thresh').value
        self.YAW_THRESH = self.get_parameter('yaw_thresh').value
        self.YAW_ALIGN_SPEED = self.get_parameter('yaw_align_speed').value
        self.STABLE_COUNT_REQ = self.get_parameter('stable_count_req').value
        self.YAW_ALIGN_COUNT_REQ = self.get_parameter('yaw_align_count_req').value
        self.MIN_EFFECTIVE_YAW = self.get_parameter('min_effective_yaw').value
        self.lan_port = self.get_parameter('lan_port').value
        
        # 全局变量
        self.sport_client = None
        self.goal_pose = None
        self.current_pose = None
        self.goal_reached_flag = False
        
        # 状态机 - 严格顺序
        self.STATE_GO_TO_GOAL = 0      # 前往目标点阶段
        self.STATE_ALIGN_YAW = 1       # 对齐角度阶段
        self.STATE_GOAL_REACHED = 2    # 目标达成阶段
        self.STATE_STRINGS = {
            self.STATE_GO_TO_GOAL: "GO_TO_GOAL",
            self.STATE_ALIGN_YAW: "ALIGN_YAW",
            self.STATE_GOAL_REACHED: "GOAL_REACHED"
        }
        self.state = self.STATE_GO_TO_GOAL
        self.reach_count = 0
        self.yaw_align_count = 0
        
        # spin 相关状态
        self.spin_active = False          # 当前是否有 spin 正在执行
        self.use_test_cmd_vel = False     # 是否使用 /cmd_vel_test
        self.spin_start_count = 0          # 当前周期内 spin 开始次数
        
        # 初始化Unitree客户端
        self.initialize_unitree_client()
        
        # 发布者与订阅者
        self.goal_pub = self.create_publisher(Bool, "/goal_reached", 10)
        self.state_pub = self.create_publisher(String, "/g1_state", 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        # 原有速度指令订阅
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel_smooth", self.cmd_vel_smooth_callback, 10)
        # 备用速度指令订阅
        self.cmd_vel_test_sub = self.create_subscription(Twist, "/cmd_vel_test", self.cmd_vel_test_callback, 10)
        # spin 恢复状态订阅
        self.spin_status_sub = self.create_subscription(
            GoalStatusArray, '/spin/_action/status', self.spin_status_callback, 10)
        
        self.localization_sub = self.create_subscription(PoseStamped, "/robot_pose_map", self.localization_callback, 10)
        
        # 服务与定时器
        self.cancel_service = self.create_service(SetBool, "/goal_cancel", self.cancel_callback)
        self.timer = self.create_timer(2.0, self.status_timer_callback)
        
        self.get_logger().info("g1_movement_gate节点启动完成")

    def initialize_unitree_client(self):
        """初始化Unitree SDK客户端"""
        try:
            ChannelFactoryInitialize(0, self.lan_port)
            self.sport_client = LocoClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            self.get_logger().info("Unitree客户端初始化完成")
        except Exception as e:
            self.get_logger().error(f"Unitree客户端初始化失败: {e}")

    def move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        """发送运动指令"""
        try:
            self.sport_client.Move(vx, vy, vyaw, continous_move)
            return True
        except Exception as e:
            self.get_logger().error(f"发送运动指令失败: {e}")
            return False

    def stop(self):
        """停止运动并发布目标到达消息"""
        if not self.goal_reached_flag:
            try:
                self.sport_client.StopMove()
            except Exception as e:
                self.get_logger().error(f"停止指令发送失败: {e}")
            self.goal_pub.publish(Bool(data=True))
            self.get_logger().info("目标点到达并完成对齐")
            self.goal_reached_flag = True
            self.state = self.STATE_GOAL_REACHED

    def emergency_stop(self):
        """紧急停止，用于状态切换时的速度清零"""
        try:
            self.sport_client.StopMove()
            return True
        except Exception as e:
            self.get_logger().error(f"紧急停止失败: {e}")
            return False

    def get_yaw_diff(self):
        """计算目标与当前朝向差"""
        if self.goal_pose is None or self.current_pose is None:
            return 0.0
        goal_yaw = quaternion_to_yaw(self.goal_pose.pose.orientation)
        curr_yaw = quaternion_to_yaw(self.current_pose.pose.orientation)
        return math.atan2(math.sin(goal_yaw - curr_yaw), math.cos(goal_yaw - curr_yaw))

    def has_reached_position(self):
        """判断是否到达位置目标"""
        if self.goal_pose is None or self.current_pose is None:
            return False
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return math.hypot(dx, dy) < self.DIST_THRESH

    def has_reached_yaw(self):
        """判断是否对齐朝向"""
        if self.goal_pose is None or self.current_pose is None:
            return False
        yaw_diff = abs(self.get_yaw_diff())
        return yaw_diff < self.YAW_THRESH

    def check_goal_precondition(self):
        """
        检查目标点预条件：如果当前位置已经在目标范围内，直接进入对齐阶段
        返回：True 表示已到达，False 表示需要移动
        """
        if self.goal_pose is None or self.current_pose is None:
            return False
            
        # 检查位置是否已在范围内
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        dist = math.hypot(dx, dy)
        
        if dist < self.DIST_THRESH:
            # 位置已在范围内，直接进入对齐阶段
            self.get_logger().info("位置已在目标范围内，直接进入角度对齐阶段")
            self.emergency_stop()
            self.state = self.STATE_ALIGN_YAW
            self.reach_count = 0
            self.yaw_align_count = 0
            return True
        else:
            # 位置不满足，需要移动
            self.state = self.STATE_GO_TO_GOAL
            return False

    # ========== spin 状态回调（仅在第1次开始切换 test）==========
    def spin_status_callback(self, msg):
        """监听 /spin/_action/status，第1次开始切换到 test"""
        executing_exists = False
        for status_msg in msg.status_list:
            if status_msg.status == GoalStatus.STATUS_EXECUTING:
                executing_exists = True
                break

        # 如果当前没有 spin 执行，仅更新标志
        if not executing_exists:
            self.spin_active = False
            return

        # 有 spin 正在执行
        if not self.spin_active:
            # 这是一个新的 spin 开始
            self.spin_active = True
            self.spin_start_count += 1
            self.get_logger().info(f"🔥 spin recovery started ({self.spin_start_count} times in current cycle)")

            if self.spin_start_count == 1:
                self.use_test_cmd_vel = True
                self.get_logger().info("Switching to /cmd_vel_test")
            # 不再处理第3次开始切回，切回由速度条件触发

    # ========== 速度处理核心逻辑 ==========
    def process_cmd_vel(self, linear_x, linear_y, angular_z):
        """根据当前状态处理速度指令（由不同速度源回调调用）"""
        if self.goal_pose is None or self.current_pose is None:
            return

        # 状态处理逻辑
        if self.state == self.STATE_GO_TO_GOAL:
            # GO_TO_GOAL状态：只关注位置到达，转发所有cmd_vel指令
            position_reached = self.has_reached_position()
            
            if position_reached:
                self.reach_count += 1
                if self.reach_count >= self.STABLE_COUNT_REQ:
                    # 位置稳定达标，进入对齐阶段
                    self.emergency_stop()
                    self.state = self.STATE_ALIGN_YAW
                    self.yaw_align_count = 0
                    self.get_logger().info("位置稳定达标，切换到角度对齐阶段")
                    # 注意：这里不发送运动指令，等待对齐阶段处理
                else:
                    # 位置已到达但未稳定，停止线速度，只转发角速度
                    self.move(0.0, 0.0, angular_z, False)
            else:
                # 位置未到达，转发所有指令
                self.reach_count = 0
                self.goal_reached_flag = False
                self.move(linear_x, linear_y, angular_z, False)

        elif self.state == self.STATE_ALIGN_YAW:
            # ALIGN_YAW状态：只关注角度对齐，不转发任何cmd_vel数据
            # 使用自算角速度，绝对值至少为MIN_EFFECTIVE_YAW
            
            yaw_reached = self.has_reached_yaw()
            
            if yaw_reached:
                self.yaw_align_count += 1
                if self.yaw_align_count >= self.YAW_ALIGN_COUNT_REQ:
                    self.stop()  # 这会设置状态为GOAL_REACHED
                    self.get_logger().info("角度稳定对齐，目标完成")
                else:
                    # 角度已对齐但未稳定，保持停止状态
                    self.emergency_stop()
            else:
                # 角度未对齐，使用自算角速度
                self.yaw_align_count = 0
                yaw_diff = self.get_yaw_diff()
                
                # 计算角速度（比例控制）
                k_p = 1.5  # 比例系数
                vyaw = yaw_diff * k_p
                
                # 确保角速度绝对值至少为MIN_EFFECTIVE_YAW
                if abs(vyaw) < self.MIN_EFFECTIVE_YAW:
                    vyaw = self.MIN_EFFECTIVE_YAW if yaw_diff > 0 else -self.MIN_EFFECTIVE_YAW
                
                # 限制最大角速度
                vyaw = max(min(vyaw, self.YAW_ALIGN_SPEED), -self.YAW_ALIGN_SPEED)
                
                # 只发送角速度，线速度为0
                self.move(0.0, 0.0, vyaw, False)

        elif self.state == self.STATE_GOAL_REACHED:
            # 目标已达成，不发送任何指令
            pass

        # 发布当前状态
        self.state_pub.publish(String(data=self.STATE_STRINGS[self.state]))

    # ========== 速度回调 ==========
    def cmd_vel_smooth_callback(self, msg):
        """收到 /cmd_vel_smooth 时的回调"""
        # 如果当前使用 test 源，需要检查是否应该切回
        if self.use_test_cmd_vel:
            # 检测线速度 x 是否大于 0.2
            if msg.linear.x > 0.2:
                self.use_test_cmd_vel = False
                self.spin_start_count = 0
                self.get_logger().info("cmd_vel_smooth x>0.2 detected, switching back to /cmd_vel_smooth and reset count")
                # 继续处理当前 smooth 指令
                self.process_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
            else:
                # 速度条件不满足，忽略 smooth 消息
                return
        else:
            # 正常处理 smooth 指令
            self.process_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def cmd_vel_test_callback(self, msg):
        """收到 /cmd_vel_test 时的回调"""
        # 只有 test 源激活时才使用 test 消息
        if not self.use_test_cmd_vel:
            return
        self.process_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def goal_callback(self, msg: PoseStamped):
        """新目标回调"""
        self.goal_pose = msg
        self.reach_count = 0
        self.yaw_align_count = 0
        self.goal_reached_flag = False

        # 重置 spin 相关状态（新目标重新开始计数）
        self.spin_start_count = 0
        self.use_test_cmd_vel = False   # 新目标默认使用 smooth
        self.spin_active = False       # 重置 spin 执行标志

        # 首先发布goal_reached=False，表示开始处理新目标
        self.goal_pub.publish(Bool(data=False))
        
        goal_yaw = quaternion_to_yaw(msg.pose.orientation)
        self.get_logger().info(f"收到新目标点: 位置({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), 朝向: {goal_yaw:.3f}rad")
        
        # 进行目标点预检查
        if self.current_pose is not None:
            if self.check_goal_precondition():
                # 已在目标范围内，直接进入对齐阶段
                return
            else:
                # 需要移动到目标点
                self.state = self.STATE_GO_TO_GOAL
        else:
            # 还没有当前位置信息，等待定位数据
            self.state = self.STATE_GO_TO_GOAL

    def localization_callback(self, msg: PoseStamped):
        """位置更新回调"""
        self.current_pose = msg
        
        # 如果有目标点但还没有到达，检查是否已经在目标范围内
        if self.goal_pose is not None and not self.goal_reached_flag and self.state == self.STATE_GO_TO_GOAL:
            self.check_goal_precondition()

    def cancel_callback(self, req, res):
        """取消目标服务"""
        if req.data:
            self.goal_pose = None
            self.state = self.STATE_GO_TO_GOAL
            self.reach_count = 0
            self.yaw_align_count = 0
            self.goal_reached_flag = False
            try:
                self.sport_client.StopMove()
                self.get_logger().info("目标已取消")
            except Exception as e:
                self.get_logger().error(f"停止移动失败: {e}")
            res.success = True
        else:
            res.success = False
        return res

    def status_timer_callback(self):
        """定时状态报告（2秒一次，精简信息）"""
        status = f"状态: {self.STATE_STRINGS[self.state]}"
        if self.goal_pose and self.current_pose:
            dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
            dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
            dist = math.hypot(dx, dy)
            yaw_diff = abs(self.get_yaw_diff())
            status += f", 距离误差: {dist:.3f}m, 角度误差: {yaw_diff:.3f}rad"
        status += f", 速度源: {'/cmd_vel_test' if self.use_test_cmd_vel else '/cmd_vel_smooth'}"
        # status += f", spin计数: {self.spin_start_count}/3"
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    node = G1MovementGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()