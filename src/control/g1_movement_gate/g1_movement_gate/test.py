#!/usr/bin/env python3
"""
Unitree G1 纯转发节点
功能：仅接收速度指令并直接转发给机器人
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class G1PureForwarder(Node):
    def __init__(self):
        super().__init__("g1_pure_forwarder")
        
        # 参数
        self.declare_parameter('lan_port', 'enP8p1s0')
        self.lan_port = self.get_parameter('lan_port').value
        
        # 初始化Unitree客户端
        ChannelFactoryInitialize(0, self.lan_port)
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        
        # 订阅速度话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.cmd_vel_callback, 
            10
        )
        
        self.get_logger().info("G1纯转发节点启动")

    def cmd_vel_callback(self, msg: Twist):
        """直接转发速度指令"""
        try:
            self.sport_client.Move(msg.linear.x, msg.linear.y, msg.angular.z, False)
        except Exception as e:
            self.get_logger().error(f"转发失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = G1PureForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()