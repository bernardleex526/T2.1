#!/usr/bin/env python3
"""
剧本执行节点 - 简化版（无goal_id校验 + 空导航点原地执行）+ Zenoh集成 + 严格串行动作 + 导航后延迟
"""

import rclpy
import json
import os
import time
import requests
import threading
from typing import Dict, List, Optional, Set
from enum import Enum
from dataclasses import dataclass, field
from threading import Lock, Event
from concurrent.futures import ThreadPoolExecutor
import zenoh

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool


class ScriptState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class ScriptStep:
    step_id: str
    point: Optional[str] = None
    goal_id: Optional[int] = None  # 仅存储，不校验
    actions: List[str] = field(default_factory=list)
    audio_id: Optional[str] = None
    is_walk_transition: bool = False


class ScriptExecutor(Node):
    def __init__(self):
        super().__init__('script_executor')
        
        # 参数
        self.declare_parameter('audio_host', 'localhost')
        self.declare_parameter('audio_port', 5000)
        self.declare_parameter('motion_host', 'localhost')
        self.declare_parameter('motion_port', 5000)
        self.declare_parameter('audio_duration_api_host', 'localhost')
        self.declare_parameter('audio_duration_api_port', 5000)
        
        self.audio_host = self.get_parameter('audio_host').value
        self.audio_port = self.get_parameter('audio_port').value
        self.motion_host = self.get_parameter('motion_host').value
        self.motion_port = self.get_parameter('motion_port').value
        self.audio_duration_api_host = self.get_parameter('audio_duration_api_host').value
        self.audio_duration_api_port = self.get_parameter('audio_duration_api_port').value
        self.audio_duration_api_url = f"http://{self.audio_duration_api_host}:{self.audio_duration_api_port}/api/scripts/audio/duration"
        
        # 音频API地址
        self.audio_play_api_url = f"http://{self.audio_host}:{self.audio_port}/api/scripts/audio/play"
        self.audio_stop_api_url = f"http://{self.audio_host}:{self.audio_port}/api/scripts/audio/stop"
        
        # 状态
        self.state = ScriptState.IDLE
        self.current_map = None
        self.current_script_name = None
        self.current_steps = []
        self.current_step_index = 0
        self.stop_requested = False
        self.pause_requested = False
        self.pause_event = Event()
        self.pause_event.set()
        self.state_lock = Lock()
        
        # 强制终止剧本标记
        self.force_stop_script = False
        self.force_stop_lock = Lock()
        
        # 暂停中断状态标记
        self.pause_interrupted_index = None
        self.is_pause_interrupted = False
        
        # 音频完成停止动作相关状态
        self.audio_play_complete = False
        self.audio_motion_lock = Lock()
        
        # 动作相关属性
        self.motion_map = {}
        self.motion_lock = Lock()
        self.motion_session = requests.Session()
        self.motion_session.timeout = 10
        self.motion_url = f"http://{self.motion_host}:{self.motion_port}/api/motions"
        
        # ==================== 核心修改：严格的串行执行控制 ====================
        self.current_action = None  # 当前正在执行的动作
        self.action_complete_event = Event()  # 动作完成事件
        self.action_complete_event.clear()  # 初始化为未完成
        self.action_control_lock = Lock()  # 动作控制锁
        # ===================================================================
        
        # Zenoh订阅线程
        self.zenoh_subscription_active = True
        self.zenoh_subscription_thread = threading.Thread(
            target=self._subscribe_zenoh, 
            daemon=True
        )
        self.zenoh_subscription_thread.start()
        
        # 线程池
        self.thread_pool = ThreadPoolExecutor(max_workers=3)
        
        # ROS接口
        self._setup_ros_interfaces()
        
        # 导航服务客户端
        self.cancel_nav_client = self.create_client(SetBool, '/goal_cancel')
        
        # 导航状态
        self.goal_reached = False
        self.navigation_active = False
        
        # 初始化时加载动作列表
        self._load_motions()

        self.get_logger().info("✅ 剧本执行节点已初始化（严格串行动作执行版本 + 导航后延迟）")

    def _setup_ros_interfaces(self):
        """设置ROS接口"""
        # 订阅器
        self.create_subscription(String, '/run_scripts', self.script_callback, 10)
        self.create_subscription(String, '/semantic/stop_task', self.stop_callback, 10)
        self.create_subscription(Bool, '/scripts/pause', self.pause_callback, 10)
        
        # 发布器
        self.status_pub = self.create_publisher(String, '/script_status', 10)
        self.nav_start_pub = self.create_publisher(Int32, '/navigation_start', 10)
        
        # 订阅导航状态
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        
        # 定时状态发布
        self.create_timer(1.0, self.publish_status)

    def motion_pub(self, cmd: str):
        """发布动作指令 - 使用Zenoh"""
        try:
            with zenoh.open(zenoh.Config()) as session:
                key = 'unitree/motion_cmd'
                pub = session.declare_publisher(key)
                pub.put(cmd)
                self.get_logger().info(f"📤 已发布动作指令：{cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ 发布动作指令失败: {str(e)}")

    def _subscribe_zenoh(self):
        """订阅Zenoh动作状态"""
        try:
            with zenoh.open(zenoh.Config()) as session:
                sub = session.declare_subscriber(
                    "unitree/move_flag",
                    self._motion_state_callback
                )
                # 保持订阅活跃
                while self.zenoh_subscription_active:
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"❌ Zenoh订阅失败: {str(e)}")

    def _motion_state_callback(self, sample):
        """Zenoh动作状态回调 - 严格串行版本"""
        try:
            # 解析Zenoh消息
            status_value = sample.payload.to_string()
            self.get_logger().debug(f"📥 收到Zenoh动作状态：{status_value}")
            
            # 只处理完成信号（值为0）
            if status_value.isdigit() and int(status_value) == 0:
                with self.action_control_lock:
                    self.get_logger().info(f"✅ 收到动作完成信号 flag=0")
                    
                    # 标记当前动作为完成
                    if self.current_action:
                        self.get_logger().info(f"✅ 动作 {self.current_action} 执行完成")
                        self.current_action = None
                    
                    # 设置完成事件，唤醒等待线程
                    self.action_complete_event.set()
                    
        except Exception as e:
            self.get_logger().error(f"❌ Zenoh回调处理失败: {str(e)}")

    def _load_motions(self):
        """加载动作列表"""
        try:
            self.get_logger().info(f"正在从 {self.motion_url} 加载动作列表...")
            response = self.motion_session.get(self.motion_url)
            response.raise_for_status()
            motion_data = response.json()
        
            with self.motion_lock:
                motion_list = motion_data.get("data", [])
                self.motion_map = {
                    item["key"]: item["value"] for item in motion_list
                    if "key" in item and "value" in item  
                }
            self.get_logger().info(f"成功加载 {len(self.motion_map)} 个动作：{list(self.motion_map.keys())}")
            if len(self.motion_map) == 0:
                self.get_logger().warn("⚠️ 加载的动作列表为空！")
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f"❌ 连接动作服务失败: {self.motion_url} - {str(e)}")
            self.motion_map = {}
        except requests.exceptions.Timeout as e:
            self.get_logger().error(f"❌ 动作服务请求超时: {self.motion_url} - {str(e)}")
            self.motion_map = {}
        except Exception as e:
            self.get_logger().error(f"❌ 加载动作列表失败: {str(e)}，异常类型：{type(e).__name__}")
            self.motion_map = {}

    def _get_audio_total_duration(self, audio_id: str) -> float:
        """获取音频时长"""
        if not self.current_map or not self.current_script_name or not audio_id:
            self.get_logger().error("❌ 缺少地图/剧本名/音频ID，无法获取时长")
            return 0.0
        
        try:
            params = {
                "map": self.current_map,
                "audio_id": audio_id,
                "scripts_name": self.current_script_name
            }
            self.get_logger().info(f"🔍 调用音频时长API：{self.audio_duration_api_url}，参数：{params}")
            response = requests.get(
                self.audio_duration_api_url,
                params=params
            )
            response.raise_for_status()
            
            result = response.json()
            self.get_logger().debug(f"📤 音频时长API返回：{result}")
            total_duration = float(result.get("data", {}).get("duration", 0.0))
            
            if total_duration <= 0:
                self.get_logger().warn(f"⚠️ 音频 {audio_id} 时长无效：{total_duration} 秒")
            else:
                self.get_logger().info(f"✅ 获取音频 {audio_id} 总时长：{total_duration:.2f} 秒")
            return total_duration
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f"❌ 连接音频时长API失败: {self.audio_duration_api_url} - {str(e)}")
        except requests.exceptions.Timeout as e:
            self.get_logger().error(f"❌ 音频时长API请求超时: {self.audio_duration_api_url} - {str(e)}")
        except ValueError as e:
            self.get_logger().error(f"❌ 音频时长转换失败：{str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 获取音频 {audio_id} 时长失败: {str(e)}，异常类型：{type(e).__name__}")
        return 0.0

    def publish_status(self):
        """发布状态"""
        msg = String()
        with self.state_lock:
            msg.data = self.state.value
            self.get_logger().debug(f"📢 发布节点状态：{msg.data}")
        self.status_pub.publish(msg)

    def script_callback(self, msg: String):
        """接收剧本数据"""
        with self.state_lock:
            if self.state != ScriptState.IDLE:
                self.get_logger().warn(f"⚠️ 当前状态 {self.state.value}，无法执行新剧本")
                return
        
        try:
            self.get_logger().info(f"📥 收到剧本执行请求，原始数据：{msg.data}")
            data = json.loads(msg.data)
            self.current_map = data.get('map', 'default')
            self.current_script_name = data.get('scripts_name', 'default')
            steps_data = data.get('data', [])
            
            self.get_logger().info(f"📋 解析剧本：map={self.current_map}，script_name={self.current_script_name}，原始步骤数={len(steps_data)}")
            
            # 解析步骤（去掉所有goal_id校验）
            steps = []
            for idx, step_data in enumerate(steps_data):
                step = self._parse_step(step_data)
                if step:
                    steps.append(step)
                else:
                    self.get_logger().warn(f"⚠️ 第{idx+1}个步骤解析失败，原始数据：{step_data}")
            
            if not steps:
                self.get_logger().error("❌ 没有有效的步骤")
                return
            
            with self.state_lock:
                self.current_steps = steps
                self.current_step_index = 0
                self.stop_requested = False
                self.pause_requested = False
                self.state = ScriptState.RUNNING
                self.pause_interrupted_index = None
                self.is_pause_interrupted = False
            with self.force_stop_lock:
                self.force_stop_script = False
            with self.audio_motion_lock:
                self.audio_play_complete = False
            with self.action_control_lock:
                self.current_action = None
                self.action_complete_event.clear()
            
            # 执行剧本
            self.thread_pool.submit(self._execute_script)
            
            self.get_logger().info(f"🚀 开始执行剧本: {self.current_script_name} (有效步骤数：{len(steps)})")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ 剧本JSON解析失败：{str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 解析剧本失败: {str(e)}，异常类型：{type(e).__name__}")

    def _parse_step(self, step_data: Dict) -> Optional[ScriptStep]:
        """解析单个步骤（完全去掉goal_id校验），使用步骤id作为audio_id"""
        try:
            step_id = str(step_data.get('id', 'unknown'))  # 转换为字符串
            data = step_data.get('data', {})
            
            point = data.get('point', '')
            action_str = data.get('action', '')
            # 仅尝试转换，失败也不终止步骤解析
            goal_id = data.get('goal_id')
            parsed_goal_id = None
            if goal_id is not None and goal_id != '':
                try:
                    parsed_goal_id = int(goal_id)
                    self.get_logger().debug(f"🔧 解析步骤 {step_id}：goal_id={goal_id} → 整数{parsed_goal_id}")
                except (ValueError, TypeError):
                    self.get_logger().debug(f"⚠️ 步骤 {step_id} 的goal_id={goal_id} 不是有效整数，设为None")
                    parsed_goal_id = None
            
            # 原有动作解析逻辑
            actions = []
            if action_str:
                actions = [a.strip() for a in action_str.split(',') if a.strip()]
            
            # 原有行走过渡判断
            is_walk_transition = (point == "行走过渡")
            
            self.get_logger().debug(f"🔧 解析步骤 {step_id}：point={point}, goal_id={parsed_goal_id}, actions={actions}, is_walk_transition={is_walk_transition}")
            
            return ScriptStep(
                step_id=step_id,
                point=point if point and not is_walk_transition else None,
                goal_id=parsed_goal_id,
                actions=actions,
                audio_id=step_id,  # 使用步骤id作为audio_id
                is_walk_transition=is_walk_transition
            )
        except Exception as e:
            self.get_logger().error(f"❌ 解析步骤失败: {str(e)}，异常类型：{type(e).__name__}")
            return None

    def _execute_script(self):
        """执行剧本"""
        try:
            self.get_logger().info(f"📝 开始执行剧本主逻辑，总步骤数：{len(self.current_steps)}")
            step_index = 0
            while step_index < len(self.current_steps):
                with self.state_lock:
                    if self.is_pause_interrupted:
                        step_index = self.pause_interrupted_index
                        self.is_pause_interrupted = False
                        self.get_logger().info(f"🔄 恢复中断，重置步骤索引为：{step_index}")
                
                with self.force_stop_lock:
                    if self.force_stop_script:
                        self.get_logger().error(f"🛑 检测到强制终止标记，立即终止所有步骤！")
                        break
                
                with self.state_lock:
                    if self.stop_requested:
                        self.get_logger().info("🛑 检测到停止请求，终止剩余步骤")
                        break
                    self.current_step_index = step_index
                    self.get_logger().info(f"🔄 当前执行步骤索引：{step_index+1}/{len(self.current_steps)}")
                
                step = self.current_steps[step_index]
                self._execute_step(step, step_index)
                
                with self.force_stop_lock:
                    if self.force_stop_script:
                        self.get_logger().error(f"🛑 强制终止剧本：当前步骤{step_index}执行完，后续步骤全部终止")
                        break
                
                with self.state_lock:
                    if not self.is_pause_interrupted:
                        step_index += 1
                    else:
                        self.get_logger().info(f"⏸️ 暂停中断，步骤索引保持：{step_index}")
            
            with self.force_stop_lock:
                self.force_stop_script = False
            with self.state_lock:
                if self.stop_requested or self.force_stop_script:
                    self.get_logger().info("🛑 剧本已彻底终止")
                    self.state = ScriptState.IDLE
                    self.pause_interrupted_index = None
                    self.is_pause_interrupted = False
                else:
                    self.get_logger().info("🎉 剧本所有步骤执行完成")
                    self.state = ScriptState.IDLE
                    
        except Exception as e:
            self.get_logger().error(f"💥 剧本执行异常: {str(e)}，异常类型：{type(e).__name__}")
            with self.state_lock:
                self.state = ScriptState.ERROR
        finally:
            with self.force_stop_lock:
                self.force_stop_script = False
            with self.state_lock:
                self.stop_requested = False

    def _execute_step(self, step: ScriptStep, step_index: int):
        """执行单个步骤，使用步骤的audio_id作为音频ID"""
        # 使用步骤的audio_id作为音频ID
        audio_id = step.audio_id if step.audio_id else str(step_index)
        self.get_logger().info(f"📌 开始执行步骤 {step.step_id}（索引：{step_index}，audio_id：{audio_id}，导航点：{step.point}，goal_id：{step.goal_id}，动作数：{len(step.actions)}）")
        
        with self.force_stop_lock:
            if self.force_stop_script:
                self.get_logger().error(f"🛑 强制终止剧本：步骤{step_index}未执行")
                return
        
        self._check_pause()
        
        with self.audio_motion_lock:
            self.audio_play_complete = False
        
        # 导航逻辑：如果goal_id有效（非None），则执行导航
        if step.goal_id is not None:
            self.get_logger().info(f"🗺️ 开始处理导航：goal_id={step.goal_id}（备注点：{step.point}）")
            with self.force_stop_lock:
                if self.force_stop_script:
                    self.get_logger().error(f"🛑 强制终止剧本：取消导航")
                    return
            # 执行导航（不进行校验，直接转发）
            nav_result = self._navigate_to_goal_id(step.goal_id)
            with self.force_stop_lock:
                if self.force_stop_script:
                    self.get_logger().error(f"🛑 强制终止剧本：导航完成后退出步骤")
                    return
            if nav_result:
                self.get_logger().info(f"✅ 导航到 goal_id={step.goal_id} 成功")
            else:
                self.get_logger().warn(f"⚠️ 导航到 goal_id={step.goal_id} 失败或取消，继续执行动作/音频")
        # 空goal_id：跳过导航，原地执行动作/音频
        else:
            self.get_logger().info(f"ℹ️ 无有效goal_id，跳过导航，原地执行动作/音频")
        
        # ========== 新增：导航完成后延迟1秒再执行语音和动作 ==========
        if step.goal_id is not None and self.goal_reached:
            self.get_logger().info(f"⏱️ 导航完成，延迟1秒再执行语音和动作")
            delay_start_time = time.time()
            delay_duration = 1.0  # 1秒延迟
            
            while time.time() - delay_start_time < delay_duration:
                with self.force_stop_lock:
                    if self.force_stop_script:
                        self.get_logger().error(f"🛑 强制终止剧本：延迟过程中被终止")
                        return
                
                with self.state_lock:
                    if self.stop_requested:
                        self.get_logger().info(f"🛑 延迟过程中收到停止请求")
                        return
                
                if self.pause_requested:
                    self.get_logger().info(f"⏸️ 延迟过程中检测到暂停请求")
                    self.pause_event.wait()
                
                remaining = delay_duration - (time.time() - delay_start_time)
                if remaining > 0:
                    time.sleep(min(0.1, remaining))
            
            self.get_logger().info(f"✅ 1秒延迟完成")
        # ===========================================================
        
        # 音频播放
        audio_thread = None
        with self.force_stop_lock:
            if self.force_stop_script:
                self.get_logger().error(f"🛑 强制终止剧本：跳过音频播放")
                return
        
        if audio_id:
            self.get_logger().info(f"🔊 启动音频播放线程：audio_id={audio_id}")
            audio_thread = threading.Thread(
                target=self._play_audio,
                args=(audio_id,),
                daemon=True
            )
            audio_thread.start()
        else:
            self.get_logger().info(f"🔇 该步骤无音频ID，跳过音频播放")
        
        # ============== 严格串行动作执行 ==============
        if step.actions:
            self.get_logger().info(f"🤖 开始严格串行执行 {len(step.actions)} 个动作")
            
            for idx, action in enumerate(step.actions):
                # 检查强制终止
                with self.force_stop_lock:
                    if self.force_stop_script:
                        self.get_logger().error(f"🛑 强制终止：跳过动作 {action}")
                        break
                
                # 检查停止请求
                with self.state_lock:
                    if self.stop_requested:
                        self.get_logger().info(f"🛑 停止请求：跳过动作 {action}")
                        break
                
                # 检查音频是否完成
                with self.audio_motion_lock:
                    if self.audio_play_complete:
                        self.get_logger().info(f"🔇 音频已完成：跳过动作 {action}")
                        break
                
                self._check_pause()
                
                self.get_logger().info(f"🎯 准备执行第 {idx+1}/{len(step.actions)} 个动作：{action}")
                
                # 如果动作名为"none"，直接跳过
                if action == "none":
                    self.get_logger().info(f"⏭️ 跳过none动作")
                    continue
                
                # 查找动作映射
                motion_cn = next((k for k, v in self.motion_map.items() if v == action), None)
                if not motion_cn:
                    self.get_logger().error(f"❌ 未知动作: {action}")
                    continue
                
                motion_en = self.motion_map[motion_cn]
                self.get_logger().info(f"🔄 转换动作：{motion_cn} → {motion_en}")
                
                # ========== 核心：发送动作并等待完成 ==========
                with self.action_control_lock:
                    # 重置完成事件
                    self.action_complete_event.clear()
                    
                    # 设置当前正在执行的动作
                    self.current_action = action
                    
                    # 发送动作指令
                    self.get_logger().info(f"📤 发送动作指令：{motion_en}")
                    try:
                        self.motion_pub(motion_en)
                    except Exception as e:
                        self.get_logger().error(f"❌ 发送动作失败：{str(e)}")
                        continue
                
                # ========== 等待动作完成 ==========
                self.get_logger().info(f"⌛ 等待动作 {action} 完成...")
                
                # 等待动作完成事件（收到flag=0）
                while not self.action_complete_event.is_set():
                    # 检查各种停止条件
                    with self.force_stop_lock:
                        if self.force_stop_script:
                            self.get_logger().error(f"🛑 强制终止：中断动作 {action} 的等待")
                            break
                    
                    with self.state_lock:
                        if self.stop_requested:
                            self.get_logger().info(f"🛑 停止请求：中断动作 {action} 的等待")
                            break
                    
                    with self.audio_motion_lock:
                        if self.audio_play_complete:
                            self.get_logger().info(f"🔇 音频完成：中断动作 {action} 的等待")
                            break
                    
                    # 检查暂停
                    if self.pause_requested:
                        self.get_logger().info(f"⏸️ 暂停：中断动作 {action} 的等待")
                        break
                    
                    # 短暂等待，避免CPU占用过高
                    time.sleep(0.1)
                
                # 如果等待被中断，退出循环
                if not self.action_complete_event.is_set():
                    self.get_logger().warn(f"⚠️ 动作 {action} 等待被中断")
                    break
                
                self.get_logger().info(f"✅ 第 {idx+1}/{len(step.actions)} 个动作 {action} 执行完成")
                time.sleep(0.3)  # 动作间小间隔
            
            self.get_logger().info(f"🎉 步骤 {step.step_id} 的所有动作执行完成")
        # =============================================
        
        # 等待音频完成
        if audio_thread:
            self.get_logger().debug(f"⌛ 等待音频线程执行完成：audio_id={audio_id}")
            while audio_thread.is_alive():
                with self.force_stop_lock:
                    if self.force_stop_script:
                        self.get_logger().error(f"🛑 强制终止剧本：终止音频线程")
                        self._stop_audio()
                        break
                audio_thread.join(timeout=0.1)
            self.get_logger().info(f"🔚 音频线程执行完毕：audio_id={audio_id}")
        
        # 行走过渡
        if step.is_walk_transition:
            self.get_logger().info(f"🚶 执行行走过渡，等待0.5秒")
            time.sleep(0.5)
        
        self.get_logger().info(f"✅ 步骤 {step.step_id} 执行完毕")

    def _navigate_to_goal_id(self, goal_id: int) -> bool:
        """使用goal_id导航（直接转发，不进行校验）"""
        self.get_logger().info(f"🗺️ 开始导航流程：目标goal_id={goal_id}")
        
        # 重置导航状态
        self.goal_reached = False
        self.navigation_active = True
        
        # 发布导航指令（直接转发，不校验）
        msg = Int32()
        msg.data = goal_id
        self.nav_start_pub.publish(msg)
        self.get_logger().info(f"📢 发布导航命令到 /navigation_start：goal_id={goal_id}")
        
        # 等待导航完成
        start_time = time.time()
        self.get_logger().info(f"⌛ 等待导航完成，超时时间：5分钟（300秒）")
        while self.navigation_active and not self.goal_reached:
            if self.pause_requested:
                self.get_logger().info(f"⏸️ 检测到暂停请求，立即取消导航 goal_id={goal_id}")
                self._cancel_navigation()
                return False
            
            with self.force_stop_lock:
                if self.force_stop_script:
                    self.get_logger().error(f"🛑 导航中检测到强制终止，取消导航")
                    self._cancel_navigation()
                    return False
            
            self._check_pause()
            
            with self.state_lock:
                if self.stop_requested:
                    self.get_logger().info("🛑 收到停止请求，取消导航")
                    self._cancel_navigation()
                    return False
            
            elapsed_time = time.time() - start_time
            if elapsed_time > 300:
                self.get_logger().error(f"❌ 导航超时：goal_id={goal_id}，已等待{elapsed_time:.1f}秒")
                self._cancel_navigation()
                return False
            
            if int(elapsed_time) % 10 == 0:
                self.get_logger().debug(f"⌛ 导航中：已等待{elapsed_time:.1f}秒，goal_id={goal_id}")
            
            time.sleep(0.1)
        
        self.get_logger().info(f"✅ 导航完成：goal_id={goal_id}，总耗时={time.time()-start_time:.1f}秒")
        return True

    def _cancel_navigation(self):
        """取消导航"""
        try:
            self.get_logger().info(f"🛑 开始取消导航流程")
            if self.cancel_nav_client.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True
                future = self.cancel_nav_client.call_async(req)
                self.navigation_active = False
                self.get_logger().info(f"✅ 已发送导航取消命令到 /goal_cancel")
            else:
                self.get_logger().error(f"❌ /goal_cancel 服务不可用，无法取消导航")
        except Exception as e:
            self.get_logger().error(f"❌ 取消导航失败: {str(e)}，异常类型：{type(e).__name__}")

    def _play_audio(self, audio_id: str):
        """播放音频"""
        self.get_logger().info(f"🔊 开始音频播放流程：audio_id={audio_id}")
        play_success = False
        try:
            total_duration = self._get_audio_total_duration(audio_id)
            if total_duration <= 0:
                self.get_logger().error(f"❌ 音频 {audio_id} 时长无效，跳过播放")
                with self.audio_motion_lock:
                    self.audio_play_complete = True
                return
            
            play_params = {
                "map": self.current_map,
                "audio_id": audio_id,
                "scripts_name": self.current_script_name
            }
            self.get_logger().info(f"📤 调用音频播放API：{self.audio_play_api_url}，参数：{play_params}")
            
            play_resp = requests.post(
                self.audio_play_api_url,
                json=play_params
            )
            
            self.get_logger().debug(f"📤 音频播放API响应：状态码={play_resp.status_code}")
            if play_resp.status_code == 200:
                play_result = play_resp.json()
                if play_result.get('code') == 2000 or play_result.get('data', {}).get('status') == 'playing':
                    play_success = True
                    self.get_logger().info(f"✅ 音频 {audio_id} 播放成功，预计时长：{total_duration:.2f}秒")
                    
                    start_time = time.time()
                    max_wait_time = total_duration + 2
                    self.get_logger().info(f"⌛ 开始音频播放等待：总时长={total_duration:.2f}秒")
                    
                    while True:
                        if self.pause_requested:
                            self.get_logger().info(f"⏸️ 检测到暂停请求，停止音频 {audio_id}")
                            self._stop_audio()
                            break
                        
                        with self.force_stop_lock:
                            if self.force_stop_script:
                                self.get_logger().error(f"🛑 音频播放中检测到强制终止，停止音频")
                                self._stop_audio()
                                break
                        
                        if self.pause_requested:
                            self.get_logger().info(f"⏸️ 暂停音频播放等待")
                            self.pause_event.wait()
                            self.get_logger().info(f"▶️ 恢复音频播放等待")
                        
                        if self.stop_requested:
                            self.get_logger().info(f"🛑 收到停止请求，终止音频播放")
                            self._stop_audio()
                            break
                        
                        elapsed_time = time.time() - start_time
                        if elapsed_time >= total_duration:
                            self.get_logger().info(f"✅ 音频 {audio_id} 播放完成")
                            break
                        
                        if elapsed_time >= max_wait_time:
                            self.get_logger().warn(f"⚠️ 音频 {audio_id} 播放超时")
                            break
                        
                        if int(elapsed_time) % 1 == 0:
                            self.get_logger().debug(f"🔊 音频播放中：已播放{elapsed_time:.1f}秒/总{total_duration:.1f}秒")
                        
                        time.sleep(0.2)
                else:
                    self.get_logger().warn(f"⚠️ 音频 {audio_id} 播放API返回失败：{play_result.get('message', '未知错误')}")
            else:
                self.get_logger().warn(f"⚠️ 音频 {audio_id} 播放API HTTP错误：状态码={play_resp.status_code}")
                
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f"❌ 连接音频播放API失败: {self.audio_play_api_url} - {str(e)}")
        except requests.exceptions.Timeout as e:
            self.get_logger().error(f"❌ 音频播放API请求超时: {self.audio_play_api_url} - {str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 播放音频 {audio_id} 失败: {str(e)}，异常类型：{type(e).__name__}")
        finally:
            if not play_success:
                self.get_logger().info(f"🛑 音频播放失败，执行兜底停止")
                self._stop_audio()
            with self.audio_motion_lock:
                self.audio_play_complete = True

    def _stop_audio(self):
        """停止音频"""
        try:
            self.get_logger().info(f"🔇 调用音频停止API：{self.audio_stop_api_url}")
            stop_resp = requests.post(
                self.audio_stop_api_url
            )
            self.get_logger().debug(f"📤 音频停止API响应：状态码={stop_resp.status_code}")
            stop_result = stop_resp.json()
            self.get_logger().info(f"✅ 音频停止API调用结果：{stop_result.get('status')} - {stop_result.get('message', '无消息')}")
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f"❌ 连接音频停止API失败: {self.audio_stop_api_url} - {str(e)}")
        except requests.exceptions.Timeout as e:
            self.get_logger().error(f"❌ 音频停止API请求超时: {self.audio_stop_api_url} - {str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 停止音频失败: {str(e)}，异常类型：{type(e).__name__}")

    def goal_reached_callback(self, msg: Bool):
        """目标到达回调"""
        self.get_logger().info(f"📥 收到导航目标到达信号：{msg.data}")
        if msg.data:
            self.goal_reached = True
            self.navigation_active = False
            self.get_logger().info("✅ 标记导航完成")

    def stop_callback(self, msg: String):
        """停止剧本回调"""
        self.get_logger().info(f"🛑 收到停止剧本请求：{msg.data}")
        
        with self.force_stop_lock:
            self.force_stop_script = True
        with self.state_lock:
            self.stop_requested = True
            self.state = ScriptState.IDLE
            self.pause_interrupted_index = None
            self.is_pause_interrupted = False
        
        self.get_logger().info(f"🗺️ 取消导航")
        self._cancel_navigation()
        
        self.get_logger().info(f"🔊 停止音频")
        self._stop_audio()
        
        self.get_logger().info(f"🤖 停止动作")
        self.motion_pub("stop")
        
        with self.action_control_lock:
            self.current_action = None
            self.action_complete_event.set()
        
        self.pause_event.set()
        self.get_logger().info(f"✅ 停止流程执行完毕")

    def pause_callback(self, msg: Bool):
        """暂停/恢复回调"""
        pause = msg.data
        self.get_logger().info(f"⏯️ 收到暂停/恢复请求：{pause}")
        
        try:
            with self.state_lock:
                if pause and self.state == ScriptState.RUNNING:
                    self.get_logger().info(f"⏸️ 暂停剧本执行：{self.state.value} → PAUSED")
                    self.state = ScriptState.PAUSED
                    self.pause_requested = True
                    self.pause_event.clear()
                    self.pause_interrupted_index = self.current_step_index
                    self.is_pause_interrupted = True
                    self.get_logger().info(f"📌 记录暂停中断索引：{self.pause_interrupted_index}")
                    
                    self.get_logger().info(f"🛑 暂停时终止当前操作")
                    self._stop_audio()
                    self.motion_pub("stop")
                    self._cancel_navigation()
                    
                    with self.action_control_lock:
                        self.current_action = None
                        self.action_complete_event.set()
                
                elif not pause and self.state == ScriptState.PAUSED:
                    self.get_logger().info(f"▶️ 恢复剧本执行：{self.state.value} → RUNNING")
                    self.state = ScriptState.RUNNING
                    self.pause_requested = False
                    self.pause_event.set()
                else:
                    self.get_logger().warn(f"⚠️ 暂停/恢复请求无效：pause={pause}，当前状态={self.state.value}")
        except Exception as e:
            self.get_logger().error(f"❌ 处理暂停/恢复请求异常：{str(e)}，异常类型：{type(e).__name__}")

    def _check_pause(self):
        """检查暂停状态"""
        if self.pause_requested:
            self.get_logger().info(f"⏸️ 检测到暂停请求，等待恢复")
            self.pause_event.wait()
            self.get_logger().info(f"▶️ 暂停恢复，继续执行")

    def destroy_node(self):
        """销毁节点"""
        self.get_logger().info("🧹 开始清理节点资源...")
        
        self.thread_pool.shutdown(wait=False)
        self._stop_audio()
        self.motion_session.close()
        
        # 停止Zenoh订阅线程
        self.zenoh_subscription_active = False
        if self.zenoh_subscription_thread.is_alive():
            self.zenoh_subscription_thread.join(timeout=1.0)
        
        self.get_logger().info("✅ 节点资源清理完成")
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = ScriptExecutor()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        node.get_logger().info("🚀 剧本执行节点启动成功，开始自旋")
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ 节点收到键盘中断信号")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f"💥 节点运行异常: {str(e)}，异常类型：{type(e).__name__}")
        else:
            print(f"💥 节点初始化失败: {str(e)}")
    finally:
        if 'node' in locals():
            node.get_logger().info("🔌 销毁剧本执行节点")
            node.destroy_node()
        rclpy.shutdown()
        print("✅ ROS2已关闭，节点退出")


if __name__ == "__main__":
    main()