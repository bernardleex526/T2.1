# flyos-universe-ROS2

@杭州飞阔科技有限公司

## 配置

* 环境：Ubuntu 22.04, ROS Humble
* 激光雷达：MID360
* IMU：MID360内置IMU
* 驱动包：[Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2), [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 安装

```bash

# Nav2额外插件
sudo apt install ros-humble-spatio-temporal-voxel-layer
sudo apt-get install ros-humble-nav2-constrained-smoother
sudo apt-get install ros-humble-nav2-smac-planner

# GTSAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

sudo apt install tf2_eigen ros-humble-tf2-ros

# octomap
sudo apt install ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server ros-humble-rviz-default-plugins

# pcl
sudo apt install ros-humble-perception-pcl

# nav2
sudo apt install ros-humble-nav2-*

# Sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install

# Livox SDK2
cd ~
git clone http://121.40.156.131/fc_developers_public/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make
sudo make install

# 运行livox_ros_driver2遇到共享库报错 解决方法
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/livox.conf
sudo ldconfig
ldconfig -p | grep liblivox_lidar_sdk_shared.so
# 输出：liblivox_lidar_sdk_shared.so (libc6,x86-64) => /usr/local/lib/liblivox_lidar_sdk_shared.so

# unitree_sdk2_python
cd ~
sudo apt install python3-pip
git clone http://121.40.156.131/fc_developers_public/unitree_sdk2_python.git
cd unitree_sdk2_python
sudo pip3 install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple --upgrade numpy
sudo pip3 install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple onnx
pip3 install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple -e .

# flyos-universe-ros2
cd ~
git clone http://121.40.156.131/fc_developers/flyos-universe-ros2.git
cd flyos-universe-ros2

cd ~/flyos-universe-ros2
colcon build
source install/setup.bash
```

## build on G1 (arm64)

### CycloneDDS

**Error when pip3 install -e .:**
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
This error mentions that the cyclonedds path could not be found. First compile and install cyclonedds:

```
cd ~
git clone http://121.40.156.131/fc_developers_public/cyclonedds.git -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install -j$(nproc)
```

Enter the unitree_sdk2_python directory, set CYCLONEDDS_HOME to the path of the cyclonedds you just compiled, and then install unitree_sdk2_python.

```
cd ~/unitree_sdk2_python
export CYCLONEDDS_HOME="/home/unitree/cyclonedds/install"
pip3 install -e .
```

For details, see: https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries

## 参数设置
**livox_ros_driver2/config/MID360_config.json**

实机时使用 livox_ros_driver2/config/MID360_config_g1.json
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.123.222",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.123.222",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.123.222",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.123.222",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.123.120",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 180.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

## 启动

### 启动相机
```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=camera -r __ns:=/vision
```

### 建图
```bash
ros2 launch sensing_launch sensing.launch.py
```

### 导航
```bash
ros2 launch universe_launch universe.launch.py map:=<map_name>
```

## 通用接口

### 保存点云地图（包括单色点云与彩色点云）
```bash
# 建图启动时调用
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$HOME/flyos-universe-ros2/data/<map_name>', save_patches: true}"
```

### 保存2D栅格地图
```bash
# 建图启动时调用
ros2 run nav2_map_server map_saver_cli -t /octomap/projected_map -f $HOME/flyos-universe-ros2/data/<map_name>/map --fmt pgm --mode trinary
```

### 保存着色点云（只有彩色点云）
```bash
# 建图启动时调用
ros2 service call /mapping/save_colored_pcd interface/srv/SaveColoredPcd "{save_path: '$HOME/flyos-universe-ros2/data/<map_name>/colored_pcd'}"
```

### 调用优化服务（一致性地图优化）
```bash
# 启动一致性优化节点
ros2 launch hba hba_launch.py

# 离线调用
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": '$HOME/flyos-universe-ros2/data/<map_name>'}"

# 保存优化后的位姿(HBA 写入的是位姿而非点云地图)
ros2 service call /hba/save_poses interface/srv/SavePoses "{file_path: '$HOME/flyos-universe-ros2/data/<map_name>/optimized_poses.txt'}"
```

### 重定位

支持在初始点调用 service 进行重定位和通过 ```/initial_pose``` 发布位姿进行重定位

#### 设置重定位初始值（在初始点）
```bash
# 导航启动时调用
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "$HOME/flyos-universe-ros2/data/<map_name>/map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```

#### 检查重定位结果（通用）
```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 迁移到其他机器人 / 激光雷达

本仓库已将硬件相关项参数化，迁移到不同雷达/底盘时按以下修改即可(无需改源码)：

1. 换雷达：编辑 `fastlio2/config/lio.yaml`
   - `lidar_type`：`livox`(订阅 Livox CustomMsg) 或 `pointcloud2`(订阅通用 PointCloud2，适配 Velodyne/Ouster/RoboSense 等)
   - `lidar_topic`：对应雷达点云话题；`lidar_max_line`：Livox 扫描线数(MID360=4)
   - `pcl2_time_field`/`pcl2_time_scale`：通用模式下每点时间字段名(Velodyne 用 `time`/scale 1.0；RoboSense 用 `timestamp`/scale 1.0，支持 FLOAT32/FLOAT64；为空则不做扫描内运动补偿)
   - `imu_acc_scale`：MID360 内置 IMU 设 10.0；已输出 m/s² 的标准 IMU 设 1.0
   - `ext_il`：雷达-IMU 外参；同步修改下方 TF
2. 换底盘/安装位：
   - `ros2 launch universe_launch universe.launch.py data_dir:=<map数据目录> map:=<map名> body_base_x:=.. body_base_y:=.. body_base_z:=.. body_base_roll:=.. body_base_pitch:=.. body_base_yaw:=..`
   - `body_base_*` 为 `body`(lio 体坐标系)→`base_link` 的静态外参，雷达/IMU 非原点安装时务必填实际偏移
   - 数据目录可用环境变量 `FLYOS_DATA_DIR` 覆盖(默认 `/home/flyos-universe-ros2/data`)
3. 换运动学/控制层：控制层 `g1_movement_gate` 与避障 `preprocessor` 当前为 Unitree G1 全向(输出 `vy`、`LocoClient.Move`)，差速/阿克曼机器人需重写该网关为对应速度接口；nav2 控制器 `nav2_smac_params2.yaml` 中 DWB 的 `max_vel_y`/`vy_samples`/`acc_lim_y` 已按差速设为 0，全向底盘需相应放开

### nav2规划相关（暂时，后续需要将启动命令集成到导航中）
```bash
#规划部分启用
ros2 launch robot_2d_navigation nav2_planning_launch.py

#机器人位姿节点
ros2 run robot_pose map_pose_publisher

#速度平滑节点
ros2 run cmd_vel_smoother cmd_vel_smoother_node

#机器人控制接口
python3 ~/flyos-universe-ros2/src/control/g1_movement_gate/g1_movement_gate/g1_movement_gate.py 
```

### 接口测试
```bash
ros2 service call /multi_nav/list g1_multi_goal_manager/srv/ListGoals  #查看航点列表
/////ros2 topic pub /navigation_start std_msgs/msg/Int32 "{data: -1}" -1     #从1开始连续导航
/////ros2 service call /multi_nav/pause g1_multi_goal_manager/srv/StopNavigation   #停止导航(内部已用 NavigateToPose action 取消服务取消 nav2 当前目标)
/////ros2 service call /goal_cancel std_srvs/srv/SetBool "{data: true}"            #检查控制节点是否取消
/////ros2 service call /multi_nav/resume g1_multi_goal_manager/srv/StopNavigation  #重启导航
ros2 service call /multi_nav/update_name g1_multi_goal_manager/srv/UpdateGoalName "{index: 1, new_name: '测试点'}"  #给导航点改名
/////ros2 service call /multi_nav/delete g1_multi_goal_manager/srv/DeleteGoal "{index: 2}"   #删除导航点
ros2 service call /multi_nav/clear g1_multi_goal_manager/srv/ClearGoals                 #清除所有导航点
ros2 service call /nav/location g1_multi_goal_manager/srv/GetCurrentLocation            #获取当前位置
```
## FAST-LIO既存研究

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
3. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
4. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
5. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
6. [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM#for-livox-lidar): A [Scan-Context](https://github.com/irapkaist/scancontext) loop closure module that can directly work with FAST-LIO.
