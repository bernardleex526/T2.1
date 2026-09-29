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
# 启动一致性优化节点（rviz2 默认关闭，需要可视化时加 use_rviz:=true）
ros2 launch hba hba_launch.py
# 等价的无 launch 方式（config_path 可指向自定义 hba.yaml）
ros2 run hba hba_node --ros-args -r __ns:=/hba -p config_path:=<install>/share/hba/config/hba.yaml

# maps_path 必须是由 /pgo/save_maps(save_patches: true) 产生的目录，
# 且同时包含 poses.txt 与 patches/：
#   - poses.txt  每行 "<patch 文件名> tx ty tz qw qx qy qz"
#   - patches/*.pcd  对应的 body 系关键帧点云
ros2 service call /hba/refine_map interface/srv/RefineMap "{maps_path: '$HOME/flyos-universe-ros2/data/<map_name>'}"

# 保存优化后的位姿(HBA 写入的是位姿而非点云地图)
ros2 service call /hba/save_poses interface/srv/SavePoses "{file_path: '$HOME/flyos-universe-ros2/data/<map_name>/optimized_poses.txt'}"
```

注意事项（2026-09 实测）：
* `refine_map` 只负责**载入**地图并置位一个标志，真正的 HBA 迭代在与服务回调不同的
  100 ms 定时器里跑（`hba_iter` 次），期间持有服务互斥锁。所以服务返回 `success=True`
  只代表“已载入”，**不代表优化结束**：必须等节点日志出现 `END OPTIMIZE` 再调用
  `save_poses`，否则写出的还是未优化的原始位姿。
* `optimized_poses.txt` 每行 7 个数 `tx ty tz qw qx qy qz`，顺序与 `poses.txt` 一一对应；
  重建优化后的地图要用 `patches/` + 这两个位姿文件按行配对，包内已带该工具：
  `ros2 run hba rebuild_map_from_poses.py --map-dir <map_name> --poses <map_name>/optimized_poses.txt --out <map_name>/refined_map.pcd`
* HBA 只改位姿、不改点云分辨率：它能减小局部面片重影（map thickness），不能提高地图分辨率，
  也不会闭合全局回环（窗口是连续窗口，`window_size`/`stride` 决定）。

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
## T2.1 集成候选（分支 `algo/integration`，尚未合入 main）

本工作区 `wt/integration` 的 `algo/integration` 是 WP2 T2.1 的集成候选：以 main `0a7f513` 为基，
按补丁逐个挑选并修正评审问题。`git patch-id` 证明 `algo/loop` 的 `89134bf`/`1569591` 与
`algo/lio-drift` 的 `e1de4f6`/`3d7d388` 补丁完全相同，因此只合入一份。

| 集成内容 | 来源提交（本分支） | 说明 |
| :--- | :--- | :--- |
| 点平面质量门 + 实测重力（上游修复） | `393a153` ← `e1de4f6` | 恢复上游 `s > 0.9` 门限；`gravity_align` 采用实测 |g| |
| 静态窗 IMU 初始化 + 更密扫描滤波 | `83a8b7b` ← `3d7d388` | `imu_init_window_s` 静窗判定与整段缓存噪声 |
| `det_range: 100` | `4cb7999` ← `5b855d8` | 与上游 mcdviral 一致；逐序列量程仍由 manifest 覆盖 |
| PGO 多种子回环 + 可信性门控 | `d87b873` ← `759d19b`、`38335e8` ← `c920684` | 仅取回环提交（前端重复补丁不合入） |
| 重定位外点门控 / EMA / 有界 ICP | `3a5894e` ← `1216e46` | 见下方参数说明 |

**集成期修正（评审 B1/B2/B3 与 N1–N5、NB1/NB2）**：`c43b428`、`3587b2d`、`4aad2ea`、`58884b0`、`135b075`

* B1 回环实测偏航角改用 `pgo_loop::yawZ`（`atan2(R10,R00)`）与 `withYawAboutZ`，不再用
  `eulerAngles(2,1,0)`（Eigen 会把第一个角折到半区间，−1° 会读成 +179°，同向重访会被误拒、
  180° 翻转反而可能通过）。
* B2 局部定位门控抽成 `localizers/outlier_gate.h`：increment 量在**机体系**位姿上比较（不再随
  里程计原点距离退化）、参考值是**上一次被接受的原始解**（不是 EMA）、并且是**有界**的——
  `gate_max_consecutive_rejects` 次被拒后判定 INVALID 并按当前候选重播种且**继续门控**，
  之后连续 `gate_recovery_accepts` 次接受即恢复 valid。
  被门控**拒绝**的候选**永不会**成为发布出去的 `map→odom`：丢失期间发布值冻结在最后一次可信值
  （TF 本身没有有效标志位，下游可能忽略 `relocalize_check`），只有恢复到 valid 时才**直接释放**
  到恢复后的位姿；跟踪用的内部参考量（也是下一次 ICP 的初值来源）与发布值分离。
* B3 IMU 静窗初始化：最长等待（`imu_init_max_wait_s`）回退**只按时间**触发，不再额外要求陀螺
  安静（否则"振动但不旋转"的平台会永远初始化失败并让缓存无限增长）。
* N1 重力判定用**所选窗口**的加速度离散度与配置阈值；N2 状态从窗末传播到最新 IMU 采样
  （不再带着 v=0 直接跳到当前时刻）；N3 局部定位打分用米制对应半径 + 内点比例（PCL 的
  `getFitnessScore` 参数是**平方距离**）；N4 相对平移按**矢量**比较；N5 粗配准位姿无条件写出。

### 参数单位与默认值（易错点）

* `fastlio2` 的 `point_quality_thresh`：点平面残差**质量评分 s（无量纲）**，
  `s = 1 - 0.9·|点到面残差| / sqrt(|p|)`，上游 FAST-LIO2 门限为 `s > 0.9`。
  `lio.yaml` / `lio_highres.yaml` 均设 `0.9`；`commons.h` 里的 `0.1` 只是结构体默认值。
  它与 `esti_plane(points_near, 0.1, ...)` 中的 `0.1 m`（平面拟合距离）不是同一个量。
* `localizer` 的 `rough/refine_score_thresh`：落在对应半径内的内点**平均平方最近邻距离 [m²]**，
  取值范围上限即 `rough_max_corr_dist²=0.1225` / `refine_max_corr_dist²=0.0225`；
  `*_min_inlier_ratio` 为内点占整帧扫描的比例（PCL 的 `getFitnessScore` 看不到部分匹配）。
  仓库默认值取已验证 TIERS 运行使用的 `2.0 Hz / 8,15 次迭代 / 0.08,0.02`。
* `localizer` 门控：`gate_max_translation_m` / `gate_max_angle_rad` 是每次 ICP 更新相对上次被接受
  原始解的机体系位姿增量上限 [m] / [rad]；`gate_ema_alpha` 只影响发布出去的 `map←odom`。
* `pgo` 的 `max_loop_correction_m`、`min_odo_correction_m`、`cross_seed_max_m` 都是**矢量**量
  （分别为里程计相对平移与实测相对平移之差、其模长下限、两种子实测相对平移之差）。

### 运行模式与验证边界

* `map→odom` 有两个**互斥**生产者：`pgo_node`（建图，图优化偏移）与 `localizer_node`（先验地图
  定位）。二者同时运行会给同一 TF 对产生两个发布者，**不要**在同一 ROS_DOMAIN_ID 同时启动；
  `lio_node` 只发布 `odom→body`。
* 已验证：三包 Release 拷贝安装构建通过；4 个确定性 gtest 目标（`test_imu_init`、`test_gate_math`、
  `test_localizer_gate`、`test_scan_context_yaw`）全部通过；两种模式各 ≤20s 传感器时间的实机冒烟
  （lio+pgo 建图、lio+localizer 定位）进程健康、位姿有限且推进、服务与地图保存成功、无重复 TF 发布者。
* **未验证**（资源受限，需一次完整回放）：ATE/RPE 指标重算、回环门控在真实重访上的表现；
  `c920684` 的门限是在 `det_range=40` 前端上标定的，而集成前端为 `100`；N4 的矢量修正量与
  N3 的米制评分/内点比例门限都需要用真实回放重新标定。上述冒烟**不构成厘米级精度结论**。

## FAST-LIO既存研究

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
3. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
4. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
5. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
6. [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM#for-livox-lidar): A [Scan-Context](https://github.com/irapkaist/scancontext) loop closure module that can directly work with FAST-LIO.
