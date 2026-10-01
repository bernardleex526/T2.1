# WP2 T2.1 激光惯性里程计与高精度定位建图系统

本项目为面向复杂场景与移动机器人底盘的激光-惯性 SLAM（LiDAR-Inertial SLAM）与高精先验地图定位系统，基于 ROS 2 Humble 开发。系统集成了误差状态迭代卡尔曼滤波（ESIKF）惯性里程计、多重位姿图优化（PGO/Scan-Context）、全局地图一致性优化（HBA）、具有创新门控与恢复状态机的点云重定位跟踪器，以及支持旋转杆臂补偿的有界因果位姿外推预测器。

本文档完整阐述系统的算法架构、ROS 2 软件包拓扑、环境依赖、干净克隆构建流程、三档位姿输出模式及参数界限、仿真生成与评测复现流程、交付与排除边界，以及核心验收指标与工程整改计划。

---

## 一、算法架构与软件包结构

系统采用解耦的 ROS 2 架构设计，涵盖传感器前处理、前端里程计、后端全局优化、先验地图重定位与位姿发布，以及底盘控制与规划网关：

```
                    +------------------------------------+
                    |  Sensor Input (LiDAR, IMU, Cam)    |
                    +-----------------+------------------+
                                      |
                         [/livox/lidar, /livox/imu]
                                      v
+----------------------------------------------------------------------------------+
| 前端感知层 (Sensing Layer)                                                        |
|   - fastlio2: 21-D 误差状态迭代卡尔曼滤波 (ESIKF) 惯性里程计 + ikd-Tree 点云着色  |
|   - lidar_preprocessor / pointcloud_processor: 点云去噪、运动去畸变与ROI裁剪     |
|   - octomap_server2 / pcd2grid: 2.5D 占用栅格与 2D 投影转换                      |
|   - 输出 TF: odom -> body                                                        |
+----------------------------------------------------------------------------------+
                                      |
                      [/world_cloud, /lio_node/odom]
                                      v
+----------------------------------------------------------------------------------+
| 后端建图与一致性优化层 (Optimization Layer) - 【建图模式生效】                    |
|   - pgo: 基于 GTSAM iSAM2 的全局位姿图优化，支持 Scan-Context 多种子回环检测     |
|   - hba: 分层光束法平差 (Hierarchical Bundle Adjustment) 地图重叠消除与位姿精化  |
|   - 建图发布 TF: map -> odom                                                     |
+----------------------------------------------------------------------------------+
                                      |
                           [Prior Map: map.pcd]
                                      v
+----------------------------------------------------------------------------------+
| 定位与因果预测层 (Localization Layer) - 【定位模式生效】                         |
|   - localizer: 基于 NDT/ICP 的先验地图重定位，具备创新门控与 LockValidity 状态机 |
|   - robot_pose: 因果位姿预测器，支持 SO(3) 差分角速度外推与旋转杆臂合成          |
|   - 定位发布 TF: map -> odom                                                     |
+----------------------------------------------------------------------------------+
                                      |
                              [/robot_pose_map]
                                      v
+----------------------------------------------------------------------------------+
| 控制与规划层 (Planning & Control Layer)                                           |
|   - robot_2d_navigation: Nav2 导航适配 (DWB / MPPI / Smac Planner 插件)          |
|   - cmd_vel_smoother / g1_movement_gate: 速度平滑滤波与底盘运动安全网关          |
+----------------------------------------------------------------------------------+
```

### 1. 核心 ROS 2 软件包清单

仓库软件包拓扑可通过 `colcon list --base-paths src` 查询，核心算法与支持模块如下：

- `fastlio2` (`src/sensing/fastlio2`)：激光-惯性里程计前端。维护 21 维系统状态（位置、姿态、速度、陀螺零偏、加表零偏、重力向量）；内置 ikd-Tree 动态点云索引；支持外部相机输入进行点云投影着色。
- `pgo` (`src/optimization/pgo`)：位姿图优化节点。集成 GTSAM iSAM2 平滑求解器与 Scan-Context 激光扫描环描述子，提供多重闭环检测与矢量一致性门控。
- `hba` (`src/optimization/hba`)：一致性地图优化节点。通过局部点云面片与位姿残差迭代优化消除建图重影，提升全局地图一致性。
- `localizer` (`src/localization/localizer`)：先验点云地图定位节点。集成配准算法（ICP/NDT）；实现两级创新量门控（`outlier_gate.h`）与锁有效性状态机（`lock_validity.h`），杜绝错配发散。
- `robot_pose` (`src/localization/robot_pose`)：全局位姿综合与因果预测节点。支持从 `map` 到 `base_link` 的完整 TF 链路计算，提供针对当前时钟的 SO(3) 差分有界因果预测（`map_pose_current.hpp`）。
- `interface` (`src/common/interface`)：系统统一自定义服务与消息接口（`Relocalize`, `IsValid`, `RefineMap`, `SaveMaps`, `SavePoses`, `SaveColoredPcd`）。
- `universe_launch` / `sensing_launch`：系统顶层编排与集成启动包，管理传感器驱动、建图、重定位与导航节点拓扑。
- `g1_movement_gate` / `cmd_vel_smoother`：底盘运动状态安全门控与速度平滑器。
- 支持包：`lidar_preprocessor`, `pointcloud_processor`, `octomap_server2`, `octomap_msgs`, `pcl_msgs`, `pcd2grid`, `robot_2d_navigation`, `task_server`, `g1_multi_goal_manager`。

---

## 二、系统依赖与干净克隆源码构建

为了确保代码交付的独立性与可重现性，**构建命令采用标准 ROS 2 源码安装，严禁使用任何本地历史 artifact overlay 路径**。

### 1. 软件环境与核心外部依赖
- **操作系统**：Ubuntu 22.04 LTS (x86_64 或 aarch64)
- **ROS 2 版本**：Humble Hawksbill
- **编译工具链**：CMake 3.22+, `colcon` 构建工具
- **第三方关键依赖（按实际 CMakeLists.txt / package.xml 声明）**：
  * **GTSAM**：全局图优化必需（`pgo` 依赖，可通过 PPA 或系统源安装）
  * **Sophus**：李代数基础库（`fastlio2` 依赖）
  * **PCL** (`ros-humble-perception-pcl`) 与 **Eigen3**
  * **Livox-SDK2**：激光雷达驱动底层 SDK（官方 GitHub: `https://github.com/Livox-SDK/Livox-SDK2`）
  * **Octomap** 与 **Nav2 插件**

### 2. 标准源码构建流程

```bash
# 步骤 1：克隆开源代码仓库到本地工作区
git clone https://github.com/bernardleex526/T2.1.git
cd T2.1

# 步骤 2：安装编译系统与系统级依赖
sudo apt update && sudo apt install -y python3-colcon-common-extensions python3-pip
sudo add-apt-repository ppa:borglab/gtsam-release-4.2 -y
sudo apt update && sudo apt install -y \
    libgtsam-dev libgtsam-unstable-dev \
    ros-humble-perception-pcl \
    ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server \
    ros-humble-nav2-bringup ros-humble-spatio-temporal-voxel-layer \
    libeigen3-dev

# 步骤 3：源码构建并安装 Livox-SDK2 (livox_ros_driver2 必需)
git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
cmake -B /tmp/Livox-SDK2/build -S /tmp/Livox-SDK2
cmake --build /tmp/Livox-SDK2/build -j$(nproc)
sudo cmake --install /tmp/Livox-SDK2/build

# 步骤 4：源码构建并安装第三方依赖 Sophus 1.22.10 (fastlio2 / hba 必需)
git clone https://github.com/strasdat/Sophus.git /tmp/Sophus
git -C /tmp/Sophus checkout 1.22.10
cmake -B /tmp/Sophus/build -S /tmp/Sophus -DCMAKE_BUILD_TYPE=Release -DSOPHUS_USE_BASIC_LOGGING=ON
cmake --build /tmp/Sophus/build -j$(nproc)
sudo cmake --install /tmp/Sophus/build

# 步骤 5：标准编译本仓库全部 ROS 2 软件包 (纯净构建，无本地覆盖层)
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF

# 步骤 6：激活工作区环境变量
source install/setup.bash
```

---

## 三、三模式位姿输出、启动命令与参数界限

定位系统针对不同时间戳契约提供了三档位姿输出模式。不同模式在位姿时间戳与因果组合上有本质差异：

### 1. 三模式行为对比与实测表现

| 输出模式 | 时间戳定义与计算逻辑 | 外推计算与因果性质 | 实测当前时钟 ATE | 实测发布戳 ATE | 模式说明 |
|:---:|:---|:---|:---:|:---:|:---|
| **`stamp`**<br>(默认缺省模式) | **解析 TF 真实戳**。由 TF 链在 `TimePointZero` 处解析，等于链路上较旧变换（如 `map->odom` 配准广播）的真实时间戳。 | **无外推**。发布真实观测时间戳的位姿，保留观测管线固有延迟（观测管线时延中位约 0.27–0.30 s）。 | **0.1878 m**<br>(延迟导致) | 0.0181 m | 缺省行为。发布位姿真实标明其测量时刻，不捏造未来位姿；但在以当前查询时钟为真值的考核下存在由于时延引起的误差。 |
| **`current`** | **保留里程计测量戳**。发布最新接收到的里程计自身的测量时间戳。 | **组合无外推**。将最近一次采纳的 `map->odom` 修正与最新接收的里程计位姿直接刚体组合，**不进行速度积分外推**。 | **0.0482 m** | 0.0180 m | 保留里程计高频时间戳但未外推位移，移动工况下受制于里程计到达时延。 |
| **`predict`**<br>(synthetic opt-in) | **当前系统时钟戳**。与查询时刻时钟完全对齐。 | **有界因果预测**。基于两里程计姿态差计算 SO(3) 差分角速度，结合恒速平移与旋转杆臂向前外推。 | **0.0203 m**<br>(达成 $\le 0.05$ m) | 0.0203 m | **冻结合成仿真环境下评估的因果预测 opt-in 模式**。后锁及时输出率达 99.53%，通过全套时钟故障注入测试。 |

### 2. 模式启动命令与节点参数

- **生产编排启动（`universe.launch.py`）**：
  `universe.launch.py` 编排整套系统并启动默认位姿发布节点。
- **仿真评测运行器启动（`run_localization_sim.sh`）**：
  仿真环境通过 runner 脚本 `--consumer-mode` 参数驱动不同输出模式（详见第四节）：
  ```bash
  # 模式 1：默认 stamp 模式
  bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc_stamp \
      --bag-dir /tmp/sim_bag_loc --prior-map /tmp/sim_map/frozen_map.pcd \
      --consumer-mode stamp --skip-bag

  # 模式 2：current 模式
  bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc_current \
      --bag-dir /tmp/sim_bag_loc --prior-map /tmp/sim_map/frozen_map.pcd \
      --consumer-mode current --skip-bag

  # 模式 3：predict 模式 (synthetic opt-in)
  bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc_predict \
      --bag-dir /tmp/sim_bag_loc --prior-map /tmp/sim_map/frozen_map.pcd \
      --consumer-mode predict --skip-bag
  ```
- **独立运行 `robot_pose/map_pose_publisher` 节点命令行参数**：
  ```bash
  # stamp 模式 (默认)
  ros2 run robot_pose map_pose_publisher

  # current 模式
  ros2 run robot_pose map_pose_publisher --ros-args -p current_pose_mode:=true -p predict_current_pose:=false

  # predict 模式
  ros2 run robot_pose map_pose_publisher --ros-args -p current_pose_mode:=true -p predict_current_pose:=true
  ```

#### 关键参数界限 (Parameter Bounds)
- **里程计采样时间窗 $h$**：$h \in [0.02, 0.20]$ s。差分角速度由相邻两帧连续里程计求解：$\boldsymbol{\omega} = \text{Log}(\mathbf{R}_{prev}^T \mathbf{R}_{latest}) / h$。$h < 0.02$ s 或 $h > 0.20$ s 均触发安全拒绝。
- **最大前向外推时间 $dt$**：$dt \in [0, 0.12]$ s。超出 0.12 s 的外推请求判定为 `prediction_dt_out_of_range` 并停止发布。
- **旋转杆臂组合顺序**：严格按照 $\text{map}\leftarrow\text{odom} \times \text{predicted}(\text{odom}\leftarrow\text{child}) \times \text{child}\leftarrow\text{base\_link}$ 执行。里程计姿态变化在杆臂乘积之前计算，确保底盘旋转时杆臂随之产生正确的角位移。
- **状态机有效性超时 `validity_timeout_s`**：`1.0` s。配准中断超过 1 秒，服务状态立即置 `valid=false`。
- **首次无人值守锁资格 `recovery_accepts`**：`10` 次。开机或失锁重捕获后，必须连续通过 10 次创新门检验才允许对下游发布有效位姿。

---

## 四、仿真生成、运行与评测复现流程

仓库包含纯 Python 合成数据生成器与端到端自动化仿真管线（位于 `simulation/`），无需任何外部数据集即可闭环验证建图与定位性能。

### 1. 运行环境与 ROS 域隔离
为了防止多节点或多进程在局域网内产生 DDS 话题污染，所有仿真运行**必须**配置独立的 `ROS_DOMAIN_ID` 与单机回环隔离：
```bash
export ROS_DOMAIN_ID=137
export ROS_LOCALHOST_ONLY=1
```

### 2. 推荐：一键端到端评测流水线
仓库提供了完整的全自动评测流水线脚本 `simulation/scripts/run_eval_pipeline.sh`，可全自动串行完成场景参考几何生成、建图仿真、坐标对齐、T1/T2 评测、定位仿真、T3 评测及结果汇总：
```bash
# 运行端到端流水线（可追加 --dry-run 仅检查命令与文件语法而不启动 ROS）
bash simulation/scripts/run_eval_pipeline.sh --out-root /tmp/sim_pipeline \
    --variants baseline --consumer-mode predict
```

### 3. 分步模块化仿真与评测复现

如果需要分步执行各环节，可遵循以下标准指令流：

```bash
# 步骤 1：生成场景几何参考基准、真值轨迹与评测 ROI (输出至 /tmp/sim_ref)
# 输出包含 reference_map.pcd、roi.json、gt_mapping.tum、gt_localization.tum
python3 simulation/synthetic_data/scene_reference.py --out-dir /tmp/sim_ref

# 步骤 2：分别生成传感器合成 bag 数据 (注：--check 参数仅做几何校验不写磁盘；实际生成 bag 请省略 --check)
# 生成 90s 主建图轨迹 bag
python3 simulation/synthetic_data/generate_test_bag.py --out /tmp/sim_bag_mapping \
    --trajectory simulation/synthetic_data/test_trajectory.json

# 生成 60s 独立定位轨迹 bag
python3 simulation/synthetic_data/generate_test_bag.py --out /tmp/sim_bag_loc \
    --trajectory simulation/synthetic_data/test_trajectory_localization.json

# 步骤 3：运行建图仿真并生成对齐的先验地图
bash simulation/scripts/run_mapping_sim.sh --run-dir /tmp/sim_map \
    --bag-dir /tmp/sim_bag_mapping --skip-bag

python3 simulation/scripts/make_sim_transform.py \
    --gt-tum /tmp/sim_ref/gt_mapping.tum --odom-tum /tmp/sim_map/odom.tum \
    --t0-epoch 0.0 --fit-until-s 30.0 \
    --apply-map /tmp/sim_map/map.pcd --out-map /tmp/sim_map/frozen_map.pcd \
    --out /tmp/sim_map/frame_chain.json

# 步骤 4：建图指标评测 (T1 精度 & T2 帧处理时延)
python3 simulation/scripts/test_t1_accuracy.py --run-dir /tmp/sim_map --ref-dir /tmp/sim_ref
python3 simulation/scripts/test_t2_speed.py --run-dir /tmp/sim_map

# 步骤 5：在先验地图上运行定位仿真 (选择 opt-in predict 模式，输入定位专属 bag)
bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc \
    --bag-dir /tmp/sim_bag_loc --prior-map /tmp/sim_map/frozen_map.pcd \
    --consumer-mode predict --skip-bag

# 步骤 6：定位指标评测 (T3 精度与因果预测可用率)
python3 simulation/scripts/test_t3_localization.py --run-dir /tmp/sim_loc --ref-dir /tmp/sim_ref
```

> **关于历史审计证据与外部评测工具的明确说明**：
> 1. 本项目的历史证据产物（如 80 次重复测试 raw bags、点云 PCD、TUM 轨迹、旧评测器归档）保存在本地 `artifacts/` 与 `.omp/` 路径下，受 `.gitignore` 保护不随公开代码仓库上传。交付文档中的历史证据路径为本地历史归档记录，而非干净克隆仓库中的预置文件。
> 2. 父工程历史评测工具未随本仓库上传；本仓库内 `simulation/` 目录下的纯 Python 仿真生成器与测试脚本构成自包含的验证闭环，用户在干净克隆环境中即可完整重现上述所有建图与定位评测。
---

## 五、验收指标与正式状态总表

在 2026-09-30 完成的终审验证中，所有 80 次逻辑运行（75 次主路径五臂消融 + 5 次留出路径）均采用标准流程串行执行完毕，所有 10 项前置准入门全绿。正式指标如下：

### 1. 80 运行大规模重复与消融实验指标表

| 实验臂 / 路径 | 运行次数与状态 | T1 建图 RMSE 均值 (m) | 95% Bootstrap 置信区间 (m) | T2 处理时延 p95 (s) | 成对配准差 95% CI (vs baseline) | 验收判定与统计结论 |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **baseline** (主路径) | **15/15 PASS** | **0.02768** | [0.02712, 0.02825] | 0.05228 | —— | **PASS**（单臂稳定达标，$\le 0.05$ m） |
| **c2_2** (主路径) | **15/15 PASS** | **0.02398** | [0.02385, 0.02413] | 0.05093 | **[-0.00421, -0.00319]** | **PASS**（成对差全低于 0；`ci_below_zero_no_equivalence_claim`，仅限本冻结合成环境下的数值改善，不声称等价或跨场景因果） |
| **c2_3** (主路径) | **15/15 PASS** | **0.02808** | [0.02736, 0.02877] | 0.05117 | **[-0.00051, 0.00146]** | **PASS / 无改进证明**（配对差置信区间跨越 0；严格判定为 `improvement: not_demonstrated`） |
| **c2_1** (主路径) | 14 PASS / **1 BLOCKED** | —— | —— | —— | —— | **BLOCKED**（`seed45/repeat2` 触发评测器 1/212085 域外单点阻断，`pass=null`） |
| **c2_combined** (主路径) | 14 PASS / **1 BLOCKED** | —— | —— | —— | —— | **BLOCKED**（`seed42/repeat3` 触发评测器 1/227138 域外单点阻断，`pass=null`） |
| **主路径总体** | **73 PASS / 2 BLOCKED** | —— | —— | —— | —— | **stable_meets_target = false**（因 2 次单点阻断，全通未成立） |
| **holdout 留出路径** | **5/5 PASS** | **0.02991** | [0.02836, 0.03145] | 0.06141 | —— | **PASS**（跨路径留出达标，明确界定为同场景跨路径，不声称跨场景泛化） |
| **80 运行总计** | **80 = 78 PASS, 2 BLOCKED, 0 FAIL**（10/10 准入门在所有 80 次运行中均为 true） |

### 2. 消费者定位服务关键指标表

| 评测协议与模式 | 当前时钟 ATE RMSE | 后锁及时输出率 | 全段及时输出率 | 旧 0.1s 协议可用性 | 最终验收结论 |
|---|:---:|:---:|:---:|:---:|:---:|
| **旧 T3 协议 (`0.1s` 修正龄期)** | 0.1878 m | 0.0000 | 0.0000 | **0.0000** | **FAIL**（实测观测管线延迟中位 0.27–0.30s，不满足 0.1s 协议，严禁伪造时间戳重戳） |
| **新预测服务 (`predict` 模式)** | **0.0203 m** | **0.9953 (99.53%)** | **0.8858 (88.58%)** | —— | **PASS**（ATE $\le 0.05$m 达标，后锁输出率 $\ge 0.95$ 达标，全段因启动期 6.62s 未达 95%） |

---

## 六、详细计划要求、当前问题及解决方案

详细规范全文见 [`docs/detailed_plan_requirements.md`](docs/detailed_plan_requirements.md)。核心要点总结如下：

### 1. 已执行 7 项真实验收与核验结论
- **Step 1（状态机修复）**：彻底移除了 `code==1` 无条件返回 valid 的历史旁路；首次自锁必须经受连续 10 次创新门检验（`recovery_accepts=10`）；实测首配准残差平移模长为 0.5136 m，未触达 1e-9 的 bit-exact 单位阵分支，该分支由单元测试回归闭环；正式测试中 localizer 26 例与 robot_pose 22 例（全仓共 48 例行为回归测试）全部通过。
- **Step 2（因果预测）**：实现 Cartographer SO(3) 差分角速度与恒速平移，严格补偿旋转杆臂，22 例单元测试全绿。
- **Step 3（协议分离）**：将旧 0.1s 修正龄期协议与新因果预测协议彻底拆分，旧协议如实维持 FAIL，新服务达成 0.0203 m。
- **Step 4（仿真接线）**：三模式独立目录运行对照，停里程计、停定位、时钟暂停、时钟回跳等四项故障注入全部安全闭环。
- **Step 5（80 重复运行）**：串行闭环 80 运行，78 PASS，2 BLOCKED。
- **Step 6（评测器复核）**：评测器 58 用例全部通过，撤回 R14 误称字段，以 66.20s 完整 GT 支撑为分母纠正时间覆盖率（58.98%）。
- **Step 7（历史准入门禁）**：维持 BLOCKED（0 次回放），守住不交包围盒与速度包线底线。

### 2. 下一阶段启动 6.62s / 全段 88.58% 瓶颈拆解与“不降资格不改分母”纪律
- **瓶颈拆解**：全段 1202 个时隙中，后锁段输出率高达 99.53%（1062/1067）；未达标时隙集中在系统启动期 6.62 s（涵盖 IMU 初始静止窗与首次建锁连续 10 次创新门检验等启动阶段）。
- **核心纪律**：
  1. **不改分母**：全段分母严守 1202 单元，**绝不通过剪除开机 6.62s 启动段来虚假凑成通过**；
  2. **不降资格**：**绝不削减连续 10 次创新门要求**，绝不放宽自锁门限；
  3. **下一阶段工作**：**首要任务是实测打点测量启动 timeline 的精确耗时分解**，严禁在未诊断前擅自扩大范围进行算法重构。

### 3. 旧 0.1s 协议测段延迟客观事实：严禁重戳
- 真实处理管线时延（雷达扫描采集、去畸变、配准优化、TF 通信）实测中位值存在 0.27–0.30 s，在当前管线延迟下无法满足旧 0.1s 修正龄期要求；
- **严禁作弊重戳**：绝对不允许通过伪造时间戳为当前时钟来作弊；旧协议实测 0.0000 即如实判定为 FAIL，作为不可篡改的历史事实存档。

### 4. 历史数据集准入闭环：独立变换 / 严格空间不相交 / 成员归因 / Worldscan 公平口径
- **阻断根因**：MCD TUHH night_09 数据集实测 89.94% 时间处于高动态超速（$> 1.0$ m/s，均速 1.56 m/s），且既有对齐矩阵仅基于时间段切分，实测拟合区与评测区 AABB 几何相交且 7.51% 采样点空间重访（$< 1.0$ m）；同时上游（90.63%）与 Fork（24.31%）存在大量图外未归因残余点。
- **准入规范**：必须基于独立来源或严格空间不相交（$\text{AABB}_{fit} \cap \text{AABB}_{eval} = \emptyset$）求解专属对齐矩阵，严禁擅自新增容差规避；对比基准必须统一定义为 0.05m 体素下采样的去畸变离线累积点云（Worldscan），严禁用上游 890 万累积原始点云与 Fork 优化点云做不公平比较。

### 5. 两个单点 Outside 阻断与 25 混合 Lint 缺陷的根因及精准解决方法
- **两个单点 Outside 阻断**：Step 5 中 `c2_1` 与 `c2_combined` 各有 1 次运行因整图二十多万点中恰好有 1 个点落在 ROI 支持区域外触发 fail-closed 阻断（`pass=null`）。该点超出距离未实测，严禁臆断根因或数毫米，**严禁私自增加边界容差以追回 PASS**；必须客观记录阻断事实，后续若调整门禁须经由正式工程评审。
- **25 个混合 Lint 缺陷**：包含 17 项 uncrustify（上游既有代码格式如 `commons.h`，以及本次改动的 `localizer_node.cpp` 等未格式化代码）、3 项 lint_cmake（尾部换行与缩进）、1 项 flake8（插件加载环境异常）、4 项其他格式。**解决方案**：在专用格式化分支集中执行 `ament_uncrustify --reformat` 与 `ament_lint_cmake --reformat`，并修复 Python 虚拟环境依赖，不与核心算法混淆。

### 6. C1 / 足式状态估计 / FastLIVO2 真实系统边界：严禁捏造实现
- **C1 受控干预**：仅为离线位姿图干预测试工具（残差由 6.196m 降至 0.153m），仅反映内部配准指标，**绝不冒充在线闭环精度**。
- **T1.4 足式状态估计**：本仓库仅提供外部输入必需满足的接口契约（Q1–Q5，机体系线速度、协方差、触地标志等），**仓库自身绝不捏造未实现的四足动力学或足式里程计观测模型**。
- **FastLIVO2 真实集成范围**：本仓库实际集成为 **FAST-LIO2（LiDAR-Inertial Odometry）前端 + 相机点云投影着色**，系统状态维度为 21 维，不包含相机外参及光度状态，**不存在视觉紧耦合残差**，严禁宣称为视觉-激光-惯性紧耦合 SLAM。

---

## 七、公开仓库上传范围与不随仓库提供项声明

### 1. 随代码仓库提交交付的内容
- `src/`：全部 ROS 2 软件包源码及标准构建文件；
- `simulation/`：纯 Python 合成仿真环境、生成器与测试套件；
- `docs/`：详细技术规范、交接文档与参数参考；
- `README.md` 与 `.gitignore`。

### 2. 明确排除且不随代码仓库提供的内容
- **海量仿真与评测中间产物**：本地测试生成的中间数据与大型结果（`artifacts/`、`.omp/` 目录）；
- **大型传感器原始数据与数据集**：MCD、HILTI、室内动捕等海量原始点云与数据集（`data/`、`datasets/`）；
- **编译缓存与日志**：`build/`、`install/`、`log/`、`__pycache__/`、`*.pyc`、`.pytest_cache/`；
- **大型二进制点云、数据包与轨迹**：`*.bag`、`*.db3`、`*.pcd`、`*.tum`、`*.ply`、`*.mcap`；
- **敏感信息与密钥**：私钥文件、密钥、环境变量文件（`*.pem`, `*.key`, `id_rsa`, `.env` 等）；
- **未经实车标定的硬件外参**：针对未提供实物标定的新型传感器配置（如 `lio_orin_nx.yaml` 中故意省略的 `ext_il`，必须由硬件板端实测标定注入，仓库不提供捏造外参）。

---

## 八、相关文档索引

- [WP2 T2.1 详细执行计划要求与问题解决规范](docs/detailed_plan_requirements.md)
- [WP2 T2.1 软件交接文档（去硬件）](docs/validation/software_handoff_20260930.md)
- [本地仿真与指标核验验收报告](docs/validation/local_simulation_acceptance.md)
- [FastLIVO2 接口与坐标契约](docs/validation/04b73f5_fastlivo2_interface.md)
- [算法参数定义、量纲与配置对照](docs/validation/04b73f5_param_notes.md)
- [工程部署方案与平台契约](docs/validation/04b73f5_p3_deployment.md)
- [Localizer 回归与门控诊断](docs/validation/04b73f5_localizer_regression.md)
