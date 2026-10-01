# P3 工程部署方案：Orin 平台、TF/时间/外参契约与四足适配（版本 04b73f5）

> **声明**：本文涉及的板端实测、硬件在环、驱动/标定、实时性与温度数据，**当前一律为【待板端验证】**。本轮零运行（完整回放 0 次，未构建、未启动节点），因此**不给出任何“生产环境保证改善”的结论**，也不把**无来源的同步精度指标**写成强制要求。

- 基线：`fork` @ `04b73f553918b1dcc751feac69c0a2834f194432`（`main`）。
- 取代关系：本文**显式取代**仓库外前版历史文档 `docs/validation/04b73f5_p3_deployment.md`（下称“前版文档”）；外层文档保持原样，纠正见 §8。
- **范围边界（重要）**：本文只覆盖**部署/接口/硬件前置条件**，不含精度结论与根因归因；主指标坐标链一致性与独立标定缺失归 `EvalContract`/证据审计负责，当前 **BLOCKED**。所有硬件项一律【待板端验证】，本文**不**宣称任何“生产环境改善/保证”。
- 证据与命令分类见 `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/audit/engineering_evidence/source_manifest.md`。
- **本次修订（DeploymentContract）**：① 修复 `universe_launch` 的 `body→base_link` 静态发布器 roll/pitch 槽位互换（改用 Humble 具名参数，C12；非零角度 scoped smoke 已验证）并新增 §2.3 第 5 条“不存在 body→lidar/imu TF”与 §10 非零倾角验证命令；② §4 删除部署 profile 中**臆造**的 `ext_il` 并区分仿真恒等；③ §5.4 登记 C2 实验开关（未验证，opt-in profile）；④ §7.1 明确 Airy/Odin1 **不是 Livox 产品**，§7.2 明确 T1.4 的里程计候选是**电机编码器/腿部运动学**而非轮式里程计且须独立资格评估；⑤ §8 增补 C11–C14；⑥ 全文 `file:line` 已按当前工作树重新核对（`lio_node.cpp`/`commons.h`/`imu_init.h`/`imu_processor.cpp`/`lio*.yaml`）。

---

## 1. Orin NX 部署依赖矩阵【待板端验证】

### 1.1 仓库自身声明的环境（来源：`README.md:7-8`）

- 标称环境：**Ubuntu 22.04 + ROS 2 Humble**；雷达 MID360 + 其内置 IMU；驱动 `Livox-SDK2` + `livox_ros_driver2`。
- 仓库已含 **arm64 章节**（`README.md:80` “build on G1 (arm64)”），但只有 **CycloneDDS/unitree_sdk2_python** 的构建说明——**没有** Orin/JetPack 相关步骤，也没有任何 ARM 构建产物或耗时记录。

### 1.2 依赖矩阵（逐项标注来源与状态）

| 依赖 | 用途 | 来源 | 仓库是否有版本钉死 | 状态 |
|---|---|---|---|---|
| ROS 2 **Humble** | 全部节点 | `README.md:7`、所有 `package.xml`（`ament_cmake`/`ament_python`） | 否（无 `.repos`/`rosdep` 锁定，`README` 只给 `apt install ros-humble-*`） | 【待板端验证】 |
| **GTSAM 4.2**（PPA `borglab/gtsam-release-4.2`，`libgtsam-dev`,`libgtsam-unstable-dev`） | `pgo`（ISAM2） | `README.md:22-25` | 否（PPA 版本随上游滚动） | 【待板端验证】arm64 是否有对应 PPA 包**未确认** |
| **Sophus 1.22.10**（源码构建，`-DSOPHUS_USE_BASIC_LOGGING=ON`） | `fastlio2` SO(3)/SE(3) | `README.md:39-46` | 是（tag `1.22.10`） | 【待板端验证】 |
| **Livox-SDK2** + `livox_ros_driver2` | 雷达驱动 | `README.md:48-58` | 否 | 【待板端验证】原文档含内网镜像 IP，公网环境应采用 GitHub 官方仓库（Livox-SDK/Livox-SDK2）源码构建 |
| **PCL** (`ros-humble-perception-pcl`) | 点云/ICP | `README.md:36` | 否 | 【待板端验证】 |
| **Eigen3 / tf2_eigen / tf2_ros** | 数学与 TF | `README.md:28` | 否 | 【待板端验证】 |
| **octomap / octomap_msgs / octomap_server / rviz 插件** | 2D 投影与可视化 | `README.md:31-33`、`sensing.launch.py:157-165` | 否 | 【待板端验证】 |
| **nav2 + 插件**（`spatio-temporal-voxel-layer`、`nav2-constrained-smoother`、`nav2-smac-planner`、`nav2-*`） | 导航栈 | `README.md:14-20` | 否 | 【待板端验证】 |
| **CycloneDDS 0.10.x**（源码构建，`CYCLONEDDS_HOME`） | `unitree_sdk2_python` 依赖 | `README.md:84-100` | 是（`releases/0.10.x`） | 【待板端验证】 |
| **unitree_sdk2_python**（`pip install -e .`） | G1 控制接口 | `README.md:60-76`、`README.md:263` | 否 | 【待板端验证】 |
| **realsense2_camera** | 相机（仅着色） | `README.md:139-141` | 否 | 【待板端验证】；且**未被任何仓库 launch 启动**（见 §8 C3） |

### 1.3 与“内层平台镜像”的潜在冲突【待核实】

前版文档给出「JetPack 5.1.x / 6.x（Ubuntu 20.04 / 22.04）+ ROS 2 Humble」的组合。需注意：
- 仓库自身要求 **Ubuntu 22.04 + Humble**（`README.md:7`）。
- JetPack 与 Ubuntu 的对应关系、以及 Humble 是否提供对应 aarch64 官方二进制包，属**外部事实**；本文不作断言，标 **【待核实】**：上板前必须确认目标 JetPack 的 Ubuntu 版本与 Humble 的官方支持矩阵，否则需源码编译或容器化方案。
- 因此**不得**把前版文档的 “JetPack 5.1.x + Humble” 直接当作已验证基线。

### 1.4 ARM 构建建议（**未在本轮验证**）

仓库**没有**任何 Orin 专用构建脚本或 CMake 交叉编译配置；下述命令形态来自前版文档，**未经验证**，仅作为待测方案模板（**不要在未审查资源上限时直接照抄**）：

```bash
# 【待板端验证】形态参考，非仓库脚本、非本轮实测
colcon build --parallel-workers <N> \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_CXX_FLAGS="-O3 -mcpu=cortex-a78ae"
```

- `-mcpu=cortex-a78ae` 与 `-march=armv8.2-a+crypto` 的合法性取决于**目标 SoC 与编译器版本**：**【待板端验证】**。
- 并行度 `N`（前版建议 2–3，理由是 OOM）在本轮**无任何内存测量**，属**未验证**经验值；板端应实测 `colcon build` 峰值 RSS 后再定。
- 仓库自身只给出 `colcon build` 的**通用**说明（`README.md:76-78`），未限并行度、未设 `-DAMENT_CMAKE_SYMLINK_INSTALL=OFF` 等选项。

---

## 2. TF 树与单发布者契约

### 2.1 事实：全仓只有 3 个 `TransformBroadcaster` 持有者

`grep -rl TransformBroadcaster` 结果（源码级，非运行证据）：

| 节点 | 广播的变换 | 代码位置 | 生效模式 |
|---|---|---|---|
| `lio_node` | `world_frame → body_frame`（配置值 `odom → body`） | 定义 `lio_node.cpp:547-569`（其中 `sendTransform` 在 `:562`），调用点 `:605` | **两种模式都跑** |
| `pgo_node` | `map_frame → local_frame`（`map → odom`） | `pgo_node.cpp:526-543` | 仅建图（`sensing.launch.py`） |
| `localizer_node` | `map_frame → local_frame`（`map → odom`） | `localizer_node.cpp:346-360` | 仅定位（`universe.launch.py`） |

- **互斥**：`sensing.launch.py:132-153` 同时起 `lio_node(mapping)` 与 `pgo_node(pgo)`；`universe.launch.py:83-107` 同时起 `lio_node(mapping)` 与 `localizer_node(localizer)`。**没有任何 launch 同时起 `pgo` 与 `localizer`** ⇒ 仓库设计的 `map→odom` 生产者是互斥的。
- `README.md:326-328` 明确记录该约定：“`map→odom` 有两个互斥生产者…不要在同一 ROS_DOMAIN_ID 同时启动；`lio_node` 只发布 `odom→body`”。
- `localizer` 的 child frame 会被**入站 odom 的 `frame_id` 覆盖**（`localizer_node.cpp:340-341`），因此 `map→odom` 的 child 名实际由 `lio_odom` 决定（本仓库 = `odom`）。
- 静态外参：`universe.launch.py:150-166` 用 `static_transform_publisher` 发 `body → base_link`，参数由 `body_base_{x,y,z,roll,pitch,yaw}` 给出，**默认全 0**（`universe.launch.py:57-66`）。
  - **已修复（本任务改动）**：位置参数形式是 `x y z yaw pitch roll parent child`，旧代码按 `x y z yaw roll pitch` 传入 ⇒ roll/pitch 槽位互换（默认全 0 时不可见，非零安装倾角即发布错误旋转）。现改用 Humble 支持的**具名参数** `--x/--y/--z/--roll/--pitch/--yaw/--frame-id/--child-frame-id`，顺序无关，歧义从根上消除；其余 launch 参数（含 `body_base_*` 默认值 `0.0`）语义不变。
  - **已验证（本轮 scoped smoke，只起 `tf2_ros`，非整机）**：以 `--roll 0.3 --pitch -0.2 --yaw 0.1` 发布，读回四元数 `(x,y,z,w)=(0.153439,-0.091158,0.064071,0.981856)`，与 `Rz(0.1)·Ry(-0.2)·Rx(0.3)` **精确一致**；互换解释会得到 `(-0.106021,0.143572,0.064071,0.981856)`，两者可区分 ⇒ 该检查是**判别性**的。命令见 §10.1。`ros2 launch <file> --show-args` 亦通过（exit 0，参数默认值不变）。

### 2.2 静态检查发现的风险（**待运行验证**）

| 风险 | 依据 | 状态 |
|---|---|---|
| `nav2_bringup` 可能再引入一个 `map→odom` 发布者 | `universe.launch.py:143-149` 包含 nav2 `bringup_launch.py`（未传 `slam`）；其参数文件 `nav2_smac_params2.yaml:1-4` 写了 `amcl: ros__parameters: enabled: false`，但**上游 nav2_amcl 并未声明 `enabled` 参数**（本仓库未 vendor nav2，`find` 无 `nav2_bringup` 源码）⇒ 该开关是否生效无法由源码判定 | **【待板端验证】**：启动后 `ros2 node info`/`ros2 topic info /tf` 核对 `map→odom` 发布者数量 |
| `robot_state_publisher`/`map_pose_publisher` 是否引入冲突 TF | `nav2_smac_params2.yaml:347` 有 `robot_state_publisher` 段；`robot_pose/src/map_pose_publisher.cpp` **只监听不广播**（`lookupTransform("map","base_link")` 后发 `/robot_pose_map`） | `map_pose_publisher` **不是** TF 发布者（源码已确认）；`robot_state_publisher` 广播的是 URDF 内部固定变换，需运行确认其 frame 不与上述链路重叠 |
| `octomap_server` 的 `worldFrameId` 与占用栅格 | `sensing.launch.py:23,157-165`：`octomap_frame_id='odom'`、`base_frame_id='body'`、`remappings=[('cloud_in','/mapping/world_cloud')]` | `octomap_server` **不是** `TransformBroadcaster` 持有者；但会按 `odom/body` 做投影，需运行确认无 TF 断链告警 |

### 2.3 契约条文（可由源码直接支撑的部分）

1. `map→odom` 在任一时段**只能有一个**发布者：`pgo_node`（建图）与 `localizer_node`（定位）不得同时运行。
2. `odom→body` 由 `lio_node` 唯一发布（定义 `lio_node.cpp:547-569`，调用点 `:605`）。
3. `body→base_link` 由 `universe_launch` 的静态发布器提供（`universe.launch.py:150-156`）；`sensing` 模式**没有**该静态变换（`base_frame_id='body'` 正因此，`sensing.launch.py:30` 注释）。
4. `localizer.local_frame` 不得依赖 YAML 值，因为它会被 odom 的 `frame_id` 覆盖。
5. **不存在** `body→lidar` / `body→imu` 传感器坐标系 TF：`body` 本身就是 IMU 机体系（`ext_il` 语义 `p_imu = R_il·p_lidar + t_il`，`lio.yaml:49`），全仓没有任何节点发布雷达到机体的静态变换，也没有消费者需要它。前版 P3 规格画的 `body (base_link) → lidar/imu` 树**在源码中不存在**；若上层确实需要传感器坐标系，必须用与估计器**同一份实测外参**发布，方向为 `body → <lidar frame>`，且不得写入未标定的数值（本轮已把此前复制 MCD 外参的静态 TF 启动文件删除）。

---

## 3. 时间同步契约

### 3.1 源码实际具备的时间防护（事实）

- 只做**单调性检查**：四个回调在 `timestamp < last_*_time` 时清空对应缓冲并打 `"... Message is out of order"`（IMU `lio_node.cpp:321-325`；LiDAR `:336-341`、`:350-355`；图像 `:364-369`）。
- IMU/图像的**对齐门限**是“IMU 是否已覆盖到目标时间点”（`lio_node.cpp:418,437-438`），不是时间差阈值。
- **不存在**：软同步时间差门限、PPS/PTP 状态读取、时钟漂移补偿、跨源时间戳校验（全仓 grep 无 `PPS`/`PTP`/`1588`；唯一相关字样是 Livox 驱动内部的 `kTimestampTypeGptpOrPtp` 枚举，`livox_ros_driver2/src/comm/comm.h:97`）。

### 3.2 契约条文

1. **必须假定三种流处于同一硬件时钟域**（LiDAR/IMU/相机）。代码既不校验也不纠正；错域会直接表现为乱序告警与缓冲清空。
2. 硬同步（PPS/PTP）是**部署前提**，不是代码能力；**不得**把任何具体同步精度（如“微秒级”“漂移 <100 µs”“纳秒精度”）写成代码提供的保证或强制要求——源码无此依据，也无本轮实测。
   - **Airy/Odin1 不是 Livox 产品**：Livox 驱动内部的授时/同步选项（`livox_ros_driver2` 的 `kTimestampTypeGptpOrPtp` 枚举等）**不能假定适用**。目标雷达/IMU 是否提供硬件授时、以何种方式、精度多少，一律以厂商文档为准并在目标机箱实测；本文不引用任何转述的精度数值，也不给出任何具体标定工具/命令。
3. 若需要“软同步门限 + 时间回拨日志”，必须作为**新增实现**立项（当前不存在），不能按前版文档当作既有功能。
4. 逐点时间语义必须满足 `curvature = 相对帧首毫秒偏移`（见接口文档 §4.2）。注意 `PointCloud2` 路径的零点是“**首个经距离过滤后保留的点**”（`utils.cpp:98-110`），**不一定等于消息帧头**：抽稀、距离过滤或驱动布局都可能使其整体平移 ⇒ 换雷达时必须逐项核实，**不得假定**自动成立。

---

## 4. 外参与四元数规范

| 项 | 事实（源码） | 部署要求 |
|---|---|---|
| YAML 顺序 | `[t_x,t_y,t_z,q_x,q_y,q_z,q_w]` | 写入时不要漏位/换位 |
| 构造顺序 | `Eigen::Quaterniond(ext[6], ext[3], ext[4], ext[5])` ⇒ 入参 `(w,x,y,z)`（`lio_node.cpp:254,272`） | 阅读源码时勿把 yaml 顺序当成构造入参顺序 |
| 归一化 | `norm()>0` 时无条件 `normalize()`（`lio_node.cpp:262,273`） | 允许 YAML 浮点截断导致的非单位四元数；**不要**用非单位四元数表达缩放 |
| 语义 | `p_imu = R_il·p_lidar + t_il`；`r_cl=R_lcᵀ, t_cl=-R_lcᵀ·t_lc` | 外参方向必须按此约定标定，反向会静默产生系统性误差 |
| 在线估计 | `esti_il` 配置存在，但 `initialize()` 无条件用配置写 `x().r_il/t_il`；`State` 含 `r_il/t_il` 且协方差块为 `1e-5`（`imu_processor.cpp` initialize 段） | 当前应视为**固定外参**：标定精度直接进入建图误差 |
| 相机外参 | `ext_lc` 仅用于着色投影，**不进入估计** | 视觉升级前不得据此声称外参已被在线优化 |
| 部署 profile | `config/lio_orin_nx.yaml` **不写** `ext_il`（也无 `ext_lc`）：仓库内不存在该雷达/IMU 的标定 | 首次部署必须实测后再填；缺键时 `loadParameters()` 只 `WARN` 并退回 `r_il=I, t_il=0`（`lio_node.cpp:266`），那不是可用外参 |
| 仿真 profile | 仿真启动文件在 `SIM_OVERRIDES` 里把 `ext_il` 覆盖为单位阵 | **仅因为合成点云本来就渲染在机体系**；这是**仿真专用**恒等，实机与验收中**绝不可**沿用（也不得作为任何实机标定值的来源） |
| 四足杆臂 | 仓库**没有**任何 `base_link→body` 之外的杆臂参数文件；`universe_launch` 的 `body_base_*` 是唯一可传的静态杆臂（默认 0） | 雷达/IMU 相对机体的杆臂必须量测后填入 `ext_il`（传感器链），`base_link` 杆臂填入 `body_base_*`；两者是不同链路，不要混填 |

---

## 5. 四足（动态工况）初始化与去畸变的行为与异常点

### 5.1 静态窗初始化（`lio.yaml` 生效路径）

- 判据：在缓存内找**最安静的 `imu_init_window_s` 连续窗**（打分只用**陀螺标准差**），窗内 `gyro_std < imu_init_static_gyro_std(0.005 rad/s)` **且** `acc_dev < imu_init_static_acc_dev(0.3 m/s²)` ⇒ `is_static`（`imu_init.h:125-139`）。
- 超时回退：缓冲时间 `>= imu_init_max_wait_s(5.0 s)` 即 `waited_out=true`，`ready = is_static || waited_out`，回退到“最安静窗”**无条件**继续（`imu_init.h:141-143`）。该回退**只按时间**触发，不再额外要求陀螺安静——这是针对“振动但不旋转”平台（四足典型）的修复（`imu_init.h:6-13`）。
- 重力：仅当 `8.5 < |a_mean| < 10.5` 且窗内 `acc_dev < 0.3` 才用实测 `|a|` 覆盖 `State::gravity`，否则保持 `9.81`（`imu_processor.cpp:99-101`，`ieskf.cpp:4`）。
- 窗末状态先按 `v=0` 假定，再**传播到最新 IMU 样本**（`imu_processor.cpp:124-138`），避免把“秒级静止”硬塞到当前时刻。

### 5.2 四足工况下的行为（可由源码推断，**未经运行观察**）

| 场景 | 代码行为 | 影响（[推断]） |
|---|---|---|
| 启动即小跑（无静止窗） | 等满 `imu_init_max_wait_s=5.0 s` 才回退 | 前 ~5 s **不产生任何输出**（`MapBuilder` 在 `IMU_INIT` 状态直接 return，`map_builder.cpp:13-18`）；该段数据永久缺失 |
| 同上，回退窗选“陀螺最安静” | 打分只看 gyro 标准差（`imu_init.h:129-135`），不看 accel | 振动但角速度平稳的窗会被选中；`v=0` 假定与实际运动不符 ⇒ 初速/重力方向误差 |
| 步态冲击导致 IMU 时序抖动 | 一旦 `timestamp < last` 即**清空整个 IMU 缓冲**（`lio_node.cpp:321-325`） | 该次清空后 `syncPackage` 会因 `imu_buffer` 空而反复 `return false`（`lio_node.cpp:427-428`），直到新样本到达；短时丢帧 |
| `lio_highres.yaml` 被选中 | 该文件**未设** `imu_init_window_s` ⇒ `0.0` ⇒ legacy 模式（`imu_init.h:103-108`） | 静窗判定**被静默关闭**，初始化只看样本数（`imu_init_num=20`）⇒ 起步运动会被当作静止 |
| 剧烈俯仰/角速度（Gallop/跳跃） | 去畸变用 `m_poses_cache` 的**倒序线性插值 + SO(3) exp 补偿**，每点 dt 由 `curvature` 决定（`imu_processor.cpp` undistort 段） | 若 `curvature` 语义错误（换雷达）或 IMU 频率过低，补偿失真；**未验证** |
| 无逐点时间的 PointCloud2 雷达 | `curvature = 0` ⇒ 不做扫描内去畸变（`utils.cpp:110-112`） | 四足高动态下点云“锯齿/分层”无法抑制 |

### 5.3 明确的“异常”登记（待板端验证）

1. `imu_init_window_s` 在两个 profile 之间**不一致**，会把四足初始化行为切换成完全不同的模式（`3.0 s 静窗判定` vs `legacy 计数`）。`lio_highres.yaml` 的头注释过去**未声明**这一差异，本修订已改写其头部把差异逐条列出（见参数文档 §1.1）。
2. `waited_out` 回退是否适用于“起步即运动的四足”：源码注释承认回退窗末 `v=0` 只是**假定**；四足场景下该假定错误的后果**未测**。
3. 图像分支的**时间语义**是“先按图像时间推进一次 IMU，再在帧尾做 LiDAR 更新”（`imu_processor.cpp` `propagate_time_end`）；相机缺帧/迟到会改变子周期划分 —— 对四足**未验证**。
4. 乱序缓冲清空的**频率**与四足振动强度的关系：**未记录**。

### 5.4 与四足强相关的**实验性**候选（均未验证，禁止当作已修复）

- `imu_init_mode: "first_batch"`（C2.2，首帧整批初始化，不等静窗/不超时回退）与 `imu_acc_normalize: true`（C2.1，加速度尺度归一化）是针对“起步即运动 + 实测 g 偏低”这两条**假设**的开关。它们只写在 opt-in 的 `config/lio_c2_experimental.yaml` 中；C2 消融回放尚未执行，因此**没有任何测量支持**把它们用于部署（见参数文档 §1.5）。
- `imu_init_static_gyro_std` / `imu_init_static_acc_dev` 是否应随四足振动水平放宽，同样**未验证**：放松判据可以让初始化更早就绪，但也可能把一个运动窗当成静止窗。
- `pcl2_time_field` 为空时的扫描内去畸变缺失，是四足高动态下最大的**已知**结构缺口（见 §3.2 第 4 条与接口文档 §4.2）。

---

## 6. 资源 / 温度 / 实时性计划（全部【待板端验证】，本项目前无任何测量）

**当前事实**：仓库内**没有** CPU 亲和性、实时优先级、内存上限、温度或端到端延迟的任何配置或测量记录（全仓无 `chrt`/`taskset`/`sched_setscheduler`/温度相关代码或脚本）。

| 项 | 计划（待测，非保证） | 测量方法与记录要求 | 现状 |
|---|---|---|---|
| 端到端单帧耗时 | 观测 `print_time_cost`（`lio_node.cpp:575-579`）输出的 `process()` 毫秒数 | 打开 `print_time_cost: true`，记录分布（中位/p95/max）而非单值 | **未记录** |
| 节点调度 | `lio_node` 独立核 + 实时优先级 | 见下方**纠正**：`chrt/taskset` 施加在 `ros2 launch` 父进程上、子进程一般继承但**不保证生效**；应改为 per-node `prefix`/cgroup 并实测核实 | **未记录** |
| 内存 | ikd-Tree/地图点规模与 RSS | 用高分辨率 profile 时特别关注（`lio_highres.yaml` 头注释给出 `[INFERENCE]` 级的 ~36× 点量估算，**非实测**） | **未记录** |
| 温度/降频 | Orin 长期满载热行为 | 需在目标机型、目标机箱、目标环境温度下实测 | **未记录** |
| 编译资源 | `colcon build` 峰值内存/并行度 | 见 §1.4 | **未记录** |

**纠正（对前版文档 §3）**：

```
sudo chrt -f 50 taskset -c 2,3 ros2 launch sensing_launch sensing.launch.py
```

- **技术事实**：`chrt`/`taskset` 作用于它们 exec 的进程（这里是 `ros2 launch` 的 Python 父进程），设置其调度策略与 CPU 亲和性掩码；Linux 下**子进程一般会通过 fork/exec 继承亲和性与调度策略**，所以该写法通常**确实能影响到**由 launch 派生的 `lio_node` 等节点——前版文档**并非原理性错误**。
- **但要纠正的是“保证”两个字**：继承**不是保证**，且该命令**不构成调度已按生效的证据**。可能改变结果的因素是**显式重置/显式覆盖**，而不是进程组本身：① 执行路径上存在显式 reset（如 `SCHED_RESET_ON_FORK`、包装器/节点自身调用 `sched_setscheduler`/`pthread_setaffinity_np` 覆盖）；② cgroup/systemd 的 CPU 配额与 `RLIMIT_RTPRIO` 限制会阻止或削弱实时策略；③ 即使亲和性/策略被继承，也**没有任何测量**证明节点实际以该优先级/核心持续运行（温度降频、抢占、迁移都会改变实际行为）。**注意：把节点放入独立进程组或另开会话（`setsid`/新进程组）并不会改变 fork/exec 对亲和性与调度策略的继承**——这一点前版隐含的担忧与我的初稿表述都不成立。
- **因此**：该行只能作为“待板端验证”的候选手段；要明确归属与可观测性，推荐用 launch 的 per-node `prefix`（如 `prefix='taskset -c 2,3 chrt -f 50'`）或 cgroup/systemd 单元，并**实测**验证：亲和性读 `/proc/<pid>/status` 的 `Cpus_allowed_list`；调度策略读 **`chrt -p <pid>`**（或 `/proc/<pid>/sched` 的 `policy` 字段）——`/proc/<pid>/status` **没有** `policy` 字段。**上述实测均未执行，属【待板端验证】**。

---

## 7. 外部驱动与轮足融合的前置条件

### 7.1 T1.2 传感器抽象层（Airy / Odin1 驱动）

**源码现状**：仓库只支持两类 LiDAR 输入——`livox`(CustomMsg) 与通用 `PointCloud2`（`lio_node.cpp:114-132`、`utils.cpp:9-113`）；IMU 只支持 `sensor_msgs/msg/Imu`（`lio_node.cpp:108-112`）。**没有任何** Airy/Odin1 相关驱动或标定代码。部署发起文件 `config/lio_orin_nx.yaml` 是本任务新增的**未验证起点**（话题名按 RoboSense 惯例取 `/rslidar_points`、走通用 `PointCloud2` 分支，逐点时间字段留空，`ext_il` 有意留空）。

**身份前提**：Airy / Odin1 **不是 Livox 产品**，因此 `lidar_type: livox`、`livox_ros_driver2` 及其同步/时间戳选项**不得假定适用**；P6（Livox 线数）对它们不适用。此前该部署文件里写的 `ext_il=[0.020,0.000,0.037,1,0,0,0]` 是从 MCD 回放配置（另一套 LiDAR/IMU）复制的**臆造外参**，已删除：外参只能来自本平台的实测标定。

| 前置条件 | 判定依据（源码） | 不满足的后果 | 现状 |
|---|---|---|---|
| P1 点云消息类型明确（CustomMsg 或 PointCloud2） | 分支二选一，由 `lidar_type` 决定（`lio_node.cpp:114-132`） | 类型不符则无输入 | **未验证** |
| P2 若为 PointCloud2：存在**逐点相对时间**字段，datatype ∈ {FLOAT32,FLOAT64} | `utils.cpp:49-62`：字段缺失或类型非浮点 ⇒ `off_time=-1` ⇒ `curvature=0` | **静默**丢失扫描内去畸变（无报错） | **未验证** |
| P3 时间字段量纲与 `pcl2_time_scale` 匹配（结果必须是**秒**） | `utils.cpp:98-110`（`(t-t0)*1000` 得毫秒） | 帧尾时间错误、IMU 断点错误 | **未验证**；Ouster 类绝对纳秒时间戳需 `t0` 基准与 `1e-9` scale |
| P4 IMU 加速度单位（g 或 m/s²）与 `imu_acc_scale` 匹配 | `imuCB` 直接乘 `imu_acc_scale`（`lio_node.cpp:327-329`） | 10× 系统误差（重力/加速度尺度） | **未验证** |
| P5 IMU 话题与频率、时间戳钟域与 LiDAR 一致 | `lio_node.cpp:418,437-438` 的对齐门 | 初始化/同步长期阻塞或乱序清空 | **未验证** |
| P6 若为 Livox 系：`lidar_max_line` 与线数匹配（严格 `<`） | `utils.cpp:10` | 线数超限的点被**静默丢弃** | 迁移时须确认 |
| P7 内参/外参重标定（`ext_il`，以及若用相机则 `ext_lc`+`cam_*`） | `lio_node.cpp:256-291` | 外参不进入估计，误差**不可被滤波吸收** | **未验证（无硬件与标定数据）** |

**结论**：T1.2 的交付物是**参数化配置 + 驱动话题适配 + P1–P7 的板端验证记录**；在 P1–P5 未确认前，**不得**给出 Airy/Odin1 的 `pcl2_time_field/scale`、`imu_acc_scale` 等“推荐值”（见 `04b73f5_param_notes.md` §4）。

### 7.2 T1.4 状态估计层（足式平台：电机编码器/腿部运动学速度，**不是**轮式里程计）

**源码现状（关键事实）**：`lio_node` 只订阅 IMU、LiDAR、Image 三类输入（`lio_node.cpp:108-137`，全仓无 `Twist`/里程计订阅）；`State` 21 维为 `(r_wi,t_wi,r_il,t_il,v,bg,ba)`（`ieskf.h:38-53`），**没有**任何车轮/足端运动学观测项；`lio_odom` 的 `twist` 是**输出**而非输入（`lio_node.cpp:522-524`）。控制侧 `g1_movement_gate` 只消费 `/cmd_vel_smooth`、`/robot_pose_map` 等（`g1_movement_gate.py:79-88`），**不产出**任何里程计。

**“里程计输入”在本平台的含义（必须写清）**：四足**没有车轮**，因此**不存在**“轮式里程计（wheel odom）”这一类输入；唯一物理上可能的机体速度来源是**关节电机编码器 + 腿部运动学/接触状态**推出机体系线速度（本仓库的 G1 控制链路见 `README.md:60-100` 的 `unitree_sdk2_python`）。这类里程计必须作为**独立标定与独立质量评估**的输入对待：先有自己的精度/时延/打滑特性测量，才谈得上融合——**不得**用“底盘提供速度”一句话替代该评估，也不得把它当作已存在的接口。

| 前置条件 | 说明 | 现状 |
|---|---|---|
| Q1 取得**机体系线速度**观测（带时间戳与协方差） | 候选来源是关节编码器/腿部运动学（`unitree_sdk2_python` 链路），**不是**轮式里程计；需要新订阅 + 新观测模型 | **未实现** |
| Q2 观测的**独立资格评估**（精度、时延、打滑/悬空、步态相位相关误差） | 必须先于融合单独测量并留档；否则无法区分“里程计误差”与“LIO 误差” | **未验证** |
| Q3 时间对齐（该观测时间戳与 IMU/LiDAR 同域；插值限间隔、不跨大缺口） | 与 §3 同源要求；现有代码无软同步机制 | **未实现** |
| Q4 零速/静止检测与步态耦合（足底接触相位） | 直接影响 `v=0` 假定与初始化（§5） | **未验证** |
| Q5 外参（机体/足端 → `body`）标定 | 与 §4 同源 | **未验证** |

**结论**：T1.2/T1.4 在当前版本**都不是“改配置即可”的项**：T1.2 主要是**配置 + 驱动适配 + P1–P7 板端验证记录**（部署 profile 见 `config/lio_orin_nx.yaml`，其中 `ext_il` 有意留空）；T1.4 需要**新增观测模型与状态扩展**（属算法变更，须另行授权），且必须先完成 Q2 的独立资格评估。

---

## 8. 对前版文档的纠正/取代清单

| # | 前版位置 | 前版陈述 | 事实 | 处置 |
|---|---|---|---|---|
| C1 | §一.1 | 目标环境 “JetPack 5.1.x / 6.x（Ubuntu 20.04 / 22.04）+ ROS 2 Humble”；依赖含 “TensorRT/CUDA” | 仓库只声明 Ubuntu 22.04 + Humble（`README.md:7`）；仓库无 CUDA/TensorRT 依赖，也**无** Orin/JetPack 构建步骤 | **降级为【待核实】**，并删除“TensorRT/CUDA 必需”的暗示（无源码依据） |
| C2 | §一.2 | `colcon build ... -mcpu=cortex-a78ae -DCMAKE_CXX_FLAGS=...`、`MAKEFLAGS="-j3"` | 仓库无该脚本；参数合法性与内存上限**无任何测量** | **改为待测模板 + 显式标注未验证** |
| C3 | §一.3 | `sudo chrt -f 50 taskset -c 2,3 ros2 launch ...` 可保障 `lio_node` 实时 | 该写法设置在 `ros2 launch` 父进程上；子进程**一般会继承**亲和性/调度策略，故并非原理错误，但**继承不等于生效**、更不构成已生效证据 | **纠正“保障/保证”的措辞**（§6 详列不确定性 + 建议 per-node prefix/cgroup + 实测方法） |
| C4 | §二.2 | “若时间差 > 10 ms，系统记录警告；时间回拨时自动清空缓冲并记录日志” | 源码**无** 10 ms 门限；只有 `timestamp < last` 的乱序清空与 `"... out of order"` 日志（`lio_node.cpp:321-325` 等） | **纠正**：软同步门限属未实现功能 |
| C5 | §二.2 | “必须接入 PPS/PTP，对齐至微秒级（漂移 <100 µs）” | 源码无同步实现，亦无实测；“微秒/100 µs”**无来源** | **改写为部署前提 + 禁止无来源精度承诺**（§3） |
| C6 | §二.1 | TF 拓扑图（建图 PGO 发 `map→odom`；定位 Localizer 发 `map→odom`；LIO 发 `odom→body`） | 与源码一致（§2.1） | **保留**，并补 `body→base_link` 静态变换与 nav2/AMCL 风险项 |
| C7 | §三.1 | `imu_init_window_s = 3.0s`；静止判据 `0.005 rad/s` / `0.3 m/s²`；重力支持 `8.5~10.5` | 与源码一致（`imu_init.h:139`、`imu_processor.cpp:100`） | **保留** |
| C8 | §三.2 | “逐点去畸变…倒序线性/样条插值…微秒级偏移” | 去畸变为倒序线性 + SO(3) exp，**无样条**；偏移单位是**毫秒**（`curvature`） | **纠正**（去掉“样条”与“微秒级”表述） |
| C9 | §三.3 | 门控“单步位移 >0.04 m 或旋转 >0.03 rad 直接拒绝；连续 10 次拒绝失锁…” | 与 `outlier_gate.h:50-59` 一致；但“10 次恢复”是 `recovery_accepts=10`，且**失锁期间仍继续门控** | **保留并补语义** |
| C10 | §四 表 | Airy/Odin1 驱动“需提供 CustomMsg 或 PointCloud2 含精确逐点相对时间字段” | 与 `utils.cpp` 的解析能力一致 | **保留**，并补 P1–P7 判定表 |
| C11 | §二.1 TF 树 | `body (base_link) → lidar / imu` | 源码中**不存在** `body→lidar`/`body→imu` TF（`body` 本身就是 IMU 机体系，`ext_il` 只在估计器内部使用） | **纠正**（§2.3 第 5 条） |
| C12 | §二.1（TF 方向隐含） | 未提及 | `universe.launch.py` 的静态发布器旧写法把 `body_base_roll`/`body_base_pitch` 传进了 `pitch`/`roll` 槽位（位置参数序为 `x y z yaw pitch roll`）⇒ 非零倾角时旋转错误 | **新增缺陷登记并已修复**：改用具名参数 `--roll/--pitch/--yaw/--frame-id/--child-frame-id`（`universe.launch.py:156-166`）；已用非零角度 scoped smoke 验证（§2.1、§10.1） |
| C13 | （04b73f5 之后的分支产物） | `config/lio.yaml` 曾写入 `imu_acc_normalize: true` / `imu_init_mode: "first_batch"` / `imu_init_min_samples: 40`，并在文件中称之为 C2 修复 | C2 消融回放**一次未跑**，无任何测量支持；现撤回为 legacy 缺省（`lio.yaml` 不含这三个键），开关移入 opt-in 的 `config/lio_c2_experimental.yaml` 并标注“未验证假设” | **纠正**（§5.4、参数文档 §1.5） |
| C14 | （分支产物） | `config/lio_orin_nx.yaml` 曾写入 `ext_il: [0.020, 0.000, 0.037, 1, 0, 0, 0]`，并附 `launch/tf_static_airy.launch.py` 静态发布器 | 该外参复制自 MCD 回放配置（另一套 LiDAR/IMU），不是 Airy 标定；静态发布器使用未标定值 + 仓库其它地方未使用的帧名 | **删除**：profile 不写 `ext_il`（缺键 ⇒ `WARN` + 恒等占位），静态 TF 启动文件已移除 |

---

## 9. 待板端验证清单（汇总）

| 类别 | 项 | 现状 |
|---|---|---|
| 平台 | JetPack/Ubuntu/Humble 组合可行性；GTSAM 4.2 的 aarch64 可得性；CycloneDDS/unitree_sdk2_python 构建 | 【待板端验证】 |
| 构建 | `colcon build` 并行度/峰值内存；`-mcpu` 参数合法性 | 【待板端验证】 |
| 运行 | `map→odom` 发布者唯一性（含 nav2/AMCL 是否引入第二发布者） | 【待板端验证】 |
| 时间 | 三源钟域一致性；乱序/丢帧率；逐点时间字段正确性 | 【待板端验证】 |
| 四足 | 起步即运动时的初始化时长与 `v=0` 假定误差；静窗判据在振动下的可达性；去畸变在高动态下的效果 | 【待板端验证】 |
| 资源 | 端到端单帧耗时分布、RSS、温度/降频、调度优先级绑定方式 | 【待板端验证】 |
| 融合 | T1.2 的 P1–P7；T1.4 的 Q1–Q5（含算法新增立项，及 Q2 的独立资格评估） | 【待板端验证】/【未实现】 |
| 标定 | `ext_il` 实测（当前部署 profile 缺键 ⇒ 恒等占位，**不可用**）；`body_base_{x,y,z,roll,pitch,yaw}` 实测（槽位缺陷已修，需按 §10.2 用非零角度复核） | 【未标定】 |
| 消融 | `imu_acc_normalize` / `imu_init_mode` / `imu_init_min_samples`（C2.1/C2.2）——回放未执行，**无测量结论** | 【未执行】 |
| 视觉 | 相机流接入、`image_topic` 与内/外参、着色正确性 | 【待板端验证】 |

**本文不构成任何“生产环境性能保证”**：所有硬件相关项均为待验证；所有数值若未标注来源即为“未记录”。

---

## 10. 最终验证命令（含**非零倾角**冒烟）

> 本轮**未**跑整机 launch/回放（预算 0）；以下是集成验证阶段要执行的命令，其中 10.1/10.2 是**判别性**检查（互换 roll/pitch 会给出不同的可测结果）。执行者须记录：命令、时间、`ROS_DOMAIN_ID`、`config_path`、提交哈希、二进制路径与哈希。

### 10.1 `body→base_link` 静态外参（非零角度，判别性）

```bash
source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
export ROS_DOMAIN_ID=<专用ID>; export ROS_LOCALHOST_ONLY=1

# 1) 用已知的非零姿态发布（具名参数，顺序无关；负值可用空格形式与 `=` 形式）
ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.2 --z 0.3 \
  --roll 0.3 --pitch -0.2 --yaw 0.1 --frame-id body --child-frame-id base_link &
# 2) 读回核对
ros2 run tf2_ros tf2_echo body base_link
kill %1
```

判据：`translation: ('0.100000','0.200000','0.300000')`，`rotation` 四元数 `(x,y,z,w) = (0.153439,-0.091158,0.064071,0.981856)`，即 `RPY = Rz(0.1)·Ry(-0.2)·Rx(0.3)`。若读回 `(-0.106021,0.143572,0.064071,0.981856)`，说明 **roll/pitch 又写反了**。
（本轮已按此命令做过 scoped smoke：实测值与期望完全一致，进程按 PID 结束，未用 blanket pkill。）

### 10.2 整机 launch 的非零倾角复核

```bash
ros2 launch universe_launch universe.launch.py data_dir:=<dir> map:=<name> \
  body_base_roll:=0.3 body_base_pitch:=-0.2 body_base_yaw:=0.1
# 另开终端：
ros2 run tf2_ros tf2_echo body base_link        # 期望与 10.1 完全相同的四元数
ros2 topic info /tf --verbose                   # 断言 map→odom 只有一个发布者（定位模式 = localizer_node）
```

判据：`body→base_link` 与 10.1 同值；`map→odom` 唯一；无 TF 断链告警。默认（全 0）时该检查**不具判别性**，必须传非零角度。

### 10.3 其余

沿用 §9 清单与 `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/audit/engineering_evidence/source_manifest.md` §4 的待验证命令（建图/定位冒烟、话题/服务/TF 唯一性、时间域与逐点时间字段、`ext_il` 标定）。
