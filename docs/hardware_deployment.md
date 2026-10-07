# Orin NX 硬件部署指南（T2.1：FastLIO2 + PGO + 先验地图定位）

> 适用平台：NVIDIA Jetson Orin NX 16GB + 速腾聚创 Airy/Odin1 + 四足机器狗 + ROS 2 Humble。
> 本文是**上机顺序与验收门槛**的实操文档，不是"已验证报告"。
> 相关文档：[标定流程](calibration_procedure.md)、[调参与排查指南](tuning_guide.md)、
> [四足适配说明](quadruped_adaptation.md)、[测试场景与统计口径](test_scenarios.md)、
> [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md)、
> [FastLIVO2 接口契约](validation/04b73f5_fastlivo2_interface.md)、
> [工程部署方案与平台契约](validation/04b73f5_p3_deployment.md)。

---

## 1. 本文的目的与诚实声明

### 1.1 目的

本仓库**不附带任何在目标机器人上实测得到的配置**。本文的全部作用是回答一个问题：

> **从一台装好 ROS 2 Humble 的 Orin NX 和一套未接线的 Airy/Odin1 + IMU 开始，
> 要按什么顺序、跑哪些命令、用什么判据，才能产出一份"可以上机"的配置？**

产出的配置是一份**实测记录**，不是本文能给出的东西。本文只给流程、命令与判据。

### 1.2 诚实声明（先读这一节）

| # | 事实 | 依据 |
|:---:|:---|:---|
| 1 | `src/sensing/fastlio2/config/lio_orin_nx.yaml` 是**未验证的部署起点**，文件头注释原文写明 "STATUS: unvalidated deployment starting point. NO value in this file was measured on the target robot"。 | 配置文件头注释 |
| 2 | 该文件**故意不完整**，并**会被 `tools/preflight_config.py` 判为 NOT READY**。这是设计意图，不是缺陷：一个缺失的键会挡住一次误把占位值当成实测值的上机。 | 见 §4 阶段 5 |
| 3 | 该文件**没有** `ext_il` 键。缺键时 `loadParameters()` 打印 `Missing or invalid ext_il parameter, using defaults`，估计器以 `r_il = I, t_il = 0` 运行。**那是占位符，不是外参。** | `lio_node.cpp` |
| 4 | 该文件 `pcl2_time_field: ""`，即**不做扫描内运动补偿**（每点 `curvature = 0`）。移动平台会看到点云分层/锯齿。 | `utils.cpp`、[时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) §3 |
| 5 | 该文件的 `na/ng/nba/nbg` 复制自**另一套传感器对**（MID360 + VN200），不是本平台 IMU 的噪声参数。 | 配置文件注释、`tools/imu_allan_variance.py` 头部 |
| 6 | **Airy/Odin1 不是 Livox 产品。** `lidar_type: livox`、`livox_ros_driver2` 及其同步/时间戳选项**不适用于它们**；本 profile 走通用 `PointCloud2` 分支。 | [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) §1 |
| 7 | 本仓库的 `lio_node` **不含任何时间同步实现**：只有"时间戳回退即清空缓冲"的单调性检查，没有软同步门限、没有 PPS/PTP 状态读取。 | [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) §0 |
| 8 | 本仓库**没有**板端 CPU / 内存 / 温度 / 实时性的任何测量记录。本文 §5 给出的目标值是**建议值**，不是本平台的实测结果。 | [工程部署方案与平台契约](validation/04b73f5_p3_deployment.md) §6 |
| 9 | 步态陷波滤波的**收益在本仓库数据上未被测量**。本文不声称它改善精度。 | [四足适配说明](quadruped_adaptation.md) §4 |

**本文不给出任何未标注来源的数值。** 全文出现的数值只有三类：**(a) 源码/工具事实**、
**(b) 本仓库既有的仿真口径数值（明确标注为仿真）**、**(c) 明确标注为"建议/待实测"的目标值**。
凡标 **待实测** 的格，必须由括号内指名的工具在目标平台上产出后才能填入验收文档。

---

## 2. 交付物清单与前置条件

### 2.1 上机前必须物理存在并已知的东西

| # | 项目 | 具体要求 | 不知道的后果 |
|:---:|:---|:---|:---|
| 1 | LiDAR | Airy 或 Odin1，机械安装完成（含减振） | 无法开始 |
| 2 | LiDAR 驱动包 | 厂商 ROS 2 驱动（RoboSense 为 `rslidar_sdk`），**已构建且能发布 `PointCloud2`** | 无点云输入 |
| 3 | LiDAR 驱动构建选项 | 必须知道 `POINT_TYPE` 的实际取值（`XYZI` / `XYZIRT`）。**这是决定能否做扫描内去畸变的开关**，见 §4 阶段 2 | 无法判断"没有逐点时间"是硬件事实还是配置错误 |
| 4 | IMU | 已安装并标定过轴系；已知其输出单位是 **m/s²** 还是 **g** | `imu_acc_scale` 错 → 10× 系统误差 |
| 5 | IMU 话题名与频率 | 例如 `/imu/data`，频率**待实测**（`ros2 topic hz`） | 初始化与同步长期阻塞 |
| 6 | 录包手段 | `ros2 bag record` 可用，且磁盘剩余空间足够（静止 10 min 的 IMU+LiDAR 包不是小文件） | 无法留证据、无法复现 |
| 7 | ROS 2 Humble | Orin NX 上已安装并可 `source /opt/ros/humble/setup.bash` | 无法开始 |
| 8 | 仓库已构建 | `colcon build` 通过，`install/setup.bash` 可用 | 无法开始 |
| 9 | Python 依赖 | `python3` + `numpy` + `PyYAML`（`tools/*` 需要） | 工具无法运行 |
| 10 | 温度/资源观测手段 | `tegrastats`（Jetson 自带） | 无法给出 §5 的记录 |

> **前置条件 3 是本节最容易被跳过的一条。** 速腾官方 ROS 2 SDK `rslidar_sdk` 的**默认**
> `POINT_TYPE=XYZI` 只发布 `x/y/z/intensity`，**根本没有逐点时间字段**。若驱动是用默认选项
> 构建的，那么"Airy 没有逐点时间"就是真实答案，而不是配置问题——此时唯一正确的动作是
> **改 SDK 构建选项重建驱动**，而不是在 YAML 里编一个字段名。

### 2.2 上机前检查表

工作目录：仓库根。每个会话先执行环境准备：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=137
export ROS_LOCALHOST_ONLY=1
```

| 检查项 | 命令 | 期望结果 |
|:---|:---|:---|
| 仓库工作区已构建 | `ls install/setup.bash` | 文件存在 |
| `lio_node` 可执行 | `ros2 pkg executables fastlio2` | 输出含 `fastlio2 lio_node` |
| 配置文件存在 | `ls src/sensing/fastlio2/config/lio_orin_nx.yaml` | 文件存在 |
| 预检工具可用 | `python3 tools/preflight_config.py --config src/sensing/fastlio2/config/lio_orin_nx.yaml` | **退出码 2**（出厂即 NOT READY，见 §4 阶段 5） |
| 工具自检通过 | `python3 -m pytest tests/test_probe_tools.py` | `51 passed` |
| LiDAR 驱动在跑 | `ros2 topic list \| grep rslidar` | 出现 `/rslidar_points` |
| LiDAR 有数据 | `timeout 10 ros2 topic hz /rslidar_points` | 稳定频率（Airy 标称 10 Hz，**实测为准**） |
| IMU 有数据 | `timeout 10 ros2 topic hz /imu/data` | 稳定频率，**记录实测值**（待实测） |
| 两流时间戳同域 | 见 §4 阶段 1 的单调性脚本 | 无 `NON-MONOTONIC` 输出 |
| 域隔离 | `echo $ROS_DOMAIN_ID` | 与本仓库其它实验不冲突（仓库仿真约定 `137`） |

---

## 3. 系统架构与话题 / TF 契约

### 3.1 输入（`lio_node` 订阅）

| 键 | 代码默认值 | `lio_orin_nx.yaml` | 类型 | QoS | 备注 |
|:---|:---|:---|:---|:---|:---|
| `imu_topic` | `/livox/imu` | `/imu/data` | `sensor_msgs/msg/Imu` | `SensorDataQoS()` | 只支持 `sensor_msgs/Imu` |
| `lidar_topic` | `/livox/lidar` | `/rslidar_points` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS()` | 由 `lidar_type` 决定消息类型；`pointcloud2` 走通用分支 |
| `image_topic` | `/camera2/camera/color/image_raw` | **未设**（用默认值） | `sensor_msgs/msg/Image` | `SensorDataQoS()` | 相机**未被任何仓库 launch 启动**；图像不进入估计，只影响 `color_world_cloud` |

### 3.2 输出（`lio_node` 发布）

发布名是**相对名**，实际话题名 = 命名空间 + 相对名。`lio_launch.py` 用命名空间 `fastlio2`；
`sensing.launch.py` 与 `universe.launch.py` 用命名空间 **`mapping`**。

| 相对名 | 类型 | 在 `namespace=mapping` 下 | 发布时机 |
|:---|:---|:---|:---|
| `lio_odom` | `nav_msgs/msg/Odometry` | `/mapping/lio_odom` | 每次 `process()` 成功后 |
| `body_cloud` | `sensor_msgs/msg/PointCloud2` | `/mapping/body_cloud` | 仅帧尾（`lidar_end`） |
| `world_cloud` | `sensor_msgs/msg/PointCloud2` | `/mapping/world_cloud` | 仅帧尾 |
| `color_world_cloud` | `sensor_msgs/msg/PointCloud2` | `/mapping/color_world_cloud` | 仅图像子周期（需相机） |
| `lio_path` | `nav_msgs/msg/Path` | `/mapping/lio_path` | 仅帧尾 |
| （服务，绝对名） | `interface/srv/SaveColoredPcd` | `/mapping/save_colored_pcd` | 服务名写成绝对名，不受命名空间影响 |

> **重要陷阱**：所有发布函数在 `get_subscription_count() <= 0` 时**直接 return**。
> 没有任何订阅者时节点**不发布**，`ros2 topic echo` 之外没有任何东西在监听时也看不到数据。
> 调试时不要把"没有数据"直接当成算法故障——先确认有订阅者（`ros2 topic hz` / `ros2 bag record`
> 本身都会建立订阅）。

> **话题名与工具默认值不一致（实测核对）**：`tools/odom_static_drift.py` 的 `--topic` 默认值是
> `/fastlio2/lio_odom`，它只在 `lio_launch.py`（命名空间 `fastlio2`）下正确。
> 用 `sensing.launch.py` / `universe.launch.py` 启动时，实际话题是 **`/mapping/lio_odom`**，
> 必须显式传 `--topic /mapping/lio_odom`。同理 `localizer.yaml` 订阅的也是
> `/mapping/body_cloud` 与 `/mapping/lio_odom`。

### 3.3 TF 契约

| 变换 | 发布者 | 说明 |
|:---|:---|:---|
| `world_frame → body_frame` | `lio_node` | 配置值即 `odom → body`。这是 `lio_node` 唯一发布的 TF |
| `map → odom` | `pgo_node`（建图）**或** `localizer_node`（定位） | **两者互斥，不得同时运行** |
| `body → base_link` | `universe.launch.py` 的静态发布器 | 参数 `body_base_{x,y,z,roll,pitch,yaw}`，默认全 0 |

- **不存在** `body → lidar` / `body → imu` 静态 TF。`body` 本身就是 IMU 机体系，外参
  `ext_il` 只在估计器内部使用。
- `localizer` 的 `local_frame` 会被**入站 odom 的 `header.frame_id` 覆盖**，因此 `map → odom`
  的 child 名实际由 `lio_odom` 决定（本仓库为 `odom`）。

### 3.4 坐标与外参契约（写入前必须先确认）

| 项 | 契约 |
|:---|:---|
| YAML 书写顺序 | `ext_il: [x, y, z, qx, qy, qz, qw]` |
| 内部构造顺序 | `Eigen::Quaterniond(ext[6], ext[3], ext[4], ext[5])` ⇒ 入参是 **(w, x, y, z)** |
| 几何语义 | **`p_imu = R_il * p_lidar + t_il`** |
| 归一化 | `norm() > 0` 时无条件 `normalize()`，允许浮点截断导致的非单位四元数 |
| 在线估计 | `esti_il: false`（本 profile）⇒ **固定外参**，标定误差直接进入建图误差 |
| 逐点时间语义 | `curvature = 相对帧首毫秒偏移`；零点是**首个经距离过滤后保留的点**，不一定等于消息帧头 |

**方向写反会静默产生系统性误差，不会报错。** 标定方法见[标定流程](calibration_procedure.md) §1。

---

## 4. 部署顺序（分阶段，含每阶段验收门槛）

> 顺序不可跳。原因：若扫描内去畸变本身没工作（阶段 2/3 未完成），后面所有标定的效果都无法判断。
> 这条顺序与[标定流程](calibration_procedure.md) §0 一致。

### 阶段 0：编译与自检

```bash
# 0.1 构建（并行度按板端实测内存调整；仓库 README 用 4）
colcon build --parallel-workers 4 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash

# 0.2 C++ 单元测试（三个 gtest 套件）
colcon test --packages-select fastlio2
colcon test-result --verbose

# 0.3 Python 工具测试
python3 -m pytest tests/test_probe_tools.py
```

**验收门槛（全部满足才进入阶段 1）**

| 判据 | 期望 |
|:---|:---|
| `colcon build` | 无 error；warning 需逐条登记 |
| `colcon test --packages-select fastlio2` | 全绿，含 `test_gait_filter`、`test_imu_init`、`test_utils_preprocess` |
| `colcon test-result --verbose` | 无 failure |
| `python3 -m pytest tests/test_probe_tools.py` | 51 项全部通过（本仓库实测输出 `51 passed`） |

> 构建并行度 `4` 是仓库 README 的取值，**Orin NX 上的峰值内存未测量**。若 `colcon build` 被
> OOM killer 杀掉，先降并行度并记录峰值 RSS，而不是直接加大 swap。

---

### 阶段 1：单传感器验证（先证明两个传感器各自是好的）

```bash
# 1.1 LiDAR 频率（记录实测值，不要照抄标称值）
timeout 10 ros2 topic hz /rslidar_points

# 1.2 IMU 频率（这个值后面要填进 gait_filter_sample_rate_hz）
timeout 10 ros2 topic hz /imu/data

# 1.3 各取一帧看字段与时间戳
ros2 topic echo --once --field header /rslidar_points
ros2 topic echo --once --field header /imu/data
ros2 topic echo --once --field linear_acceleration /imu/data

# 1.4 录制 10 s 双流，供单调性检查
timeout -s INT 10 ros2 bag record -o sensors10 /rslidar_points /imu/data
```

时间戳单调性检查（`lio_node` 只做这一项时间防护，所以必须先确认输入本身是单调的）：

```bash
python3 - <<'PY'
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "sensors10"
TOPICS = ["/imu/data", "/rslidar_points"]

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))

last = {t: 0.0 for t in TOPICS}
n = {t: 0 for t in TOPICS}
while r.has_next():
    topic, raw, _ = r.read_next()
    msg = deserialize_message(raw, get_message(types[topic]))
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if t < last[topic]:
        print(f"NON-MONOTONIC {topic}: {last[topic]:.6f} -> {t:.6f}")
    last[topic] = t
    n[topic] += 1
for t in TOPICS:
    print(f"{t}: {n[t]} msgs, last stamp {last[t]:.6f}")
PY
```

**验收门槛**

| 判据 | 期望 | 不达标时 |
|:---|:---|:---|
| `/rslidar_points` 频率 | 稳定，记录实测值（Airy 标称 10 Hz） | 查驱动配置与网口带宽 |
| `/imu/data` 频率 | 稳定，**记录实测值**（待实测） | 查 IMU 驱动/串口波特率 |
| 时间戳单调 | 无 `NON-MONOTONIC` 行 | 驱动侧问题，先修驱动，不要上估计器 |
| 两流时间戳量级 | 同域（同为 epoch 秒，或同为相对时基） | 进入阶段 3，**不要**先调参数 |

---

### 阶段 2：逐点时间字段识别

```bash
# 2.1 对在线话题探测（--timeout 是等待一帧的秒数）
python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --timeout 10

# 2.2 或对已录的包探测，并直接打印可粘贴的两行
python3 tools/rslidar_pcl2_probe.py --bag sensors10 --topic /rslidar_points --emit-yaml

# 2.3 无 ROS 环境时：先在有 ROS 的机器上导出布局，再离线分析
python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --dump-fixture layout.json
python3 tools/rslidar_pcl2_probe.py --fixture layout.json
```

> **关于 `--once`**：该选项**已被工具正式接受**，且等价于默认行为——探针读取一帧即退出。
> 保留它是为了让仓库内各处文档中形如 `ros2 topic echo --once` 的惯用写法可以直接复制执行。
> 回归测试 `tests/test_probe_tools.py::TestPcl2Probe::test_once_flag_is_accepted` 与
> `test_every_documented_command_line_parses` 会校验文档中出现的每个选项都真实存在。

**工具判据（按实测值域，不按字段名猜）**：逐点时间字段的值域跨度必须约等于**一个扫描周期**
（Airy 10 Hz → 0.1 s）；单位由"哪个 10 的幂次能把跨度折算到约一个扫描周期"决定；
字段类型必须是 `FLOAT32` / `FLOAT64`。

**厂商事实（必须知道）**

| 构建选项 | 发布字段 | 是否有逐点时间 |
|:---|:---|:---|
| `POINT_TYPE=XYZI`（**`rslidar_sdk` 默认**） | `x, y, z, intensity` | ❌ **没有** |
| `POINT_TYPE=XYZIRT` | `x, y, z, intensity, ring, timestamp` | ✅ `timestamp`，`FLOAT64`，单位秒 |

**验收门槛与分支处置**

| 探针退出码 | 含义 | 必须做的动作 |
|:---:|:---|:---|
| **0** | 识别到可用逐点时间字段 | 把输出的两行写进部署 profile（`pcl2_time_field` / `pcl2_time_scale`），进入阶段 3 |
| **2** | 消息读到了，但**没有**可用的逐点时间字段 | ① 确认驱动 `POINT_TYPE` 的实际取值；② 若为默认 `XYZI`，**重建驱动**为 `XYZIRT`（改 SDK 构建选项，不是改 YAML）；③ 重建后重跑本阶段；④ 若确认硬件/驱动无法提供该字段，则**保持 `pcl2_time_field: ""` 并在验收文档中显式写明"不做扫描内去畸变"**，同时必须用 `--allow-no-time-field` 让预检显式承认这一点 |
| **1** | 输入读不了（话题无数据 / 包不存在 / 缺 ROS Python 包） | 先解决输入，不要解读为"没有时间字段" |

> **绝不要在 YAML 里写一个不存在的字段名。** `pcl2_to_PCL` 找不到该字段时会静默退化为
> `curvature = 0`（与留空等价），但配置文件看起来"已填"——这比留空更危险。
> `tools/preflight_config.py` 无法识别这种错误（它只能检查键是否存在），只有本阶段的探针能。

---

### 阶段 3：时间同步（LiDAR / IMU 进入同一时间轴）

**先读**：[时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md)。
结论先行：**本仓库的 `lio_node` 不含任何时间同步实现**，同步必须由硬件或节点外的手段完成。

```bash
# 3.1 记录两流的实际时间戳量级与差值（同域检查）
ros2 topic echo --once --field header.stamp /rslidar_points
ros2 topic echo --once --field header.stamp /imu/data

# 3.2 连续录制 >= 10 min，统计乱序告警（见阶段 1 的脚本，把时长与包名改掉）
timeout -s INT 600 ros2 bag record -o sync10 /rslidar_points /imu/data
```

两条可行路径（二选一，均**不改动本仓库代码**）：

1. **硬件授时**：让 LiDAR 与 IMU 进入同一硬件时间轴（如 PTP/触发授时）。
   是否存在该接口、支持哪些授时方式、精度多少，**一律以厂商文档为准**，
   并须在目标机箱上实测。**本仓库不认证任何具体工具或命令。**
2. **节点外改写时间戳**（推荐先验证这一路径，因为它不动本仓库代码）：在发布前改写消息的
   `header.stamp`，使两条流进入同一时间轴。整体常量偏移的估计方法见
   [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) §2。

**为什么必须做**：代码**假定**三流处于同一硬件时钟域，既不校验也不纠正。错域的直接表现是
`IMU Message is out of order` / `Lidar Message is out of order` 告警 + **整个缓冲被清空**；
清空后 `syncPackage()` 会因缓冲为空反复 `return false`，直到新样本到达（短时丢帧）。

**验收门槛**

| 判据 | 期望 | 备注 |
|:---|:---|:---|
| ≥ 10 min 双流录制中的乱序告警次数 | **0** | 出现即未通过 |
| 同步方式 | **必须记录**：硬件授时（方式/型号）或外部改写节点（名称/版本/偏移量） | 不记录 = 无法复现 |
| 实测同步误差 | **待实测**：本仓库**没有**任何同步精度测量工具，也**不给出任何精度指标**。误差必须由标定承担方提供的方法测出并留档 | 不得照抄厂商手册指标 |
| 交叉验证 | 阶段 6 的静止漂移中 `largest step` 在预算内（单步大跳变是时钟域故障的签名） | 见[调参与排查指南](tuning_guide.md) §6.3 |

> 逐点时间（阶段 2）解决的是**扫描内** 0.1 s 量级的畸变；本阶段解决的是 **LiDAR 与 IMU 之间**
> ms–s 量级的时钟域问题。**两者是不同的问题，不能互相替代。**

---

### 阶段 4：外参标定与 IMU 噪声

#### 4.1 IMU 加速度量纲自检（最便宜、最致命的一项）

```bash
# 机器狗上电、站立不动，录 30 s
timeout -s INT 30 ros2 bag record -o imu_static30 /imu/data
python3 tools/imu_allan_variance.py --bag imu_static30 --topic /imu/data
```

| 静止均值 `|a|` | `imu_acc_scale` | 说明 |
|:---|:---|:---|
| ≈ 9.81 m/s² | `1.0` | IMU 已输出 m/s² |
| ≈ 1.0 | `10.0` | IMU 以 g 为单位；代码默认 10.0 是为 MID360 内置 IMU 准备的 |
| 其它 | **停下** | 先查单位/轴系，量纲错了后面所有标定都无意义 |

工具会自己检查这一项：`mean |accel|` 不在 `(8.0, 11.0)` 区间时它**拒绝出结果**（退出码 2）。

#### 4.2 IMU 噪声 `na / ng / nba / nbg`（Allan 方差）

```bash
# 建议 6–12 h 静止录制（白噪声区需要短 τ，随机游走区需要长 τ）
# 要求：机器狗上电、站立不动、保持正常工作温度
timeout -s INT 7200 ros2 bag record -o imu_static /imu/data
python3 tools/imu_allan_variance.py --bag imu_static --topic /imu/data \
    --plot allan.png --emit-yaml
```

**关键行为（不是工具故障）**：工具只输出**统计上可辨识**的系数（> 2 倍标准误）。
随机游走 `nba/nbg` 的可观测性由交叉点 `τ_c = √3·N/K` 决定；若 `τ_c` 超过录制长度，
工具会诚实地报 **`NOT IDENTIFIABLE`**（`--emit-yaml` 输出 `# <key>: NOT IDENTIFIABLE - record longer and re-run`）。
此时正确的做法是**保留现值并标注"未标定"**，或延长记录时间；**不要**把拟合噪声当参数写入配置。

**验收门槛**

| 判据 | 期望 |
|:---|:---|
| 静止门控 | 通过（工具拒绝非静止数据，退出码 2；`--force` 只能用于观察曲线形状，其结果**不是**传感器噪声参数） |
| `na` / `ng` | **必须**产出实测值（退出码 0 的前提） |
| `nba` / `nbg` | 产出实测值，**或**明确记录为 `NOT IDENTIFIABLE` 并保留现值 |
| 原始输出 | 保留完整 stdout 与 `allan.png`，不得只写结论 |

#### 4.3 LiDAR→IMU 外参 `ext_il`

```bash
# 本仓库不附带标定工具，也不认证第三方工具。方法见：
#   docs/calibration_procedure.md §1
# 标定完成后，先做一次 30 s 快速自检（只能证伪，不能证实）
timeout -s INT 30 ros2 bag record -o static30 /mapping/lio_odom
python3 tools/odom_static_drift.py --bag static30 --topic /mapping/lio_odom \
    --max-drift-m 0.10
```

**验收门槛**

| 判据 | 期望 |
|:---|:---|
| `ext_il` 七元数 | 有实测来源（工具名/版本/日期/残差），写入 profile |
| 30 s 自检 | 漂移 ≤ 10 cm。**通过只代表"没有大错"，不代表外参准确** |
| 旋转部分 | 若只有偏航在漂而位置正常，是陀螺零偏或 `ext_il` 的**旋转**部分，不是平移误差 |

---

### 阶段 5：配置预检（上机前的强制门）

```bash
# 5.1 出厂 profile：应当失败（这是设计意图）
python3 tools/preflight_config.py --config src/sensing/fastlio2/config/lio_orin_nx.yaml
#     实测输出：2 blocking, 1 warning(s) -> NOT READY，退出码 2
#     两个阻塞项：ext_il 缺失、pcl2_time_field 为空
#     一个警告：na/ng/nba/nbg 仍是 MID360+VN200 的占位值

# 5.2 填完标定值后的正式 profile
python3 tools/preflight_config.py --config <你的.yaml>

# 5.3 严格模式：警告也视为未就绪（推荐用于正式验收）
python3 tools/preflight_config.py --config <你的.yaml> --strict

# 5.4 仅在"确实要在静止平台上不做扫描内补偿"时才使用
python3 tools/preflight_config.py --config <你的.yaml> --allow-no-time-field
```

**退出码**：`0` = 就绪；`2` = 有阻塞项；`1` = 文件读不了。

**该工具会拦截的阻塞项**（正是阶段 2–4 的漏做项）

| 阻塞项 | 含义 |
|:---|:---|
| `ext_il` 缺失 | 估计器将以 `r_il = I, t_il = 0` 运行——占位符，不是外参 |
| `pcl2_time_field` 为空 | 每点 `curvature = 0`，**没有**扫描内运动补偿 |
| `imu_acc_scale` 未声明 | 隐含使用代码默认值，g 制 IMU 会静默变成 10× 误差 |
| `na/ng/nba/nbg` 非正 | 噪声参数必须有意义 |
| 步态滤波开启但无频率 | 无法构成任何陷波节，补偿**不会**运行 |
| 陷波频率超过 Nyquist | 该节在运行期被丢弃，配置与实际行为不符 |
| **上游风格嵌套分组** | 本节点是**扁平读取器**，`common:` / `mapping:` / `preprocess:` / `publish:` 整块会被**静默忽略** |

**验收门槛**：正式 profile 必须 **`--strict` 下退出码 0**。退出码 2 而上机是
[调参与排查指南](tuning_guide.md) §7.3 列出的**一票否决项**。

---

### 阶段 6：静止漂移基线

```bash
# 机器狗上电、站立不动，>= 10 min（短录制通过也要重跑）
timeout -s INT 600 ros2 bag record -o static10 /mapping/lio_odom
python3 tools/odom_static_drift.py --bag static10 --topic /mapping/lio_odom
```

**默认预算**（可用命令行覆盖）

| 指标 | 默认预算 | 超预算的含义 |
|:---|:---|:---|
| 总漂移 `--max-drift-m` | 0.05 m | 外参或加速度量纲 |
| 漂移速率 `--max-drift-per-min` | 0.005 m/min | 同上 |
| 单步最大跳变 `--max-jump-m` | 0.10 m | **时钟域问题**（LiDAR/IMU 不同轴，或回放未加 `--clock`） |
| 偏航漂移 `--max-yaw-deg` | 1.0 ° | 陀螺零偏或 `ext_il` 的**旋转**部分 |

**验收门槛**：退出码 0，**且录制 ≥ 10 min**。工具在录制短于 10 min 时会打印
`NOTE: this recording is only <X.X> min... Re-run for >= 10 min.`——短录制通过**不等于**没问题。

> 这一步是对**外参 + 时间同步 + 加速度量纲**的整体检验，且不需要真值、不需要全站仪。
> 归因顺序见[调参与排查指南](tuning_guide.md) §6.2 与 §6.3。

---

### 阶段 7：室内建图与回环

场景、路线、控制点与统计口径见[测试场景与统计口径](test_scenarios.md) §3。

```bash
# 7.1 启动建图（lio_node + pgo_node）
#     注意：sensing.launch.py 目前把 config_path 硬编码为 config/lio.yaml，
#     且没有声明任何 launch 参数；要使用 Orin NX profile，必须显式传 config_path。
#     驱动：sensing.launch.py / universe.launch.py 启动的是 livox_ros_driver2（MID360），
#     对 Airy 无效 —— 必须改用厂商 RoboSense 驱动，或在 launch 中替换驱动节点。
ros2 run fastlio2 lio_node --ros-args \
    -p config_path:=$PWD/src/sensing/fastlio2/config/<你的.yaml> &
ros2 run pgo pgo_node --ros-args \
    -p config_path:=$PWD/src/optimization/pgo/config/pgo.yaml &

# 7.2 回放或实走（>= 3 圈）
ros2 bag play loop_3laps.bag --clock

# 7.3 留档：保存地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
    "{file_path: '/tmp/map_out', save_patches: true}"

# 7.4 只看回环决策日志
# grep -E '\[PGO\]\[(gate|sc)\]' <log>
```

**验收门槛**（口径必须与[测试场景与统计口径](test_scenarios.md) §1 一致）

| 指标 | 目标 | 不达标时的首要排查 |
|:---|:---|:---|
| 每圈闭合误差 | ≤ 5 cm | 逐点时间字段 → 外参 → PGO 门控 |
| 回环检测成功率 | ≥ 90% | Scan Context 阈值与门控，见[调参与排查指南](tuning_guide.md) §5 |
| 控制点匹配 RMSE | ≤ 5 cm | 外参、体素分辨率 |
| 体素分辨率一致性 | 若目标是 5 cm 级精度，`scan_resolution` 必须显著小于 5 cm | 出厂 profile 的 `0.3` **单此一项就已排除任何"亚分米级精度"声明** |

> **必须在验收文档中记录地图点数与内存占用**，否则建图速度的通过不可信
> （见[调参与排查指南](tuning_guide.md) §7.2）。

---

### 阶段 8：定位验证

```bash
# 8.1 启动定位（lio_node + localizer_node）。pcd_path 必须显式给出，
#     否则 localizer_node 会打印 FATAL 说明并退出（退出码 1）。
ros2 run fastlio2 lio_node --ros-args \
    -p config_path:=$PWD/src/sensing/fastlio2/config/<你的.yaml> &
ros2 run localizer localizer_node --ros-args \
    -p config_path:=$PWD/src/localization/localizer/config/localizer.yaml \
    -p pcd_path:=/tmp/map_out/map.pcd &

# 8.2 检查门控状态与重定位
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
ros2 service call /localizer/relocalize interface/srv/Relocalize \
    "{pcd_path: '/tmp/map_out/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 8.3 观察最终位姿输出
timeout 10 ros2 topic hz /robot_pose_map
```

**验收门槛**（四项**分别记录，不可合并成一个数**，见[测试场景与统计口径](test_scenarios.md) §1.2/§1.3）

| 指标 | 目标 | 时钟口径 |
|:---|:---|:---|
| 定位精度 ATE | ≤ 0.05 m | **发布戳**口径 |
| 定位精度 ATE | ≤ 0.05 m | **当前查询时钟**口径（与上一行差异极大，必须分开报） |
| 定位可用率（全段） | ≥ 95% | 含启动段 |
| 定位可用率（后锁） | ≥ 95% | 锁定之后 |

> 仓库在**合成场景**中实测全段可用率 88.58%、后锁 99.53%，全段不达标的主因是**启动时间**
> （约 6.62 s），不是运行中丢失。这是仿真口径的数字，**不能外推到本平台**；
> 板端必须重新测量并如实记录。参考[测试场景与统计口径](test_scenarios.md) §1.3。

---

### 阶段 9：四足步态适配

**前置条件（强制）**：阶段 2 的扫描内去畸变必须先确认生效。
**在 `pcl2_time_field` 仍为空时开启步态滤波是无效操作**，见[四足适配说明](quadruped_adaptation.md) §3。

```bash
# 9.1 直线行走、常速、同一地面，>= 30 s（建议 60 s）
timeout -s INT 60 ros2 bag record -o walk_imu /imu/data

# 9.2 提取基频与谐波（--emit-yaml 直接输出可粘贴的 gait_* 块）
python3 tools/imu_gait_spectrum.py --bag walk_imu --topic /imu/data \
    --emit-yaml --plot gait.png
```

工具会检查**谐波结构**（真实步态是周期运动，有 2/3/4 次谐波；孤立共振峰没有）。
**退出码 2 = 没有找到周期性步态**，此时**正确做法是保持 `gait_filter_enable: false`**，
而不是手工填一个频率。

**A/B 验证**（同一段包回放，A 关 / B 开）

```bash
# A：关闭 gait_filter_enable，回放同一段包，记录 map / odom
# B：开启同一组实测频率，重复
# 对比口径见 docs/tuning_guide.md §7.1（T1 建图精度 / T2 单帧时延）
```

**验收门槛**

| 判据 | 期望 |
|:---|:---|
| `gait_filter_sample_rate_hz` | **必须等于实测 IMU 频率**（阶段 1 的 `ros2 topic hz /imu/data`）。频率错了陷波无意义 |
| 启动日志 | 必须看到 `Gait filter ACTIVE: ...`。若看到 `Gait filter requested but NOT installed` 的 `RCLCPP_ERROR`，说明**滤波没有运行**，不要假设它生效 |
| A/B 结果 | **B 的 RMSE 不差于 A**。若 B 变差，说明陷波打到了真实运动：降低 `q` 或去掉谐波节 |
| 结论措辞 | 必须写明"**收益未在本仓库数据上测量**"。本仓库**没有任何数据**支持"补偿后精度提升"这类结论 |

> 陷波会**无条件**移除这些频率上的能量——不论步态是否真的在那里。
> 因此"开启了滤波"本身不是成果，A/B 对比才是。详见[四足适配说明](quadruped_adaptation.md) §4。

---

### 阶段 10：板端资源与温度（持续运行）

见 §5。**验收门槛**：连续 ≥ 30 min 满载运行下，资源与温度记录完整，
且 `lio_node` 无丢帧（`print_time_cost: true` 时的单帧耗时分布 p99 < 一个帧周期）。

---

## 5. Orin NX 资源与温度验证

### 5.1 现状声明

本仓库**没有** CPU 亲和性、实时优先级、内存上限、温度或端到端延迟的任何配置或测量记录
（全仓无 `chrt` / `taskset` / `sched_setscheduler` / 温度相关代码或脚本）。
**本节给出的所有目标值都是建议值，尚未在本平台上测量。**

### 5.2 观测方法

```bash
# 持续记录（1 Hz），跑到稳定状态之后再取数
tegrastats --interval 1000 --logfile tegrastats_$(date +%Y%m%d_%H%M%S).log
```

`lio_node` 的单帧耗时（需要在 profile 中把 `print_time_cost` 设为 `true`）：

```bash
# 节点会以 RCLCPP_WARN 打印 "Time cost: <ms> ms"，直接落进 launch 日志
ros2 run fastlio2 lio_node --ros-args -p config_path:=$PWD/src/sensing/fastlio2/config/<你的.yaml> \
    2>&1 | tee lio_run.log
grep 'Time cost:' lio_run.log
```

### 5.3 记录项与计算方式

> 下表左列的字段名以 `tegrastats` 在 Jetson 上的**实际输出为准**；右列的提取命令是**示例**，
> **未在 Orin NX 上执行过**（已在合成的 `tegrastats` 输出样本上验证过管道本身可用），
> 首次使用时请先 `cat` 一行原始日志核对字段格式。

| 记录项 | `tegrastats` 字段 | 示例提取命令 | 计算口径 |
|:---|:---|:---|:---|
| 内存占用 | `RAM <used>/<total>MB` | `grep -o 'RAM [0-9]*' tegrastats_*.log \| awk '{print $2}' \| sort -n \| tail -1` | 取峰值（MB），与总量比 |
| CPU 占用 | `CPU [<pct>%@<freq>,...]` | `grep -o 'CPU \[[^]]*\]' tegrastats_*.log \| sed 's/CPU \[//; s/\]//' \| awk -F, '{s=0; n=0; for(i=1;i<=NF;i++){gsub(/%.*/,"",$i); if($i!="off"){s+=$i; n++}} printf "%.1f\n", s/n}' \| sort -n \| tail -1` | 取**在线核**的百分比均值，再取峰值（Orin 空闲核会报 `off`，必须排除，否则均值被拉低） |
| 温度 | `tj@<C>C`、`Tboard@<C>C` | `grep -o 'tj@[0-9]*C' tegrastats_*.log \| awk -F'[@C]' '{print $2}' \| sort -n \| tail -1` | 取持续运行期的最高值，**不是**瞬时值 |
| 功耗 | `VDD_IN <mW>mW/<mW>mW` | `grep -o 'VDD_IN [0-9]*mW' tegrastats_*.log \| awk '{print $2}' \| sed 's/mW//' \| sort -n \| tail -1` | 取峰值（mW） |
| 单帧处理耗时 | `Time cost: <ms> ms` | `grep -o 'Time cost: [0-9.]*' lio_run.log \| awk '{print $3}' \| sort -n \| awk '{a[NR]=$1} END{printf "p50=%.2f p95=%.2f max=%.2f\n", a[int(NR*0.5)], a[int(NR*0.95)], a[NR]}'` | 直接输出 p50 / p95 / max（ms） |
| 输出帧率 | — | `timeout 30 ros2 topic hz /mapping/lio_odom` | 稳定值（Hz） |

### 5.4 建议目标值（**尚未在本平台测量**）

| 指标 | 建议目标 | 性质 |
|:---|:---|:---|
| CPU 占用 | < 60% | **建议值**，非实测 |
| 内存占用 | < 8 GB | **建议值**，非实测 |
| 温度 | 持续运行 < 85 °C | **建议值**，非实测；需在目标机箱与环境温度下测 |
| 帧率 | ≥ 10 Hz | 与 10 Hz 输入匹配 |
| 单帧耗时 | p99 < 100 ms | 一个 10 Hz 帧周期 |

> **这些数字不是本平台的实测结果，也不构成任何性能保证。**
> 必须由板端实测后替换。仓库既有的仿真数值（宿主 CPU：p50 26.1 ms / p95 48.6 ms / p99 62.3 ms，
> 实时因子 0.9957）**全部来自开发机，不是 Orin NX**，见[测试场景与统计口径](test_scenarios.md) §2.2。

### 5.5 调度绑定（候选手段，未验证）

```bash
# 候选写法：把调度策略与亲和性施加在 launch 父进程上
sudo chrt -f 50 taskset -c 2,3 ros2 launch <pkg> <file>.launch.py
```

- 子进程**一般会**通过 fork/exec 继承亲和性与调度策略，所以该写法通常**确实能影响到**节点；
  但**继承不等于生效**，也不构成"已生效"的证据。
- 要明确归属与可观测性，推荐用 launch 的 per-node `prefix`（如 `prefix='taskset -c 2,3 chrt -f 50'`）
  或 cgroup/systemd 单元，并**实测**核实：亲和性读 `/proc/<pid>/status` 的 `Cpus_allowed_list`；
  调度策略读 `chrt -p <pid>`（`/proc/<pid>/status` **没有** `policy` 字段）。
- **上述实测均未执行，属待板端验证。**

---

## 6. 常见启动故障与处置

| 症状 | 可能原因 | 处置 |
|:---|:---|:---|
| **`/rslidar_points` 不存在** | ① 厂商驱动未启动；② `sensing.launch.py` / `universe.launch.py` 启动的是 `livox_ros_driver2`（MID360），**对 Airy 无效**；③ 话题名与驱动实际发布名不一致；④ 驱动 `POINT_TYPE` 与预期不符 | ① `ros2 node list` 确认驱动节点在跑；② 改用厂商 RoboSense 驱动，或替换 launch 中的驱动节点；③ `ros2 topic list` 核对真实话题名并同步改 `lidar_topic`；④ 核对 SDK 构建选项 |
| **静止不动却在漂移** | 外参、加速度量纲、或时钟域 | 跑 `python3 tools/odom_static_drift.py --bag static10 --topic /mapping/lio_odom`，按三步归因：`largest step` 超预算 → 时钟域；平滑漂移 → `ext_il` 其次 `imu_acc_scale`；只有 yaw 漂 → 陀螺零偏或 `ext_il` 旋转部分 |
| **`IMU Message is out of order` / `Lidar Message is out of order`** | LiDAR 与 IMU 不在同一时间轴（错域），或消息时间戳本身回退 | **代码的行为是清空对应缓冲**（不是丢弃单帧）：`timestamp < last_*_time` 即 `swap()` 掉整个 deque，之后 `syncPackage()` 会因缓冲为空反复 `return false`，直到新样本到达。先修同步（§4 阶段 3），**不要**先调参数。见[时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) §0 |
| **`NO Effective Points!`** | 该帧通过质量门限的有效点 < 1，IESKF 该帧无观测 | 检查 `point_quality_thresh`（是**无量纲评分 s**，不是米）、`lidar_min_range` / `lidar_max_range`、场景是否过于空旷/退化。同时看 `FEATURE ... qualityOK=` 诊断行定位是哪一级过滤掉的。见[调参与排查指南](tuning_guide.md) §2.7 |
| **估计器以单位外参运行**（`ext_il` 缺失） | profile 里没有 `ext_il` 键 | 启动日志会出现 `Missing or invalid ext_il parameter, using defaults`；节点每 100 帧打印的 `STATE ...` 行会显示 `r_il_q=[0.00000 0.00000 0.00000 1.00000] t_il=[0.0000 0.0000 0.0000]`。**这是占位符，不是外参**——完成 §4 阶段 4.3 后填入 |
| **IMU 加速度大约 10× 偏大** | `imu_acc_scale` 与 IMU 实际单位不匹配（例如 IMU 已输出 m/s² 但用了 g 制的 `10.0`） | 静止录 30 s，比较均值 `\|a\|` 与 9.81：≈9.81 → `1.0`；≈1.0 → `10.0`。`tools/imu_allan_variance.py` 会在 `mean \|accel\|` 不在 `(8.0, 11.0)` 时拒绝出结果 |
| **点云分层 / 锯齿（尤其四足移动时）** | `pcl2_time_field` 为空 ⇒ **没有扫描内运动补偿**；或字段名/量纲填错；或 SDK 是默认 `XYZI` 构建 | ① 跑 `python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points`；② 退出码 2 且 SDK 为 `XYZI` → 用 `POINT_TYPE=XYZIRT` 重建驱动；③ 退出码 0 → 把输出的两行写进 profile；④ **不要**在 YAML 里编字段名（找不到即静默退化为不去畸变，且配置看起来已填） |
| **`gait_filter_enable: true` 但日志是 ERROR** | 开启了步态滤波但没有可用的陷波节 | 看到 `Gait filter requested but NOT installed ...` 说明**滤波没有运行**、IMU 通路未被修改。先用 `tools/imu_gait_spectrum.py` 实测频率；测不到周期性步态就保持 `false` |
| **`localizer_node` 启动即退出** | 未配置先验地图 `pcd_path` | 节点会打印 FATAL 说明并以退出码 1 结束（这是有意的改进，替代了过去的 YAML 转换异常）。用 `-p pcd_path:=/path/to/map.pcd` 或写进 `localizer.yaml` |
| **订阅了话题但没有数据** | 发布函数在 `get_subscription_count() <= 0` 时**直接 return** | 确认确实有订阅者（`ros2 topic hz` / `ros2 bag record` 都算）。"没数据"不等于算法故障 |
| **`map → odom` 有两个发布者** | `pgo_node`（建图）与 `localizer_node`（定位）被同时启动 | 仓库设计上二者**互斥**。用 `ros2 topic info /tf --verbose` 核对发布者数量 |
| **配置写了却"没生效"** | 用了上游 FAST-LIO 的嵌套分组（`common:` / `mapping:` / `preprocess:` / `publish:`） | 本节点是**扁平读取器**，嵌套分组被**静默忽略**。`tools/preflight_config.py` 会把上游分组名判为**阻塞项**。按 profile 头部的映射表拍平 |

---

## 7. 部署配置模板

### 7.1 扁平 schema 模板（**所有键都在顶层**）

> 下表中的数值**全部是占位符或出厂默认**，不是本平台的测量值。
> 每一项标 **待实测** 的都必须由括号内的工具在目标平台上产出后替换。
> 注意：**不要**引入 `common:` / `mapping:` / `preprocess:` / `publish:` 之类的嵌套分组——
> 本节点不读它们，整块会被静默忽略。
>
> **本模板刻意保持"未就绪"状态**（`ext_il` 被注释掉、`pcl2_time_field` 为空），
> 因此照抄后跑预检会**失败**，与出厂 profile 的行为一致。这是设计意图：
> 只有当每一项都换成实测值之后，预检才应该通过。

```yaml
# ---- 接口 ----
imu_topic: /imu/data                 # 待实测：确认 IMU 实际话题名
lidar_topic: /rslidar_points         # 待实测：确认驱动实际话题名
body_frame: body
world_frame: odom
print_time_cost: true                # 阶段 10 需要；正式跑可关

# ---- 前端 / 预处理 ----
lidar_type: pointcloud2              # Airy/Odin1 走通用 PointCloud2 分支（不是 livox）
lidar_max_line: 4                    # pointcloud2 模式下不使用（仅 livox CustomMsg 路径）
lidar_filter_num: 2
lidar_min_range: 0.5                 # 待实测：与雷达实际盲区/安装位置匹配
lidar_max_range: 30.0                # 待实测：是否截断有效远点

# 待实测：tools/rslidar_pcl2_probe.py --topic /rslidar_points
#   退出码 0 -> 填入该工具输出的两行（字段名 + 到秒的换算系数）
#   退出码 2 -> 保持 "" 并在验收文档中显式承认"不做扫描内去畸变"，
#               预检需加 --allow-no-time-field
# 注意：本模板刻意留空。填一个"猜的"字段名会让预检通过而补偿静默失效。
pcl2_time_field: ""
pcl2_time_scale: 1.0                 # 到【秒】的换算系数；纳秒填 1e-9（不是 1e9）

# 待实测：静止均值 |a| ≈ 9.81 时填 1.0；IMU 输出单位为 g 时填 10.0
# 预检只能检查该键"是否被显式声明"，无法判断声明的值是否正确 —— 必须实测。
imu_acc_scale: 1.0

# ---- 建图 / IESKF ----
scan_resolution: 0.3                 # 该值本身已排除亚分米级精度声明（见 tuning_guide）
map_resolution: 0.3
cube_len: 200.0
det_range: 30.0
move_thresh: 1.5

# 待实测：tools/imu_allan_variance.py --bag imu_static --topic /imu/data --emit-yaml
#   下面四个是出厂占位值（复制自 MID360 + VN200），不是本平台 IMU 的噪声参数。
#   刻意保留这四个值：预检会因此打印一条警告，提醒它们尚未被替换。
na: 0.1
ng: 0.01
nba: 0.0001
nbg: 0.0001

# ---- IMU 初始化 ----
imu_init_num: 40
imu_init_window_s: 3.0
imu_init_static_gyro_std: 0.005
imu_init_static_acc_dev: 0.3
imu_init_max_wait_s: 5.0

# ---- 估计器 ----
near_search_num: 5
ieskf_max_iter: 5
gravity_align: true
esti_il: false                       # 固定外参：标定误差直接进入建图误差
point_quality_thresh: 0.9            # 无量纲评分 s，不是距离（米）

# 待实测：LiDAR -> IMU 外参（标定方法见 docs/calibration_procedure.md §1）
#   YAML 顺序 [x y z qx qy qz qw]；内部 Eigen Quaterniond(w,x,y,z)
#   几何语义 p_imu = R_il * p_lidar + t_il
# 本模板刻意把该键注释掉：缺键时预检会判为阻塞项，估计器启动时会打印
# "Missing or invalid ext_il parameter, using defaults" 并以 r_il = I, t_il = 0 运行。
# 标定完成后取消注释并整体替换为实测值。
# ext_il: [<x>, <y>, <z>, <qx>, <qy>, <qz>, <qw>]

lidar_cov_inv: 1000.0

# ---- 四足步态补偿（默认关闭；未实测到周期性步态时保持 false）----
# 待实测：tools/imu_gait_spectrum.py --bag walk_imu --topic /imu/data --emit-yaml
gait_filter_enable: false
# gait_notch_freq_hz: [<f0>, <2*f0>, <3*f0>]   # 由上面的工具产出
# gait_notch_q: <f0 / 实测峰宽>                # 由上面的工具产出
gait_filter_sample_rate_hz: 200.0    # 待实测：必须等于 ros2 topic hz /imu/data 的实测值
gait_filter_gyro_x: true
gait_filter_gyro_y: true
gait_filter_gyro_z: false
gait_filter_accel_x: false
gait_filter_accel_y: false
gait_filter_accel_z: false
```

### 7.2 一个能通过预检的完整示例

仓库内提供了一份**完整且能通过 `tools/preflight_config.py`** 的示例：

```
tests/fixtures/calibrated_profile_example.yaml
```

```bash
python3 tools/preflight_config.py --config tests/fixtures/calibrated_profile_example.yaml
#   实测输出：0 blocking, 1 warning(s) -> READY with warnings，退出码 0
#   该警告是"步态滤波已启用，其收益在本仓库数据上未被测量"
```

> ⚠️ **该文件的数值是合成测试值，不是任何真实机器人的测量结果。**
> 它的唯一用途是证明预检工具会接受一份完整的配置、并逐项拒绝每一种缺陷
> （对应 `tests/test_probe_tools.py` 中的预检用例）。**不要把它的数值复制到部署配置里。**

### 7.3 落地顺序

```bash
# 1) 按 §7.1 填写正式 profile（扁平 schema）
# 2) 预检：先看阻塞项，再上严格模式
python3 tools/preflight_config.py --config <你的.yaml>
python3 tools/preflight_config.py --config <你的.yaml> --strict
# 3) 退出码为 0 之前不得上机（退出码 2 = 有阻塞项）
# 4) 加载到节点：config_path 必须传绝对路径
ros2 run fastlio2 lio_node --ros-args -p config_path:=$PWD/src/sensing/fastlio2/config/<你的.yaml>
```

---

## 8. 回滚与复现

### 8.1 必须随每次运行归档的记录项

一次"已验证"的运行必须能回答"当时跑的是哪份代码、哪份配置、哪段数据"。
归档要求与[测试场景与统计口径](test_scenarios.md) §6 一致：

| 项目 | 采集方式 |
|:---|:---|
| 代码版本 | `git rev-parse HEAD` |
| 工作树是否干净 | `git status --porcelain`（有未提交改动必须一并归档 diff） |
| 配置哈希 | `sha256sum <你的.yaml>` |
| 配置原文 | 归档 YAML 文件本身，不要只留哈希 |
| 二进制哈希 | `sha256sum install/fastlio2/lib/fastlio2/lio_node` |
| 原始数据包名与时长 | `ros2 bag info <bag>` 的输出 |
| 工具原始输出 | `tools/*` 的**完整 stdout**（含退出码），不要只留结论 |
| 标定记录表 | [标定流程](calibration_procedure.md) §6 的表格 |
| 统计口径标注 | 每个数字注明是发布戳还是当前时钟口径 |
| 未验证项 | 明确列出**没有**验证的部分 |

### 8.2 一键归档命令

```bash
RUN=<运行名>
mkdir -p runs/$RUN

git rev-parse HEAD            > runs/$RUN/git_head.txt
git status --porcelain        > runs/$RUN/git_status.txt
git diff                      > runs/$RUN/worktree.diff

sha256sum <你的.yaml>          > runs/$RUN/config.sha256
cp <你的.yaml>                 runs/$RUN/config.yaml

sha256sum install/fastlio2/lib/fastlio2/lio_node > runs/$RUN/lio_node.sha256

ros2 bag info <bag>           > runs/$RUN/bag_info.txt

python3 tools/preflight_config.py --config <你的.yaml> \
    > runs/$RUN/preflight.txt 2>&1; echo "exit=$?" >> runs/$RUN/preflight.txt

# 环境事实（版本、域、时区）——复现时最先要看的东西
{
  echo "date: $(date -Iseconds)"
  echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
  echo "ROS_DISTRO: $ROS_DISTRO"
  echo "RMW: $RMW_IMPLEMENTATION"
  uname -a
} > runs/$RUN/env.txt
```

### 8.3 回滚

| 场景 | 回滚动作 |
|:---|:---|
| 改参数后精度变差 | 用归档的 `config.yaml` 覆盖当前 profile，重跑 §4 阶段 5 预检与阶段 6 静止漂移 |
| 改了 `ext_il` | **必须**回到上一份**有标定记录**的 `ext_il`，不要用"手调一个值试试"的方式回滚 |
| 开启了步态滤波后变差 | `gait_filter_enable: false` 即回到逐位一致的历史行为（滤波器惰性、无样本被修改） |
| 改了 `pcl2_time_field` | 回到上一份**有探针原始输出与退出码**的取值；若都不确定，退回 `""` 并显式承认不做补偿 |
| 代码回滚 | `git checkout <归档的 HEAD>`；注意 `tools/` 与 `tests/` 当前是未跟踪目录，回滚前先确认它们仍在 |

### 8.4 复现一份"已验证"结论的最低要求

1. 代码版本 + 配置哈希 + 二进制哈希三者齐全；
2. 原始数据包可重放（回放必须加 `--clock`，否则时间戳与录制时不符）；
3. 工具输出是**原始 stdout**，不是转述；
4. 每个指标都注明口径（发布戳 / 当前时钟、全段 / 后锁、仿真 / 板端）；
5. **未在目标平台实测的配置不得声称已验证**——这是本仓库的策略，
   `src/sensing/fastlio2/config/lio_orin_nx.yaml` 因此故意保持不完整，
   并会被 `tools/preflight_config.py` 判为 `NOT READY`。

---

## 9. 相关文档

- [调参与排查指南](tuning_guide.md) — 精度不达标时的排查顺序、参数区间与一票否决项
- [标定流程](calibration_procedure.md) — 外参 / 逐点时间 / IMU 噪声 / 步态频率的测量方法
- [四足适配说明](quadruped_adaptation.md) — 步态补偿的实现、启用前置条件与 A/B 验证
- [测试场景与统计口径](test_scenarios.md) — 回环路线、控制点、统计量与证据归档要求
- [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) — 本仓库不含时间同步实现
- [FastLIVO2 接口契约](validation/04b73f5_fastlivo2_interface.md) — 话题/服务/TF/外参的源码级契约
- [工程部署方案与平台契约](validation/04b73f5_p3_deployment.md) — 依赖矩阵、平台契约与待板端验证清单
