# 四足机器狗适配说明（T2.1 部署）

> 适用平台：Orin NX + 速腾聚创 Airy/Odin1 + 四足机器狗。
> **本文档给出的是"如何测量"与"代码里到底改了什么"，不是"已经测好了"。**
> 仓库没有该机器狗的实走数据，因此所有步态频率、噪声参数、外参均为**待测量项**。

---

## 1. 四足平台为什么会破坏 FAST-LIO2 的假设

FastLIO2 的 IESKF 预积分建立在两个前提上：

1. IMU 测到的是**刚体运动**，白噪声 + 慢变零偏；
2. LiDAR 与 IMU 处于**同一时间轴**，且扫描内运动可以由 IMU 插值补偿。

四足机器狗同时破坏这两条：

| 现象 | 物理来源 | 对算法的影响 |
|:---|:---|:---|
| 俯仰/侧滚周期振荡 | 步态（对角小跑等），基频与谐波落在约 10–40 Hz | IESKF 无此模型，把振荡当真实旋转积分 → 点云扭曲、"分层/锯齿" |
| 触地冲击 | 足端着地瞬间的加速度尖峰 | 加速度计出现远高于噪声的高频尖峰，破坏 `na` 的高斯假设 |
| 机体高频抖动 | 腿部驱动、关节间隙 | 抬高有效噪声底，用静态 Allan 方差标定的 `na/ng` 偏乐观 |
| 姿态快速变化 | 四足在 0.5 m/s 下俯仰仍可 ±5–10° | 扫描内去畸变误差被放大 |

**关键结论**：在点云"分层"出现时，**第一优先级不是调 IMU 权重，而是确认扫描内去运动补偿是否真的在工作**（见第 3 节）。`lio_orin_nx.yaml` 出厂时 `pcl2_time_field: ""`，即**完全没有扫描内补偿**，此时任何四足适配都是无效的。

---

## 2. 两种补偿方案与仓库的实际选择

评审报告给出两个方案。本仓库实现的是**方案 B（频域陷波）**，因为方案 A 依赖 T1.4 足式状态估计，而该依赖在本仓库不存在。

### 方案 A：步态相位感知滤波（未实现）

需要订阅 `/leg_odometry` 或 `/gait_phase`，按触地相位动态调整 `Q`：

```cpp
// 评审报告中的伪代码，本仓库【没有】实现
if (phase.is_contact) { Q_gyro *= 5.0; Q_accel *= 10.0; }
```

**未实现的原因**：本仓库没有任何足式里程计话题的发布者，`/gait_phase` 也不存在。写入订阅代码会得到一个永远收不到消息的节点——比不实现更糟，因为它看起来像已支持。

### 方案 B：陷波滤波（已实现）

文件：`src/sensing/fastlio2/src/map_builder/gait_filter.h`

级联 RBJ 二阶陷波器，在 IMU 样本进入 IESKF **之前**去掉步态周期分量。

#### 与评审报告伪代码的差异（均为有意修正）

| 报告伪代码 | 本实现 | 原因 |
|:---|:---|:---|
| `std::deque<Eigen::Vector3d> x_hist_, y_hist_` 无界历史 | 每个二阶节只保留 2 个状态（DF-II transposed） | 双二阶滤波器不需要无界历史；无界 deque 会随运行时间增长 |
| 先滤 z 再用原值覆盖 z | 按轴掩码，未选中的轴**完全不进滤波器状态** | 报告写法让偏航轴白跑一遍滤波并推进状态，属状态泄漏 |
| 忽略群延迟 | 提供 `groupDelaySeconds()` 并在启动时打印 | IIR 陷波会延迟被滤信号；陀螺与加速度计若延迟不同，二者相对时序被破坏，而预积分正依赖该对齐 |
| 直接信任配置 | `validate()` 拒绝 Nyquist/0/负频率与非正 Q | 超 Nyquist 的中心频率会静默失效或混叠 |

#### 代码位置

| 文件 | 改动 |
|:---|:---|
| `src/map_builder/gait_filter.h` | **新增**。纯头文件、不依赖 ROS/PCL，可独立测试 |
| `src/map_builder/imu_processor.{h,cpp}` | 新增 `configureGaitFilter()` 与 `ingestImu()`；**所有** IMU 入缓存路径统一走 `ingestImu()` |
| `src/map_builder/commons.h` | 新增 `gait_*` 配置项，默认全关 |
| `src/lio_node.cpp` | 读取扁平 `gait_*` 键；开启但无频率时打印 `RCLCPP_ERROR` |
| `src/CMakeLists.txt`(包内) | 注册 `test_gait_filter` |
| `config/lio_orin_nx.yaml` | 新增 `gait_*` 键，`gait_filter_enable: false` |

#### 为什么 `ingestImu()` 是唯一入口

`imu_processor.cpp` 里 IMU 缓存原本有**两处**增长点：`initialize()` 与 `undistort()`。若只在其中一处滤波，同一个样本会被滤两次或一次，取决于它落在哪条路径上——这种 bug 在仿真里几乎看不出来，在实机上表现为"有时好有时坏"。因此两处都改为调用 `ingestImu()`，滤波只发生一次。

#### 默认值为什么是"只滤陀螺 x/y"

- **偏航（gyro z）不滤**：步态不产生偏航振荡，而偏航是 LiDAR 唯一能直接观测的姿态分量。滤它只会引入无谓的相位滞后。
- **加速度计默认不滤**：陀螺与加速度计用**同一组系数**时，二者群延迟相同、相对时序不变。只滤陀螺会引入相对时间偏移。需要压制触地冲击时才打开加速度计，且**两个三元组一起打开**，使偏移互相抵消。`configureGaitFilter()` 在检测到"只滤一侧"时会打印该偏移量。
- **`gait_filter_enable: false`**：默认关闭时滤波器完全惰性，IMU 通路与历史行为**逐位一致**。

---

## 3. 启用前的强制前置条件

> **在完成第 3.1 步之前，不要开启步态滤波。** 若扫描内运动补偿本身没工作，陷波只会让你更难定位问题。

### 3.1 先确认扫描内去畸变已生效

```bash
# 1. 识别 Airy/Odin1 的逐点时间字段（不要猜字段名）
python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --once
#    或对已录的包：
python3 tools/rslidar_pcl2_probe.py --bag walk.db3 --topic /rslidar_points --emit-yaml

# 2. 把输出的两行填进 lio_orin_nx.yaml
#    pcl2_time_field: "timestamp"
#    pcl2_time_scale: 1e-09     # 按实测单位
```

**注意**：速腾官方 ROS2 SDK `rslidar_sdk` 的**默认** `POINT_TYPE=XYZI`，即只发布 `x/y/z/intensity`，**根本没有逐点时间字段**。只有构建为 `POINT_TYPE=XYZIRT` 时才会有 `timestamp`（FLOAT64，单位秒）。所以最可能的真实答案是"没有该字段"，此时必须改 SDK 构建选项，而不是在 YAML 里编一个字段名。

`tools/rslidar_pcl2_probe.py` 对这两种情况会给出不同的退出码与不同建议。

### 3.2 再测量步态频率

```bash
# 直线行走，常速，>= 30 s（建议 60 s）
ros2 bag record /imu/data -o walk_imu

# 提取基频与谐波
python3 tools/imu_gait_spectrum.py --bag walk_imu --topic /imu/data --emit-yaml --plot gait.png
```

该工具会：
- 用 Welch PSD 找出高于噪声底 4 倍的峰；
- **检查谐波结构**：真实步态是周期运动，会有 2/3/4 次谐波；孤立的共振峰没有。没有谐波结构时工具**拒绝**输出配置（退出码 2），因为对不存在的频率做陷波会删掉真实运动；
- 由实测峰宽反推 `gait_notch_q`，而不是预设 10。

---

## 4. 启用与验证

```yaml
gait_filter_enable: true
gait_notch_freq_hz: [18.50, 37.00]   # 来自 3.2 的实测输出
gait_notch_q: 12.0
gait_filter_sample_rate_hz: 200.0    # 必须等于真实 IMU 频率，用 ros2 topic hz 核对
gait_filter_gyro_x: true
gait_filter_gyro_y: true
gait_filter_gyro_z: false
```

启动时节点会打印一行 `Gait filter ACTIVE: ...` 或 `RCLCPP_ERROR`（配置无效）。**看到 ERROR 就说明滤波没有运行**，不要假设它生效了。

### 验证口径（A/B 对比，同一段包回放）

```bash
# A：关闭
ros2 launch sensing_launch sensing.launch.py config:=lio_orin_nx.yaml
ros2 bag play walk.bag --clock
# 记录 map / odom

# B：开启同一组频率，重复
```

对比指标（用仓库已有的仿真工具口径）：
- `simulation/scripts/test_t1_accuracy.py` — 建图 RMSE
- `simulation/scripts/test_t2_speed.py` — p95 延迟（陷波几乎不增加计算量，但需确认无回归）

**验收要求**：B 的 RMSE **不差于** A。若 B 变差，说明陷波打到了真实运动 —— 降低 `q` 或去掉谐波节。

> ⚠️ 评审报告声称"补偿后精度提升 >30%"。**本仓库没有任何数据支持该数字**，它来自对合成数据的推断。请以自测结果为准，不要把 30% 写进验收文档。

---

## 5. 四足平台的其它已知约束

| 项目 | 现状 | 说明 |
|:---|:---|:---|
| 扫描内去畸变 | ❌ 未配置 | 见 3.1；**这是精度问题的首要嫌疑** |
| 步态陷波 | ⚠️ 已实现、默认关 | 见第 4 节 |
| 足式里程计融合 | ❌ 未实现 | 无话题发布者；紧耦合需要改 `ieskf.h` 状态向量 |
| 触地冲击建模 | ❌ 未实现 | 仅能靠加速度计陷波近似压制 |
| 动态物体 | ❌ 未实现 | 点云中的人/其它机器人会被当作静态地图 |
| 长走廊/退化场景 | ❌ 未验证 | 需实测；见 `docs/tuning_guide.md` 退化场景一节 |

---

## 6. 相关文档

- [硬件部署指南](hardware_deployment.md) — 上机顺序与验收门槛
- [标定流程](calibration_procedure.md) — 外参 / 时间戳 / IMU 噪声 / 步态频率
- [调参与排查指南](tuning_guide.md) — 精度不达标时的排查顺序与参数区间
- [测试场景与统计口径](test_scenarios.md) — 回环路线、控制点、统计量定义
- [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) — 本仓库不含时间同步实现
