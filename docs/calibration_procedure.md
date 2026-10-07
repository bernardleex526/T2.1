# 标定流程（Orin NX + Airy/Odin1 + 四足平台）

> 本文档描述**如何产出**外参、逐点时间字段、IMU 噪声与步态频率这四个量。
> **本仓库不含任何目标平台的实测值**，因此本文只给流程、判据与工具，不给"示例结果"。
> 每个步骤末尾都写明"记录什么"，因为验收时需要的是可追溯的原始证据，不是一句"已标定"。

---

## 0. 四个标定项与对应工具

| 标定项 | 影响的参数 | 工具 | 不做会怎样 |
|:---|:---|:---|:---|
| LiDAR→IMU 外参 | `ext_il` | 外部标定（见 §1） | 估计器以 `r_il=I, t_il=0` 运行，点云畸变、建图扭曲、里程计漂移 |
| 逐点时间字段 | `pcl2_time_field`、`pcl2_time_scale` | `tools/rslidar_pcl2_probe.py` | 扫描内**完全不去畸变**，移动平台点云分层/锯齿 |
| IMU 噪声 | `na`、`ng`、`nba`、`nbg` | `tools/imu_allan_variance.py` | 滤波器 IMU/LiDAR 权重失配（现值为另一套传感器的复制值） |
| 步态频率 | `gait_notch_freq_hz`、`gait_notch_q` | `tools/imu_gait_spectrum.py` | 四足步态振荡被当作真实旋转积分 |

**顺序很重要**：必须先做 §2（逐点时间）再做 §3（IMU 噪声）再做 §4（步态）。原因见 `docs/quadruped_adaptation.md` §3——若扫描内去畸变本身没工作，后面的标定无法判断效果。

---

## 1. LiDAR→IMU 外参 `ext_il`

### 1.1 语义与格式（先确认再标定）

本仓库的契约（源码事实）：

- YAML 顺序：`ext_il: [x, y, z, qx, qy, qz, qw]`
- 内部 Eigen 四元数构造为 `Quaterniond(w, x, y, z)`（`lio_node.cpp::loadParameters()`）
- 几何语义：`p_imu = R_il * p_lidar + t_il`
- 该外参**默认为固定值**（`esti_il: false`）。若开启 `esti_il: true`，外参会成为滤波状态并被在线估计——此时**标定误差会渗入地图**，除非你确实想在线估计，否则保持 `false`。

### 1.2 标定方法

仓库**不附带**标定工具，也不认证第三方工具。可行路径：

1. **LiDAR-IMU 联合标定**（推荐）：用支持 LiDAR-IMU 外参标定的工具链，采集包含**充分激励**的数据（平移 + 旋转，覆盖各轴）。
2. **CAD/机械测量**：若 LiDAR 与 IMU 的安装关系在机械图纸上明确，可据此得到初值，再用 1 的方法精化。

### 1.3 记录什么（验收证据）

| 项目 | 说明 |
|:---|:---|
| 原始数据包名与时长 | 例如 `calib_2026xxxx.bag`，含激励段 |
| 标定工具与版本 | 具体名称、版本、参数 |
| 平移残差 | 目标 < 3 mm（若工具输出该量） |
| 旋转残差 | 目标 < 0.5°（若工具输出该量） |
| 最终 `ext_il` 七元数 | 直接可粘贴的一行 |
| 静止漂移复核 | §5 的结果，用于交叉验证外参 |

### 1.4 快速自检（不需要标定工具）

静止放置 30 s，回放：

```bash
python3 tools/odom_static_drift.py --bag static30 --topic /fastlio2/lio_odom --max-drift-m 0.10
```

- 若 30 s 内漂移 > 10 cm → `ext_il` 或 `imu_acc_scale` 有错（工具会打印归因提示）。
- 注意：**这条自检只能证伪，不能证实**。它通过不代表外参准确，只代表没有大错。

---

## 2. 逐点时间字段 `pcl2_time_field` / `pcl2_time_scale`

### 2.1 为什么不能猜字段名

速腾官方 ROS 2 SDK `rslidar_sdk` 的字段布局由**编译期宏**决定：

| 构建选项 | 发布字段 | 是否有逐点时间 |
|:---|:---|:---|
| `POINT_TYPE=XYZI`（**SDK 默认**） | `x, y, z, intensity` | ❌ **没有** |
| `POINT_TYPE=XYZIRT` | `x, y, z, intensity, ring, timestamp` | ✅ `timestamp`，`FLOAT64` |

即：**最可能的真实答案是"没有该字段"**。此时必须改 SDK 构建选项，而不是在 YAML 里编一个字段名——写一个不存在的字段名，`pcl2_to_PCL` 会找不到偏移并静默退化为"不去畸变"，与留空等价，却看起来像已配置。

### 2.2 步骤

```bash
# 方式 A：直接订阅在线话题
python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --once

# 方式 B：对已录制的包
python3 tools/rslidar_pcl2_probe.py --bag walk.db3 --topic /rslidar_points --emit-yaml

# 方式 C：无 ROS 环境时，先在有 ROS 的机器上导出布局，再离线分析
python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --dump-fixture layout.json
python3 tools/rslidar_pcl2_probe.py --fixture layout.json
```

工具的判据（不是靠字段名猜，而是靠实测值域）：

- 逐点时间字段的值域跨度必须约等于**一个扫描周期**（Airy 10 Hz → 0.1 s）。跨度远大于一个周期说明是绝对时钟而非扫描内偏移。
- 单位由"哪个 10 的幂次能把跨度折算到约一个扫描周期"决定（s / ms / µs / ns）。
- 字段类型必须是 `FLOAT32` 或 `FLOAT64`。`UINT32` 纳秒是**真实存在但本仓库读不了**的字段：`Utils::pcl2_to_PCL` 只接受浮点，工具会以退出码 2 明确拒绝，而不是给出一个会让补偿静默失效的配置值。

### 2.3 记录什么

| 项目 | 说明 |
|:---|:---|
| 完整字段布局 | 工具打印的 `name / type / offset / bytes` 表 |
| 判定的字段名与量纲 | 工具输出 |
| 实测跨度与扫描周期之比 | 应接近 1.00 |
| SDK 构建选项 | `POINT_TYPE` 的实际值 |
| 复核证据 | 回放时 RViz 中点云"分层"是否消失 |

---

## 3. IMU 噪声 `na` / `ng` / `nba` / `nbg`

### 3.1 前置：先确认量纲

```bash
# 静止采集，检查静态均值 |a| 是否约 9.81
ros2 bag record /imu/data -o imu_static --duration 30
python3 tools/imu_allan_variance.py --bag imu_static --topic /imu/data
```

- 若静态 `|a|` ≈ 1.0 → IMU 以 g 为单位，`imu_acc_scale` 必须相应设置（代码默认 10.0 是为 MID360 内置 IMU 准备的）。**量纲错了，Allan 结果没有意义**，工具会直接拒绝（退出码 2）并提示这一点。
- 若静态 `|a|` ≈ 9.81 → `imu_acc_scale: 1.0`。

### 3.2 采集要求

| 项目 | 要求 | 原因 |
|:---|:---|:---|
| 时长 | **≥ 2 h**（建议 6–12 h） | 白噪声区需要短 τ，随机游走区需要长 τ |
| 状态 | 机器狗上电、**站立不动** | Allan 方差假设过程平稳；行走数据测到的是步态，不是传感器 |
| 环境 | 保持正常工作温度 | 零偏随温度变化，冷机与热机结果不同 |
| 采样率 | 记录实际值 | 采样率错了，噪声密度按 sqrt(rate) 整体偏移 |

### 3.3 判据与"不可辨识"的正确理解

工具用线性最小二乘拟合标准 Allan 噪声模型（`σ²(τ) = A/τ + B·τ + C`），并给出每个系数的**标准误**。只有当系数 > 2 倍标准误时才输出——否则报 `n/a`。

这不是工具失败，而是物理事实：随机游走的可观测性由**交叉点** `τ_c = √3·N/K` 决定。以本仓库现用值 `ng = 0.005`、`nbg = 3e-6` 为例，`τ_c ≈ 2887 s`，而 2 h 记录最长 τ 只有 1800 s——**该随机游走在 2 h 内根本看不见**。此时：

- `na`/`ng`（白噪声）总是可测，必须填入；
- `nba`/`nbg` 若报 `n/a`，**保留现值并标注"未标定"**，或延长记录时间。**不要**把拟合出的噪声当参数写入配置。

### 3.4 记录什么

| 项目 | 说明 |
|:---|:---|
| 原始包名、时长、实际采样率 | |
| Allan 曲线图 | `--plot allan.png` 的产物 |
| 每个轴的 N 与 K 及其标准误 | 工具表格输出 |
| 最终 `na/ng/nba/nbg` | 可粘贴的四行（`--emit-yaml`） |
| 未辨识项的处理 | 明确写"保留默认"或"延长记录后重测" |

---

## 4. 步态频率 `gait_notch_freq_hz` / `gait_notch_q`

### 4.1 采集

```bash
# 直线行走，常速（四足典型 0.5–1.0 m/s），≥ 30 s（建议 60 s）
ros2 bag record /imu/data -o walk_imu
```

要求：**直线**、**匀速**、**同一地面**。转向会引入额外的低频运动，混入频谱。

### 4.2 提取

```bash
python3 tools/imu_gait_spectrum.py --bag walk_imu --topic /imu/data --emit-yaml --plot gait.png
```

工具逻辑：

1. Welch PSD（2 s 段、Hann 窗、50% 重叠）；
2. 找出高于噪声底 4 倍的峰（`--prominence` 可调）；
3. **检验谐波结构**：周期性的步态会有 2/3/4 次谐波，孤立共振没有。只有至少 2 个谐波存在时才认定为步态基频；
4. 由实测峰宽反推 `q = f0 / 带宽`（而非预设）。

若工具以退出码 2 拒绝输出，说明**没有找到周期性步态**——此时正确的做法是**保持 `gait_filter_enable: false`**，而不是手工填一个频率。

### 4.3 记录什么

| 项目 | 说明 |
|:---|:---|
| 行走包名、速度、时长、地面 | 步态频率与速度相关 |
| PSD 图 | `--plot` 产物 |
| 基频与谐波列表、峰宽、`q` | 工具输出 |
| 该速度下的对应关系 | 若需覆盖多速度，分别测量 |

---

## 5. 集成验收：静止漂移

四项标定完成后，做一次完整的静止漂移测试，这是对**外参 + 时间同步 + 加速度量纲**的整体检验：

```bash
# 机器狗上电站立不动，≥ 10 min
ros2 bag record /fastlio2/lio_odom -o static10

python3 tools/odom_static_drift.py --bag static10 --topic /fastlio2/lio_odom
```

判据（默认预算，可在命令行覆盖）：

| 指标 | 默认预算 | 超预算的含义 |
|:---|:---|:---|
| 总漂移 | ≤ 5 cm | 外参或加速度量纲 |
| 漂移速率 | ≤ 0.5 cm/min | 同上 |
| 单步最大跳变 | ≤ 10 cm | **时钟域问题**（LiDAR 与 IMU 不同轴，或回放未加 `--clock`） |
| 偏航漂移 | ≤ 1.0° | 陀螺零偏或外参的**旋转**部分 |

工具会按"跳变 / 平滑漂移 / 纯偏航漂移"三种形态分别给出归因提示，按提示顺序排查。

---

## 6. 配置落地与复核

```bash
# 1. 把标定结果写入部署配置（注意：扁平 schema，不要嵌套分组）
# 2. 预检
python3 tools/preflight_config.py --config src/sensing/fastlio2/config/lio_orin_nx.yaml

# 3. 严格模式：把警告也视为未就绪（推荐用于正式验收）
python3 tools/preflight_config.py --config <your.yaml> --strict
```

`preflight_config.py` 会拦截的正是本文档 §1–§4 的漏做项：`ext_il` 缺失、`pcl2_time_field` 为空、`imu_acc_scale` 未声明、噪声参数非正、步态滤波开启但无频率、频率超 Nyquist、以及**上游风格的嵌套分组**（本节点是扁平读取器，嵌套分组会被静默忽略）。

### 标定记录表（建议随配置一起归档）

| 标定项 | 值 | 方法/工具 | 日期 | 证据文件 | 未辨识/未验证说明 |
|:---|:---|:---|:---|:---|:---|
| `ext_il` | | | | | |
| `pcl2_time_field` / `scale` | | | | | |
| `imu_acc_scale` | | | | | |
| `na` / `ng` | | | | | |
| `nba` / `nbg` | | | | | 常因记录长度不足而不可辨识 |
| `gait_notch_freq_hz` / `q` | | | | | |
| 时间同步方式与实测误差 | | | | | 见 TIME_SYNC_NOTES.md |

---

## 7. 相关文档

- [硬件部署指南](hardware_deployment.md) — 上机顺序
- [调参与排查指南](tuning_guide.md) — 标定后仍不达标时的排查
- [四足适配说明](quadruped_adaptation.md) — 步态补偿的代码与验证
- [测试场景与统计口径](test_scenarios.md) — 回环路线与统计量
- [时间同步说明](../src/sensing/fastlio2/config/TIME_SYNC_NOTES.md) — 本仓库不含同步实现
