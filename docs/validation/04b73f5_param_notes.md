# 算法参数定义、量纲、加载层级与实测值对照（版本 04b73f5）

- 基线：`fork` @ `04b73f553918b1dcc751feac69c0a2834f194432`（`main`）。本文为**静态源码/配置核验**，未运行算法、未构建、未回放。
- 取代关系：本文**显式取代**仓库外前版历史文档 `docs/validation/04b73f5_param_notes.md`（下称“前版文档”）。外层历史文档保持原样，其错误值在 §1.4/§7 逐条纠正。
- **本次修订（DeploymentContract）**：① `lio.yaml` 回归 **legacy 缺省**——该文件内**不含**任何 C2 开关键，三个键只出现在 opt-in 的 `lio_c2_experimental.yaml`（§1.5）；② 部署 profile `lio_orin_nx.yaml` **不写** `ext_il`（没有标定就不填，§1.5/§4）；③ Airy/Odin1 **不是 Livox 产品**（§4）；④ 全文 `file:line` 引用已按当前工作树重新核对。
- 列含义：
  - **代码默认** = 结构体/静态初始化值（`commons.h`、`icp_localizer.h`、`outlier_gate.h`、`*_node.cpp` 的 `NodeConfig`）。
  - **仓库值** = 仓库内 YAML 实际写入值（`src/**/config/*.yaml`）。
  - **有效加载层级** = 该参数**实际生效**的来源顺序（前者覆盖后者）。
  - **历史实际值** = 历史回放实际使用的值；来源为旧产物 `artifacts/04b73f5_diagnosis_20260929_JWUGeL/{replay_2_fork,replay_3_localizer}/config/*` 与 `resolved.json`。**没有 dump 的一律写“未记录”**，不估算、不填补。
- 证据哈希与命令分类见 `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/audit/engineering_evidence/source_manifest.md`。
- **范围边界（重要）**：本文只登记**参数定义/量纲/加载层级与历史配置值**，**不**做根因归因或精度评价；主指标坐标链与独立标定缺失问题归 `EvalContract`/证据审计负责，当前 **BLOCKED**。历史回放配置值**仅**证明当时所用配置，不构成对任何精度结论的支持。
- **路径约定**（本文内短名一律指下列实体，避免与同名文件混淆）：
  `sysroot=${fork}`；`commons.h`=`${sysroot}/src/sensing/fastlio2/src/map_builder/commons.h`（注意另有 `pgo/src/pgos/commons.h`、`hba/src/hba/commons.h`、`localizers/commons.h`）；`lio_node.cpp`/`utils.cpp`=`${sysroot}/src/sensing/fastlio2/src/`；`lidar_processor.cpp`/`imu_processor.cpp`/`imu_init.h`/`ieskf.*`/`map_builder.cpp`=`${sysroot}/src/sensing/fastlio2/src/map_builder/`；`lio.yaml`/`lio_highres.yaml`=`${sysroot}/src/sensing/fastlio2/config/`；`localizer.yaml`/`localizer_node.cpp`/`outlier_gate.h`/`icp_localizer.h`=`${sysroot}/src/localization/localizer/`(`config/`、`src/`、`src/localizers/`)；`pgo.yaml`/`pgo_node.cpp`=`${sysroot}/src/optimization/pgo/`(`config/`、`src/`)；`*.launch.py`=`${sysroot}/src/launch/<包名>/launch/`。

---

## 1. 前端 LIO 参数（`fastlio2`）

### 1.1 加载层级总览（先看这里，再看表）

`lio_node::loadParameters()`（`lio_node.cpp:184-309`）只读**一个** `config_path` YAML：
`--ros-args -p config_path:=<file>`；launch 传的是 `fastlio2/config/lio.yaml`（`sensing.launch.py:127-128`、`universe.launch.py:78-79`、`lio_launch.py:13-14`）。

层级（高→低）：

1. `launch`/CLI 的 `config_path` 指向的 YAML（本仓库 = `lio.yaml` 或 `lio_highres.yaml`）；
2. 该 YAML **缺键**时：`loadParameters` 的三元回退（`config[k] ? ... : <默认>`）或**直接保持结构体默认**；
3. 结构体默认（`commons.h:31-89`）/ 静态成员默认（`ieskf.cpp:4-8`）。

**陷阱**：`max_bias_accel` 有**三个**不同值——`commons.h:88` = `0.2`、`ieskf.cpp:7` 的静态初始值 = `0.5`、`loadParameters` 缺键回退 = `0.5`（`lio_node.cpp:297`）。而 `lio.yaml`/`lio_highres.yaml` **都未写**这三个键，因此实机生效值是 `max_bias_gyro=0.1 / max_bias_accel=0.5 / max_velocity=10.0`，`commons.h` 里的 `0.2` **永不生效**。

**陷阱**：`lio_highres.yaml` 头注释过去声称“changes ONLY the two voxel-downsampling parameters”，**不成立**。逐一加载两文件比对，相对 `lio.yaml` 它还改了 `lidar_filter_num`(2→3)、`det_range`(100→40)、`imu_init_num`(100→20)、`na/ng/nba/nbg`(0.06/0.005/8e-5/3e-6 → 0.1/0.1/1e-4/1e-4)，并**删除了** `imu_init_window_s`/`imu_init_static_gyro_std`/`imu_init_static_acc_dev`/`imu_init_max_wait_s` 四个键 ⇒ 静态窗 IMU 初始化被**静默关闭**（回落到 legacy 计数模式，见 `imu_init.h:103-108`）。该注释已按实际内容改写。

### 1.2 单位 / 时间 / 外参类参数（先看这组）

| 参数名 | 代码位置 | 有效加载层级 | 意义 | 单位 | 代码默认 | 仓库值 (`lio.yaml` / `lio_highres.yaml`) | 历史实际值 |
|---|---|---|---|---|---|---|---|
| `world_frame` | `commons.h`（无；`lio_node.cpp:37` NodeConfig）| YAML 覆盖 NodeConfig 默认 | `lio_odom`/TF/路径的父坐标系名 | 字符串 | `"lidar"`（`lio_node.cpp:37`）| `odom` / `odom` | `odom`（`resolved.json.world_frame`） |
| `body_frame` | `lio_node.cpp:36` | 同上 | TF/odom 子坐标系名 | 字符串 | `"body"` | `body` / `body` | `body` |
| `imu_topic` | `lio_node.cpp:33,198` | YAML > 默认 | IMU 输入话题 | 字符串 | `/livox/imu` | `/livox/imu` / `/livox/imu` | replay_2 `/vn200/imu`；replay_3 `/mid360/livox/imu` |
| `lidar_topic` | `lio_node.cpp:34,199` | 同上 | LiDAR 输入话题 | 字符串 | `/livox/lidar` | `/livox/lidar` | replay_2 `/livox/lidar`；replay_3 `/mid360/livox/lidar` |
| `image_topic` | `lio_node.cpp:35,200` | 同上 | 图像输入话题（仅着色用） | 字符串 | `/camera2/camera/color/image_raw` | **两 YAML 均未设** ⇒ 生效为默认 | 未记录 |
| `print_time_cost` | `lio_node.cpp:38,203` | YAML > 默认 | 打印单次 `process()` 耗时 | 布尔 | `false` | `false` | `true`（两次回放都打开） |
| `lidar_type` | `commons.h:37`、`lio_node.cpp:246` | YAML > 默认 | `livox`=订阅 CustomMsg；其它=订阅 PointCloud2 | 字符串 | `"livox"` | `livox` / `livox` | replay_2 `livox`；replay_3 **`pointcloud2`** |
| `lidar_max_line` | `commons.h:38`、`lio_node.cpp:247` | YAML > 默认 | CustomMsg 有效线数上限（`line < max_line` 才保留） | 整数（线） | `4` | `4` / `4` | `4`（pointcloud2 模式下该值被忽略，`utils.cpp` 不读 line） |
| `imu_acc_scale` | `commons.h:39`、`lio_node.cpp:248` | YAML > 默认（**乘法**作用于 `imuCB`） | 加速度原始值乘性缩放，用于 g→m/s² | 无量纲（增益） | `10.0` | `10.0` / `10.0` | replay_2 **`1.0`**（VN200 已输出 m/s²）；replay_3 `10.0` |
| `pcl2_time_field` | `lio_node.cpp:40,208` | YAML > 默认 | PointCloud2 逐点时间字段名；空=不做去畸变 | 字符串 | `""` | `""` / `""` | replay_3 **`timestamp`**；replay_2 未用 |
| `pcl2_time_scale` | `lio_node.cpp:41,209` | 同上 | 字段值×该系数=**秒**偏移 | 无量纲（到秒） | `1.0` | `1.0` / `1.0` | replay_3 **`1.0e-09`**（纳秒时间戳） |
| `pcl2_filter_phase`（C2.3，见 §1.5） | `lio_node.cpp:45,212`；`utils.cpp:30,67-76,95` | YAML > 默认；**仅** PointCloud2 路径 | 抽稀相位：保留原始索引 `i` 满足 `(i-phase) % lidar_filter_num == 0` 的点（0=上游 `i % point_filter_num == 0`） | 整数（0…filter_num-1，越界取模折返） | `0` | `0` / `0` | 未记录（历史回放未使用，键不存在） |
| `ext_il` | `lio_node.cpp:256-267` | YAML（缺键→`WARN`+单位阵/零平移） | `p_imu = R_il·p_lidar + t_il` | `[m,m,m,–,–,–,–]` | 恒等/零 | `[-0.011,-0.02329,0.04412, 0,0,0,1]` | replay_2 **`[0.02,0,0.037, 1,0,0,0]`**（绕 X 180°，MCD）；replay_3 = 仓库值 |
| `ext_lc` | `lio_node.cpp:269-277` | 同上 | 相机相对 LiDAR；内部转 `r_cl=R_lcᵀ`,`t_cl=-R_lcᵀt_lc` | 同上 | 恒等/零 | 见 `lio.yaml:50-59` | 未记录（两次回放的 `ext_lc` 与仓库一致，但无运行期回读 dump） |
| `cam_width`/`cam_height` | `commons.h:76-77`、`lio_node.cpp:279-286` | YAML > 默认 | 着色投影标定分辨率；图像先 `resize` 到此尺寸 | 像素 | `1280`/`720` | `1280`/`720` | 未记录 |
| `cam_fx`/`cam_fy`/`cam_cx`/`cam_cy` | `commons.h:78-81` | YAML > 默认 | 针孔内参（Brown–Conrady 模型，`pinhole_camera.cpp:9`） | 像素 | `641.9763…`/`641.0658…`/`641.5721…`/`368.2183…` | 同默认 | 未记录 |
| `cam_d` | `commons.h:82`、`lio_node.cpp:289-291` | YAML > 默认 | 5 畸变系数 `k1,k2,p1,p2,k3`；`|d0|>1e-7` 才启用去畸变 | 无量纲 | 见 `commons.h:82` | 同默认 | 未记录 |
| `lidar_cov_inv` | `commons.h:84`、`lio_node.cpp:293` | YAML > 默认 | 点到面残差信息矩阵标量权重（`H += Jᵀ·w·J`） | 无量纲权重（源码未注单位） | `1000.0` | `1000.0` / `1000.0` | 未记录 |

### 1.3 噪声 / 采样 / 前端门限类参数

`na/ng/nba/nbg` 的**实现事实**（仅此而已）：`imu_processor.cpp:13-17` 写进 `m_Q` 的 (0,0)/(3,3)/(6,6)/(9,9)，`ieskf.cpp:91-98` 以 `P += G·Q·Gᵀ` 注入，`G` 的 (0,0)/(12,3) 块含 1 个 `dt`、(15,6)/(18,9) 块含 1 个 `dt`（`ieskf.cpp:92-95`）。
**单位/量纲：源码与 YAML 均未标注，本任务不做分类。** 评审 R10 指出：`Q` 直入 `GQGᵀ` 且 `G` 含 `dt` 只证明存在 `dt` 的幂次缩放，**不能据此推断它们是“连续时间噪声密度”**（离散方差经同样乘法也会产生 `dt²` 项）。因此本表**撤回**先前的“连续密度/离散方差”分类，只登记“量纲未标注、**未判定**”；要给出量纲需另行推导 `P` 的物理含义并实测标定，属**未验证**。上表 `na/ng/nba/nbg` 的“单位”列只写“**未标注（未判定）**”。

| 参数名 | 代码位置 | 有效加载层级 | 意义 | 单位 | 代码默认 | 仓库值 | 历史实际值 |
|---|---|---|---|---|---|---|---|
| `na` | `commons.h:45`、`imu_processor.cpp:14` | YAML > 默认 | 加速度计白噪声（`Q(3,3)`） | **未标注（未判定）**（见本节说明） | `0.01` | `0.06` / `0.1` | replay_2/replay_3 均为 `0.06` |
| `ng` | `commons.h:46`、`imu_processor.cpp:13` | 同上 | 陀螺白噪声（`Q(0,0)`） | 同上 | `0.01` | `0.005` / `0.1` | 两者均 `0.005` |
| `nba` | `commons.h:47`、`imu_processor.cpp:16` | 同上 | 加速度计零偏随机游走（`Q(9,9)`） | 同上 | `0.0001` | `0.00008` / `0.0001` | 两者均 `8.0e-05` |
| `nbg` | `commons.h:48`、`imu_processor.cpp:15` | 同上 | 陀螺零偏随机游走（`Q(6,6)`） | 同上 | `0.0001` | `0.000003` / `0.0001` | 两者均 `3.0e-06` |
| `lidar_filter_num` | `commons.h:31`、`lio_node.cpp:208` | YAML > 默认 | 原始点**抽稀**步长（每 N 取 1） | 整数（点） | `3` | `2` / `3` | 两者均 `2` |
| `lidar_min_range` | `commons.h:32` | YAML > 默认 | 最近有效距离（`|p|² < min²` 丢弃） | m | `0.5` | `0.5` / `0.5` | `0.5` |
| `lidar_max_range` | `commons.h:33` | YAML > 默认 | 最远有效距离（`|p|² > max²` 丢弃） | m | `20.0` | **`30.0`** / **`30.0`** | 两者均 `30.0` |
| `scan_resolution` | `commons.h:34`、`lidar_processor.cpp:16-19,179-186` | YAML > 默认；`>0` 才建 VoxelGrid | 单帧 VoxelGrid 叶尺寸；`0` 则直通（`copyPointCloud`） | m | `0.15` | `0.15` / **`0.05`** | replay_2 **`0.5`**；replay_3 `0.15` |
| `map_resolution` | `commons.h:35`、`lidar_processor.cpp:7,135-137` | YAML > 默认 | ikd-Tree 盒滤波体素 + 地图点体素质心化步长 | m | `0.3` | `0.3` / **`0.05`** | replay_2 **`0.5`**；replay_3 `0.3` |
| `cube_len` | `commons.h:41` | YAML > 默认 | 局部地图立方体边长 | m | `300` | `1000` / `1000` | `1000` |
| `det_range` | `commons.h:42` | YAML > 默认 | 局部地图滑动检测半径 | m | `60` | `100` / **`40`** | `100` |
| `move_thresh` | `commons.h:43` | YAML > 默认 | 触发滑动的边界倍率（`det_thresh = move_thresh·det_range`） | 无量纲（倍率） | `1.5` | `1.5` / `1.5` | `1.5` |
| `near_search_num` | `commons.h:66` | YAML > 默认 | 每次 kNN 近邻数（平面拟合与选点判据共用） | 整数（点） | `5` | `5` / `5` | `5` |
| `ieskf_max_iter` | `commons.h:67` | YAML > 默认 | IESKF 迭代上限 | 整数（次） | `5` | `5` / `5` | `5` |
| `imu_init_num` | `commons.h:49` | YAML > 默认 | legacy 模式最少缓存样本数 | 整数（样本） | `20` | `100` / `20` | `100` |
| `imu_init_window_s` | `commons.h:53`、`imu_init.h:101` | YAML > 默认；`>0` 启用静态窗 | 静止窗时长；`>0` 才启用静窗判定 | s | `0.0`（=关闭） | `3.0` / **未设 ⇒ 0.0** | `3.0` |
| `imu_init_static_gyro_std` | `commons.h:54`、`imu_init.h:139` | YAML > 默认 | 静窗判据：三轴陀螺标准差上限 | rad/s | `0.005` | `0.005` / 未设⇒`0.005` | `0.005` |
| `imu_init_static_acc_dev` | `commons.h:55`、`imu_init.h:139`、`imu_processor.cpp:100` | YAML > 默认 | 静窗判据 & 实测重力采纳判据：相对窗均值的最大偏差 | m/s² | `0.3` | `0.3` / 未设⇒`0.3` | `0.3` |
| `imu_init_max_wait_s` | `commons.h:56`、`imu_init.h:142-143` | YAML > 默认 | 最长等待；到点后**仅按时间**回退到“最安静窗” | s | `15.0` | `5.0` / 未设⇒`15.0` | `5.0` |
| `gravity_align` | `commons.h:68`、`imu_processor.cpp:99-103` | YAML > 默认 | `true` 时用实测 `|a|`（仅当 `8.5<|a|<10.5` 且窗内 acc_dev 达标）覆盖 `State::gravity`，并将 `r_wi` 对齐重力 | 布尔 | `true` | `true` / `true` | `true` |
| `esti_il` | `commons.h:69` | YAML > 默认 | 是否在线估计 `r_il/t_il`（源码 `initialize()` 无条件用配置值写 `x().r_il/t_il`，未按该开关分支） | 布尔 | `false` | `false` / `false` | `false` |
| `point_quality_thresh` | `commons.h:70`、`lidar_processor.cpp:212,256-257` | YAML > 默认 | 点平面匹配质量评分门限，判据 `s > thresh`，`s = 1 - 0.9·|pd2|/sqrt(|p_body|)` | **无量纲**（不是米！） | `0.1` | `0.9` / `0.9` | `0.9` |
| `max_bias_gyro` | `commons.h:87`、`ieskf.cpp:4`、`lio_node.cpp:296` | YAML > 缺键回退 > 结构体默认 | `bg` 逐轴硬截断 | rad/s | `0.1`（三处一致） | 两 YAML **均未设** ⇒ `0.1` | `0.1`（未设） |
| `max_bias_accel` | `commons.h:88`(**0.2**)、`ieskf.cpp:7`(**0.5**)、`lio_node.cpp:297`(**0.5**) | 同上 | `ba` 逐轴硬截断 | m/s² | **`0.2`（不生效）** | 两 YAML 未设 ⇒ **`0.5`** | `0.5` |
| `max_velocity` | `commons.h:89`、`ieskf.cpp:8`、`lio_node.cpp:298` | 同上 | 速度硬截断 | m/s | `10.0` | 未设 ⇒ `10.0` | `10.0` |

**与 `point_quality_thresh` 极易混淆的量**：`esti_plane(points_near, 0.1, pabcd)` 的 `0.1` 是**平面拟合距离阈值 [m]**（`lidar_processor.cpp:250`），与 `s` 门限**不是同一个量**。源码里 `min_plane_quality` 只用于 `s > min_plane_quality`（`lidar_processor.cpp:212,257`），即门限就是 `s` 本身。

### 1.4 对前版文档的参数纠正

| 参数 | 前版文档（外层） | 实际（`lio.yaml`） | 结论 |
|---|---|---|---|
| `lidar_max_range` 仓库/生产值 | `300.0` | `30.0`（`lio.yaml:9`） | **前版错 10×**；且不能凭前版猜测，本文只报 `20.0`（代码默认）/`30.0`（仓库值）/`30.0`（历史实际） |
| `scan_resolution` 生产值 | `0.5` | `0.15`（`lio.yaml:10`） | **前版错**（`0.5` 是 replay_2 的 MCD 配置值） |
| `map_resolution` 生产值 | `0.5` | `0.3`（`lio.yaml:11`） | **前版错**（同上） |
| `na/ng/nba/nbg` 单位 | `(m/s²)²` 等离散方差 | 源码与 YAML 均未标注；**不能**由 `G·Q·Gᵀ` 推断为连续密度 | **撤回分类**：只登记“量纲未标注、未判定”（评审 R10） |
| `imu_acc_scale` 语义 | “需乘以 10 转 m/s²” | 代码是**乘性缩放**，MID360 取 10.0、VN200 取 1.0 | 保留（表述改为“乘性增益”） |
| `lidar_max_line` | “Livox 扫描线筛选上限” | `line < lidar_max_line`（`utils.cpp:10`）⇒ MID360 4 线需 `>=4` | 保留并补充严格不等号 |

### 1.5 C2 实验开关（**未验证的假设，非已证修复**）

`imu_acc_normalize` / `imu_init_mode` / `imu_init_min_samples` 是 C2 前端漂移归因的**消融开关**：实现见 `commons.h:57-66`（默认 `false` / `"static_window"` / `40`）、`lio_node.cpp:231,232-237,238`（扁平键读取，无别名/无嵌套）、`imu_init.h`（窗口选择）、`imu_processor.cpp`（归一化与初始化）。

| 参数名 | 代码位置 | 意义 | 单位 | 代码默认 | 仓库值 |
|---|---|---|---|---|---|
| `imu_acc_normalize` | `commons.h:57-60`、`lio_node.cpp:231`、`imu_processor.cpp:73-85,130,192,201` | true 时按上游 FAST-LIO2 式把每个加速度样本乘 `9.81/|窗内 acc 均值|`（在第 73-85 行只测一次，随后作用于传播与去畸变的所有 acc 样本） | 布尔 | `false` | **`lio.yaml` 未设** ⇒ legacy；`lio_c2_experimental.yaml: true` |
| `imu_init_mode` | `commons.h:61-65`、`lio_node.cpp:232-237`、`imu_processor.cpp:33-41,46-48` | `"first_batch"`=首帧整批直接初始化（不等静窗、不超时回退）；`"static_window"`=legacy | 字符串 | `"static_window"` | **`lio.yaml` 未设** ⇒ legacy；实验 profile `first_batch` |
| `imu_init_min_samples` | `commons.h:66`、`lio_node.cpp:238`、`imu_init.h:88-100` | first_batch 就绪所需样本数 | 整数（样本） | `40` | **`lio.yaml` 未设** ⇒ 40；实验 profile `40` |

- **必须保守表述**：C2 的消融回放**一次都还没跑**（`artifacts/04b73f5_controlled_ab_20260929/c2_frontend_attribution.json` 只是静态因子表 + 实验计划），因此这些开关**不得**被称为“已修复”“已验证改善”，也不得作为部署缺省。唯一写入它们的文件是 opt-in 的 `lio_c2_experimental.yaml`（内容 = `lio.yaml` + 四键置开）；单因子臂（仅 C2.1、仅 C2.2 或仅 C2.3）从该文件删掉其它键即可。
- **C2.3（点云抽样相位）现在有开关**：`pcl2_filter_phase`（默认 `0`，见上表与 `simulation/ablations/c2_3_sampling_phase.yaml`；C2.3 前版“没有开关实现”的说法已作废）。范围与证据等级：
  - **只作用于 `Utils::pcl2_to_PCL`（通用 PointCloud2 路径）**。`livox2PCL`（CustomMsg）**没有**该参数：上游 CustomMsg 分支按**有效点计数**抽稀（`valid_num % point_filter_num`），本仓 `livox2PCL` 按**原始索引** `i += filter_num` 抽稀且其后才做 tag/line/距离过滤，因此在那里加“相位”是另一个（多因子）改动，**未**纳入本因子。
  - **厂商 PointCloud2 上相位可观测（实测，非仿真）**：`datasets/bags/IndoorOffice1` 的 `/mid360/livox/lidar`（19968 点，`line == i % 4` 对 100% 点成立、逐点时间步长 ~4.8 us、单帧内 136 处时间回退）在 `lidar_filter_num=2` 下相位 0 保留 line {0,2}、相位 1 保留 {1,3}，且 `t0`（=首个**保留**点的时间）平移 4.768 us；`filter_num=3`（TIERS 回放用值）四线都保留但配比与 `t0` 同样变化。HILTI/`/hesai/pandar`（`ring` 非索引主序、时间单调）则只表现为 `t0` 平移 0.95–3.10 us、各环配比变化 <2%。
  - **合成 bag 上该因子是空对照**：生成器按发射时间排序、每个方位角 48 线共享同一时间（索引 0..47 时间同为 0），故相位 0/1 共享 `t0` 与方位网格（`simulation/diagnostics/c2_3_sampling_phase.py` 实测 `t0` 平移 0.000000 s）。**因此不得**用合成结果主张“厂商等价”或“该因子无影响”。
  - **仍未验证的部分**：相位对漂移/精度是否有实质影响**一次都没跑过**，不得称“已修复/已验证改善”。
  - **行号锚点（C2.3 落地造成的偏移，供定位用）**：本次改动使当前工作树相对本文其它引用的“上次核对编号”发生偏移——`lio_node.cpp` 第 41 行之后 +4；其抽取解析行（本文记为 205 的 `pcl2_time_scale` 行）之后再 +3，故该行之后共 +7；`utils.cpp` 第 67 行起 +10；`utils.h` 第 31 行起 +3。上表 `pcl2_time_field`/`pcl2_time_scale`/`pcl2_filter_phase` 三行已按**当前工作树**对齐（208/209/212）；本文其余 `file:line` 仍为其上次核对时的编号（其它 agent 的后续改动同样会使其漂移），定位时请以**符号名**为准。
- `"first_batch"` 路径下 `imu_init_num`/`imu_init_window_s`/`imu_init_max_wait_s` **不参与**窗口判定（只用 `imu_init_min_samples`）。

---

## 2. 后端定位/门控参数（`localizer`）

### 2.1 加载层级

`localizer_node::loadParameters()`（`localizer_node.cpp:92-150`）：

- **强制项**（无 `if` 保护，缺键直接 YAML 转换异常）：`cloud_topic`、`odom_topic`、`map_frame`、`local_frame`、`update_hz`、`rough_scan_resolution`、`rough_map_resolution`、`rough_max_iteration`、`rough_score_thresh`、`refine_scan_resolution`、`refine_map_resolution`、`refine_max_iteration`、`refine_score_thresh`（`localizer_node.cpp:109-132`）。
- **可选项**（缺键→结构体默认）：`rough_max_corr_dist`、`refine_max_corr_dist`、`rough_min_inlier_ratio`、`refine_min_inlier_ratio`（`:137-140`）与全部 `gate_*`（`:141-145`）。
- `pcd_path`：先取 `-p pcd_path:=` 参数，再取 YAML（`localizer_node.cpp:93,99,120-121`）；都为空会**FATAL 退出**（`localizer.yaml` 注释与 `:157-166` 附近）。

### 2.2 表（含数据关联半径 → 评分 → 门控）

| 参数名 | 代码位置 | 有效加载层级 | 意义 | 单位 | 代码默认 | 仓库值 | 历史实际值 |
|---|---|---|---|---|---|---|---|
| `cloud_topic` | `localizer_node.cpp:30,109` | YAML（**强制**） | 输入机体系点云 | 字符串 | `/mapping/body_cloud` | `/mapping/body_cloud` | `/mapping/body_cloud` |
| `odom_topic` | `localizer_node.cpp:31,110` | YAML（强制） | 输入里程计（与点云 approx-time 同步） | 字符串 | `/mapping/lio_odom` | `/mapping/lio_odom` | 同 |
| `map_frame` / `local_frame` | `localizer_node.cpp:32-33,111-112` | YAML（强制） | `map→local` TF 名 | 字符串 | `map` / `lidar` | `map` / `odom` | `map` / `odom`（且 `local_frame` 会被首帧 odom 的 `frame_id` 覆盖，`localizer_node.cpp:339-341`） |
| `update_hz` | `localizer_node.cpp:34,113` | YAML（强制） | ICP 触发频率 | Hz | `1.0` | `2.0` | `2.0` |
| `rough_scan_resolution` / `rough_map_resolution` | `icp_localizer.h:16-17` | YAML（强制） | 粗配准 VoxelGrid 叶尺寸（扫描/地图） | m | `0.25`/`0.25` | `0.25`/`0.25` | `0.25`/`0.25` |
| `rough_max_iteration` / `refine_max_iteration` | `icp_localizer.h:19,14` | YAML（强制） | 两阶段 ICP 迭代上限 | 次 | `5`/`10` | `8`/`15` | `8`/`15` |
| `rough_score_thresh` / `refine_score_thresh` | `icp_localizer.h:18,13` | YAML（强制） | 内点**平均平方**最近邻距离门限 | **m²** | `0.2`/`0.1` | `0.08`/`0.02` | `0.08`/`0.02` |
| `rough_max_corr_dist` / `refine_max_corr_dist` | `icp_localizer.h:24-25` | YAML > 默认 | 关联半径（评分只在该半径内统计） | m | `0.35`/`0.15` | `0.35`/`0.15` | **未在 YAML 提供 ⇒ 用默认 `0.35`/`0.15`** |
| `rough_min_inlier_ratio` / `refine_min_inlier_ratio` | `icp_localizer.h:26-27` | YAML > 默认 | 必须有地图邻点的扫描点占比下限 | 无量纲 (0–1) | `0.2`/`0.2` | `0.2`/`0.2` | 未提供 ⇒ `0.2`/`0.2` |
| `gate_max_translation_m` | `outlier_gate.h:50`、`localizer_node.cpp:141` | YAML > 默认 | 相邻 ICP 更新间**机体系位姿**平移新息上限 | m | `0.04` | `0.04` | 未提供 ⇒ `0.04` |
| `gate_max_angle_rad` | `outlier_gate.h:51` | 同上 | 机体系位姿旋转新息上限 | rad | `0.03` | `0.03` | 未提供 ⇒ `0.03` |
| `gate_max_consecutive_rejects` | `outlier_gate.h:52` | 同上 | 连续拒绝次数 → 判 `lost`、重播种参考但**继续门控** | 次 | `10` | `10` | 未提供 ⇒ `10` |
| `gate_recovery_accepts` | `outlier_gate.h:58` | 同上 | 连续接受次数 → 报 `recovered`/valid | 次 | `10` | `10` | 未提供 ⇒ `10` |
| `gate_ema_alpha` | `outlier_gate.h:59` | 同上 | **仅**发布 `map←odom` 的 EMA 平滑系数 | 无量纲 | `0.10` | `0.1` | 未提供 ⇒ `0.1` |

**门控语义要点**（`outlier_gate.h:1-39`）：新息在**机体系位姿**上比较（不是 `map←odom` 的平移）；参考值是**上次被接受的原始解**（不是 EMA）；被拒候选**永不发布**；`lost` 期间发布值冻结在最后可信解。

**与评分门限的一致性检查（静态）**：`rough_score_thresh=0.08 < 0.35²=0.1225`、`refine_score_thresh=0.02 < 0.15²=0.0225`，均在物理上限内；若把阈值写到上限之上则门限**恒真**、失去意义。前版文档该条**正确，保留**。

---

## 3. 后端回环/图优化参数（`pgo`）

`pgo.yaml`（`src/optimization/pgo/config/pgo.yaml`）为全量配置（286 行），历史两次回放都**替换为 51 行的精简版**（值一致，仅删除注释）。表中“历史实际值”以 `replay_3_localizer/config/pgo.yaml` 为准（与 `replay_2_fork` 相同）。

| 参数名 | 意义 | 单位 | 仓库值 = 历史实际值 | 备注 |
|---|---|---|---|---|
| `cloud_topic` / `odom_topic` | 后端输入 | 字符串 | `/mapping/body_cloud` / `/mapping/lio_odom` | 与 localizer 一致 |
| `map_frame` / `local_frame` | `map→odom` TF 名 | 字符串 | `map` / `odom` | `pgo_node.cpp:526-527`；与 localizer **互斥** |
| `key_pose_delta_deg` / `key_pose_delta_trans` | 关键帧判据 | deg / m | `10` / `0.5` | — |
| `loop_enable_scan_context` / `loop_enable_radius_search` | 两类候选来源 | 布尔 | `true` / `true` | — |
| `loop_search_radius` | 半径检索上限 | m | `8.0` | 注释给实测漂移 `5.80 m / 134.15 m`（历史观测，**非本轮测量**） |
| `loop_time_tresh` | 时间间隔保护 | s | `60.0` | — |
| `min_loop_detect_duration` | 两次检测事件限速 | s | `2.0` | — |
| `max_loop_candidates_per_query` / `max_accepted_loops_per_query` | 每次检测的代价/产出上限 | 次 | `3` / `1` | — |
| `loop_source_submap_half_range` / `loop_submap_half_range` | 源/目标子图关键帧半径 | 关键帧 | `2` / `5` | — |
| `submap_resolution` | 子图 VoxelGrid | m | `0.1` | — |
| `sc_num_ring` / `sc_num_sector` | Scan Context 环/扇区 | 个 | `20` / `60` | 扇区量化 ⇒ 偏航 6° |
| `sc_max_radius` / `sc_lidar_height` / `sc_downsample_resolution` | 描述子参数 | m | `80.0` / `2.0` / `0.2` | — |
| `sc_dist_thresh` | 描述子余弦距离门限 | 无量纲 | `0.30` | — |
| `sc_num_candidates` / `sc_exclude_recent` | 候选数 / 排除最近 N 关键帧 | 个 | `3` / `30` | — |
| `coarse_voxel_resolution` / `coarse_max_corr_dist` / `coarse_max_iterations` / `coarse_wide_corr_dist` | 粗配准（fast_gicp） | m / m / 次 / m | `0.5` / `3.0` / `64` / `5.0` | — |
| `coarse_max_rmse` | 粗配准粗差界（**全点** RMS） | m | `1.5` | 注释说明 legacy `loop_score_tresh=0.15` 对应 `0.387 m` RMS |
| `fine_voxel_resolution` / `fine_max_corr_dist` / `fine_max_iterations` | 精配准（点到面 ICP） | m / m / 次 | `0.10` / `0.50` / `60` | — |
| `fine_max_rmse` | 精配准**全点** RMS（粗差界，不判别） | m | `1.5` | — |
| `fine_max_plane_rmse` | **判别性**残差：`overlap_radius` 内点到面 RMS | m | `0.05` | — |
| `fine_normal_search_radius` / `correspondence_randomness` | 法向估计半径 / GICP knn | m / 个 | `0.30` / `10` | — |
| `overlap_radius` / `min_overlap_ratio` | 重叠率半径 / 下限 | m / 无量纲 | `0.10` / `0.40` | — |
| `overlap_radius_2` / `overlap_radius_3` | **仅诊断**，不门控 | m | `0.20` / `0.50` | 源码注释明示 |
| `degeneracy_gate_enabled` / `degeneracy_min_eig_ratio` | 退化门控：`λ_min/λ_max(H)` 下限 | 布尔 / 无量纲 | `true` / `0.003` | 秩亏阈值，非精度阈值 |
| `max_loop_correction` / `correction_drift_ratio` | 修正量可行界 = `max(0.5, ratio·path)` | m / 无量纲 | `0.5` / `0.01` | — |
| `max_revisit_rel_t` | 重访一致性（仅 scan_context 候选） | m | `4.0` | — |
| `min_odo_correction` | 任一接受回环的最小修正量 | m | `0.15` | 共位种子豁免 |
| `cross_seed_max` | 两种子一致性 | m | `1.0` | — |
| `max_loop_z_offset` | z 合理性 | m | `0.5` | — |
| `max_yaw_disagreement` | **实测**相对偏航 vs 里程计相对偏航 | deg | `15.0` | 注释含“ASSUMPTION: 里程计偏航漂移低于该值” |
| `loop_noise_xyz` / `loop_noise_rpy` | 回环因子标准差 | m / rad | `0.01` / `0.005` | — |
| `loop_robust_k` | Huber 阈值（白化单位）；`0` 关闭 | 无量纲 | `3.0` | — |

> `pgo.yaml` 中大量“Basis: measured on this sequence”的注释数值属**历史观测记录**，本轮**未复测**，仅作为历史事实引用；不得据此宣称本版本已验证。

---

## 4. 数据集 / 传感器配置必须分开登记

**不得把某一数据集的调参值当作另一数据集或新硬件的“推荐值”。** 下表中的“仓库值”仅指该序列历史回放实际使用的配置副本（路径见行内）。

| 序列 / 传感器 | 关键差异 | 历史回放实际配置 | 依据 |
|---|---|---|---|
| **本仓库默认**（Livox MID360 + 内置 IMU） | CustomMsg、`imu_acc_scale=10.0`、`ext_il` 见 `lio.yaml:49` | —（仓库值即 §1.2/§1.3 的 `lio.yaml` 列） | `README.md:6-8`、`lio.yaml` |
| **TIERS IndoorOffice1/2**（MID360，`PointCloud2`） | `lidar_type=pointcloud2`、`pcl2_time_field=timestamp`、`pcl2_time_scale=1e-09`、`imu_rate=200 Hz`、话题带 `/mid360/` 前缀 | `replay_3_localizer/config/lio.yaml`；`resolved.json` 记录 `scan_resolution_m=0.15`、`map_resolution_m=0.3`、`lidar_msg_type=sensor_msgs/msg/PointCloud2` | `artifacts/04b73f5_diagnosis_20260929_JWUGeL/replay_3_localizer/{config/lio.yaml,resolved.json}` |
| **MCD `tuhh_night_09`**（MID360 + **VN200** IMU） | `imu_topic=/vn200/imu`、**`imu_acc_scale=1.0`**、`ext_il=[0.02,0,0.037,1,0,0,0]`（绕 X 180°）、`scan_resolution=0.5`、`map_resolution=0.5` | `replay_2_fork/config/lio.yaml`；原始 bag/GT 哈希见 `freeze/inputs.json` | 旧产物 + `README.md:226` 系列 |
| **Airy**（LiDAR，非 Livox） | 仓库内**无**驱动、无雷达参数、无实测数值；`lio_orin_nx.yaml` 是**未经板端验证**的部署起点（话题名按 RoboSense 惯例取 `/rslidar_points`，逐点时间字段留空） | `config/lio_orin_nx.yaml`（值来自 P3 规格映射，**非实测**；`ext_il` 有意不写） | 全仓 grep 无 `Airy` 相关驱动/标定 |
| **Odin1**（LiDAR+IMU 一体，非 Livox） | 仓库内**无**配置、无实测数值；其时间字段名/datatype/量纲/IMU 单位**未知** | **未记录** | 全仓 grep 无 `Odin1` |

**关于 Airy/Odin1 的硬性约束**：

- **Airy/Odin1 不是 Livox 产品**，因此 `lidar_type: livox`（`CustomMsg`）、Livox 驱动及其同步选项**不能假定适用**；`lio_orin_nx.yaml` 按通用 `PointCloud2` 路径（`lidar_type: pointcloud2`）配置，且这是**基于话题名的推断**，须用厂商手册确认。
- 迁移前必须先确认（否则不得写入任何“推荐值”）：① 点云消息类型（`CustomMsg` 还是 `PointCloud2`）；② 若为 `PointCloud2`，是否存在**逐点相对时间**字段及其 datatype 与量纲（决定 `pcl2_time_field`/`pcl2_time_scale`；不满足则静默失去扫描内去畸变，`utils.cpp:100-102`）；③ IMU 加速度单位是 g 还是 m/s²（决定 `imu_acc_scale`，`lio_orin_nx.yaml` 里的 `1.0` 是**假设值**，必须实测）；④ IMU 频率与时间戳钟域。
- 上述四项**当前全部为“未验证”**。本文**不提供** Airy/Odin1 的参数推荐范围——任何“建议值”在无实测前都是猜测，禁止写入验收或生产口径。
- **外参**：`lio_orin_nx.yaml` **不写** `ext_il`。此前该文件曾写 `[0.020, 0.000, 0.037, 1,0,0,0]`，那是从 MCD 回放配置（另一套 LiDAR/IMU）抄来的值，**不是** Airy 的标定结果，已删除。`ext_il` 是实测量（`p_imu = R_il·p_lidar + t_il`，YAML 顺序 `[x y z qx qy qz qw]`）；缺键时 `loadParameters()` 会 `WARN` 并退回 `r_il=I, t_il=0`，那只是**显式占位**，不是可用外参。同理，任何 `tf_static_*` 静态外参发布器都不得写入未标定数值。
- 同理，`lio_highres.yaml` 头注释里的“5 cm 精度目标/内存代价”缩放推断，源码注释自身已标 `[INFERENCE]`，**不是**测量结论。

---

## 5. 可复现启动命令（**本轮未执行**，来自仓库/历史脚本）

建图（`sensing_launch`，`lio_node` 命名空间 `mapping`）：

```bash
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash
export ROS_DOMAIN_ID=<id>; export ROS_LOCALHOST_ONLY=1
ros2 launch sensing_launch sensing.launch.py          # 配置固定为 fastlio2/config/lio.yaml
```

改配置文件的**无 launch**等价方式（profile 头部给出的形态）：

```bash
# 部署 profile（未验证起点；ext_il 有意留空 ⇒ 启动时会 WARN 并退回恒等占位）
ros2 run fastlio2 lio_node --ros-args -r __ns:=/mapping \
  -p config_path:=<install>/share/fastlio2/config/lio_orin_nx.yaml
# 高分辨率 profile
ros2 run fastlio2 lio_node --ros-args -r __ns:=/mapping \
  -p config_path:=<install>/share/fastlio2/config/lio_highres.yaml
# C2 消融 profile（实验用；单因子臂请复制后删掉另一个键）
ros2 run fastlio2 lio_node --ros-args -r __ns:=/mapping \
  -p config_path:=<install>/share/fastlio2/config/lio_c2_experimental.yaml
```

> 仿真 harness 用 `config_file:=`（基础 profile）与 `extra_config:=`（3 键覆盖层）选择同一批文件；`<repo>/simulation/launch/sim_fastlio2.launch.py` 的 `SIM_OVERRIDES` 会在其之上强制仿真的雷达类型/话题/`imu_acc_scale`/`ext_il`。

定位（`universe_launch`：`lio_node` @ `mapping`、`localizer_node` @ `localizer`）：

```bash
ros2 launch universe_launch universe.launch.py map:=<map_name>            # data_dir 可用 FLYOS_DATA_DIR 覆盖
ros2 launch universe_launch universe.launch.py data_dir:=<dir> map:=<name> \
  body_base_x:=0 body_base_y:=0 body_base_z:=0 body_base_roll:=0 body_base_pitch:=0 body_base_yaw:=0
```

- 历史回放**并未**用上述 launch，而是直接 `ros2 run` 两个可执行文件（`replay_3_localizer/logs/run.log`：`install/fastlio2/lib/fastlio2/lio_node`、`install/localizer/lib/localizer/localizer_node`），并传入**改写过的配置副本**。因此“启动命令 + 配置路径”必须与结果一并记录，否则不可复现。
- `pgo`/`localizer` 的 `config_path` 都由 launch 传入各自包内的 `config/*.yaml`（`sensing.launch.py:142-153`、`universe.launch.py:90-106`）。

---

## 6. 单位与量纲陷阱清单（易错点汇总）

1. `point_quality_thresh` 是**无量纲评分 s**，不是米；同函数里的 `0.1` 才是米（§1.3）。
2. `rough/refine_score_thresh` 是**m²**（平均平方距离），物理上限 `corr_dist²`；`min_inlier_ratio` 是无量纲占比。
3. `na/ng/nba/nbg` 源码与 YAML **均未标注量纲**，且 `P += G·Q·Gᵀ` 只证明 `dt` 幂次缩放 → **不判定为连续密度，也不照抄离散方差写法**；量纲属**未判定**（评审 R10）。
4. `pcl2_time_scale` 的目标是**秒**（把字段值乘到秒）；纳秒时间戳要填 `1e-9`。
5. `curvature` 通道语义是**毫秒偏移**，不是曲率；帧尾时间依赖它（`lio_node.cpp:389`）。
6. `imu_acc_scale` 是**乘性**缩放，不是“单位换算表”；VN200 已输出 m/s² ⇒ 必须设 `1.0`（历史 replay_2 即如此）。
7. `lidar_max_line` 判据是严格 `line < max_line`，故 MID360（4 线）必须 `>=4`。
8. `move_thresh` 是**倍率**，与 `det_range` 相乘才是米。
9. `gate_*` 是**每次 ICP 更新**的新息上限（不随时长缩放），且量测在机体系上。

---

## 7. 待验证（无运行证据，本轮预算 0）

- 上述所有“历史实际值”**只证明历史回放当时用的配置**，不证明当前仓库 HEAD 的同一配置会产生相同行为（本轮未运行）。
- `lidar_max_range=30.0` 是否与实际扫描范围匹配、是否截断了有效远点：**未验证**。
- `imu_init_window_s=3.0`（`lio.yaml`）与四足振动工况下的实际初始化耗时/成功性：**未验证**（`lio_highres.yaml` 更会退化为 legacy 模式）。
- `point_quality_thresh=0.9` 在稀疏场景下的有效点数比例：**未验证**（源码注释提到历史上出现过 `NO Effective Points!`，但那是历史观测）。
- `gravity_align` 的实测 `|a|` 采纳在非 MID360/VN200 硬件上的取值：**未验证**。
- `localizer`/`pgo` 的 `-p pcd_path:=` 与 YAML 的优先级在 `universe_launch` 下的实际生效项：`universe.launch.py:103-106` 传 `pcd_path` 覆盖 YAML，**需运行确认**。
