# FastLIVO2 当前 I/O、时间/坐标契约与视觉模块状态（版本 04b73f5）

- 基线：`fork` @ `04b73f553918b1dcc751feac69c0a2834f194432`（branch `main`；启动前 `git status` 仅 `?? artifacts/`，受跟踪文件干净）。
- 本文性质：**静态源码核验**（逐行读到 `file:line`）。本文件落盘时**未启动任何 ROS 节点、未构建、未回放**（本轮完整回放预算 0 次，见工程证据 `source_manifest.md` 的命令分类）。
- 取代关系：本文**显式取代**仓库外前版历史文档 `docs/validation/04b73f5_fastlivo2_interface.md`（下称“前版文档”）。外层历史文档**保持原样不被改写**，其错误结论在 §0 逐条列出纠正理由；前版未被本文点名的陈述**不自动视为已核实**。
- 证据哈希与逐文件清单见 `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/audit/engineering_evidence/source_manifest.md`。
- **本次修订（DeploymentContract）**：① 新增 §5.3「现有已实现接口 vs 未来适配层」的升级提案，并显式列出**不存在**的 API（视觉/光度融合权重等）；② §5.1/§5.2 补部署 profile `lio_orin_nx.yaml` **不写 `ext_il`**、仿真单位阵恒等为**仿真专用**、C2 开关缺省关闭且为未验证假设；③ 上游链接更正为 `hku-mars/FAST-LIVO2`（与 `FAST-LIO` 不是同一代码库）；④ 全文 `file:line` 已按当前工作树重新核对。
- **范围边界（重要）**：本文只登记**接口/契约源码事实**，**不**给出任何建图精度、根因归因或评测结论。主指标（测绘参考地图点到面 RMSE ≤ 0.05 m）的坐标链一致性与独立标定缺失问题归 `EvalContract`/证据审计负责，当前状态 **BLOCKED**；本文不引用、也不外推任何历史误差数值或因果论断。
- **路径约定**：本文短名 `lio_node.cpp`/`utils.cpp` = `fork/src/sensing/fastlio2/src/`；`commons.h`/`lidar_processor.cpp`/`image_processor.*`/`imu_processor.cpp`/`imu_init.h`/`ieskf.*`/`map_builder.cpp` = `fork/src/sensing/fastlio2/src/map_builder/`；`pinhole_camera.*` 同前；`lio.yaml`/`lio_highres.yaml` = `fork/src/sensing/fastlio2/config/`；`localizer.yaml`/`localizer_node.cpp`/`outlier_gate.h` = `fork/src/localization/localizer/`(`config/`、`src/`、`src/localizers/`)；`pgo.yaml`/`pgo_node.cpp` = `fork/src/optimization/pgo/`(`config/`、`src/`)；`hba_node.cpp` = `fork/src/optimization/hba/src/`；`*.srv` = `fork/src/common/interface/srv/`；`*.launch.py` = `fork/src/launch/<包名>/launch/`。

---

## 0. 对前版文档的纠正/取代清单（不重写历史事实）

| # | 前版文档位置 | 前版陈述 | 源码实际（file:line） | 处置 |
|---|---|---|---|---|
| C1 | 前版 §二.1 表 | 着色点云话题 `/mapping/colored_cloud` | 实际发布者名 `color_world_cloud`（`lio_node.cpp:171`），在 `namespace=mapping` 启动下解析为 `/mapping/color_world_cloud` | **纠正**；同时补全前版遗漏的 `body_cloud`/`world_cloud`/`lio_path` |
| C2 | 前版 §二.2 表 | 服务 `/mapping/save_map`，类型 `std_srvs/srv/Empty` | 该服务在仓库内**不存在**（全仓 grep 无 `save_map` 服务定义）；保存底图的服务是 `/pgo/save_maps`，类型 `interface/srv/SaveMaps`（`pgo_node.cpp:164-165`） | **纠正**（前版为不存在的接口） |
| C3 | 前版 §二.2 表 | `SaveColoredPcd` 请求字段 `string file_path` | 请求字段是 `save_path`（`SaveColoredPcd.srv`；`lio_node.cpp` `req->save_path`） | **纠正** |
| C4 | 前版 §三.2 | 对齐逻辑在 `lio_node.cpp:376-453`，且用 `curvature_back` | `syncPackage()` 实际在 `lio_node.cpp:376-478`；帧尾时间用**末点 curvature 字段** `points.back().curvature / 1000.0`（`lio_node.cpp:389`），源码无 `curvature_back` 变量 | **纠正**（行号漂移 + 变量名不存在） |
| C5 | 前版 §二.1 表 | `image_topic` 默认 10–30 Hz、频率列为事实 | 频率**不在源码中**，是外部观测值，本轮无运行证据 | **降级为未验证** |
| C6 | 前版 §二.1/§四 | `/mapping/lio_odom` 被称为“纯激光前端” | 用词不准：`lio_odom` 是 IESKF（LiDAR 残差 + IMU 传播）输出；见 §1 结论 | **纠正措辞** |
| C7 | 前版 §三.3 | `p_c = R_cl R_il^T (R_wi^T (p_w - t_wi) - t_il) + t_cl` | 与源码等价（`image_processor.cpp:26-30,33-37` 的 `r_cw/t_cw` 展开），**保留** | 保留（未变） |
| C8 | 前版 §一 | “IESKF 仅含 LiDAR 点到平面残差、无视觉残差” | **源码层面成立**（见 §1.1 证据链），但前版未给证据；“无视觉残差”是**源码事实**，不是运行测量 | **保留结论、补齐证据** |

---

## 1. 视觉模块真实状态（源码事实 + 明确区分“无运行证据”）

### 1.1 已确认的源码事实（静态核验）

1. **滤波器状态不含任何相机量**：`State` 为 21 维 `(r_wi,t_wi,r_il,t_il,v,bg,ba)`，`V21D`/`M21D` 定长（`ieskf.h:11-12,38-53`）；无相机-IMU 外参、无时间偏移状态量。
2. **测量更新只有 LiDAR 点到平面残差**：`LidarProcessor::updateLossFunc` 组装 `pabcd` 点到面残差与法向（`lidar_processor.cpp:207-270`）；`IESKF::update()` 的 loss/stop 函数由 `LidarProcessor` 构造时注入（`lidar_processor.cpp:21-26`）。全仓不存在光度/重投影/光流残差项（`grep` 无匹配）。
3. **`ImageProcessor` 的全部对外行为是 Proto 投影着色**：`process()` 只做 `resize` + `cvtColor(BGR2GRAY)` + 缓存指针（`image_processor.cpp:15-23`）；`getLastestColoredCloud()` 用 `r_cw/t_cw` 把世界点投到相机、查表取 BGR（`image_processor.cpp:64-89`）。灰度图 `m_cur_img_gray` **被赋值后无任何消费者**。
4. **视觉前端辅助函数全是死代码**：`CVUtils::shiTomasiScore / getPatch / weightPixel / interpolateMat_8u`、`PinholeCamera::img2Cam / dpi` 在 `pinhole_camera.*` 之外**零调用点**（全仓 grep 无匹配）。这些正是 FAST-LIVO2 光流/面元补丁所需构件，说明是**预留脚手架而非在用实现**。
5. **相机外参不进入估计**：`ext_lc` 仅经 `r_cl/t_cl` 常数化后供着色投影使用（`lio_node.cpp:271-276`、`image_processor.cpp:26-52`）；`esti_il=false` 时 `r_il/t_il` 也由配置直接写死（`imu_processor.cpp` initialize 段）。
6. **`MapBuilder` 只在“非帧尾子周期”调用图像处理器**：`package.lidar_end==true` 走 LiDAR 分支；否则才 `m_image_processor->process(...)` 并渲染彩色点云（`map_builder.cpp:34-51`）。图像**不产生任何滤波器更新**，只影响 `color_world_cloud` 与 `/mapping/save_colored_pcd` 的累积点云。

### 1.2 严格结论（本轮不得越界表述）

- 可以说的：**截至 04b73f5，源码中不存在视觉残差进入状态估计的路径；视觉通道的唯一已实现功能是 LiDAR 点云着色与彩色 PCD 导出。**
- **不能说的**：“视觉融合已实现/已完成/精度已验证”——本轮**零运行**，没有任何“视觉分支实际被触发”的运行证据（相机未由任何仓库 launch 启动，见 §4.4）。
- **不得写进接口契约的研究假设**：把光度误差、稀疏光流、体素面元地图等列为**强制对外 API** 是无源码依据的。它们属于**未来升级需求**，只能在 §5.3 的“未来适配层”中以**需求项**形式出现，且必须标注为**未实现/待设计**，不得描述成现有接口。
- 上游参考：FastLIVO2 的上游是 **`https://github.com/hku-mars/FAST-LIVO2`**（与 `hku-mars/FAST-LIO` 即 FAST-LIO2 是**不同的**代码库）；本仓库当前实现**不对应**其视觉紧耦合分支，迁移/升级前须重新做接口契约评审。

---

## 2. 话题（Topics）逐行核实

发布/订阅名均为**相对名**（除服务外），实际名字 = 命名空间 + 相对名。命名空间由 launch 决定：`lio_launch.py` → `fastlio2`；`sensing.launch.py:132-138` 与 `universe.launch.py:83-87` → `mapping`。

| 方向 | 相对名 | 代码位置 | 类型 | QoS | 在 `namespace=mapping` 下 |
|---|---|---|---|---|---|
| 订阅 | `imu_topic`（YAML 键名）默认 `/livox/imu` | `lio_node.cpp:33,108-112` | `sensor_msgs/msg/Imu` | `SensorDataQoS()` | 话题名由 `lio.yaml:1` 给出，为**绝对名** `/livox/imu` |
| 订阅 | `lidar_topic` 默认 `/livox/lidar`；`lidar_type=="livox"` 时 `CustomMsg` | `lio_node.cpp:34,117-125` | `livox_ros_driver2/msg/CustomMsg` | `SensorDataQoS()` | `/livox/lidar`（`lio.yaml:2`） |
| 订阅 | 同上，`lidar_type!="livox"` 时 | `lio_node.cpp:126-132` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS()` | 同上 |
| 订阅 | `image_topic` 默认 `/camera2/camera/color/image_raw` | `lio_node.cpp:35,134-137` | `sensor_msgs/msg/Image` | `SensorDataQoS()` | `lio.yaml` **未设该键** → 生效值为代码默认（见 §3.3） |
| 发布 | `body_cloud` | `lio_node.cpp:169,600` | `sensor_msgs/msg/PointCloud2` | 队列深度 10000，默认 reliability | `/mapping/body_cloud` |
| 发布 | `world_cloud` | `lio_node.cpp:170,604` | `sensor_msgs/msg/PointCloud2` | 同上 | `/mapping/world_cloud` |
| 发布 | `color_world_cloud` | `lio_node.cpp:171,624-626` | `sensor_msgs/msg/PointCloud2`（`PointXYZRGB` 转出） | 同上 | `/mapping/color_world_cloud` |
| 发布 | `lio_path` | `lio_node.cpp:172,606` | `nav_msgs/msg/Path` | 同上 | `/mapping/lio_path` |
| 发布 | `lio_odom` | `lio_node.cpp:173,594` | `nav_msgs/msg/Odometry` | 同上 | `/mapping/lio_odom` |

- 发布函数在 `get_subscription_count()<=0` 时**直接 return**（`lio_node.cpp:482,480,493,517`）：无订阅者时无输出，调试时不要把“没数据”当成算法故障。
- `lio_path` 的 `poses` 只在 `lidar_end` 分支追加（`lio_node.cpp:605-619`），图像子周期不发。
- `world_frame` 是该节点唯一的世界系名（`lio.yaml:4` = `odom`）；路径 `header.frame_id` 初始化取它（`lio_node.cpp:86`）。

## 3. 服务（Services）逐行核实

| 服务名（**绝对名**） | 代码位置 | 类型 | 请求字段 | 响应字段 | 备注 |
|---|---|---|---|---|---|
| `/mapping/save_colored_pcd` | `lio_node.cpp:146-149` | `interface/srv/SaveColoredPcd` | `save_path` | `success`,`message` | 名字写成绝对名，因此**不受** `namespace` 影响；服务回调在独立 callback group（`lio_node.cpp:143-149`） |
| `/pgo/save_maps` | `pgo_node.cpp:164-165` | `interface/srv/SaveMaps` | `file_path`,`save_patches` | `success`,`message` | 保存单色/彩色地图与（可选）`patches/`；由 `pgo_node` 提供 |
| `/localizer/relocalize` | `localizer_node.cpp:76` | `interface/srv/Relocalize` | `pcd_path`,`x`,`y`,`z`,`yaw`,`pitch`,`roll` | `success`,`message` | 相对名 `relocalize` + `namespace=localizer` |
| `/localizer/relocalize_check` | `localizer_node.cpp:78` | `interface/srv/IsValid` | `code` | `valid` | 相对名 `relocalize_check` + `namespace=localizer`；**类型是 `IsValid`，不是 `RelocalizeCheck`**（前版 §三.4 错误，见 C2 同类问题） |
| `/hba/refine_map` | `hba_node.cpp:45` | `interface/srv/RefineMap` | `maps_path` | `success`,`message` | HBA；**本轮不启用**（契约禁 HBA） |
| `/hba/save_poses` | `hba_node.cpp:48` | `interface/srv/SavePoses` | `file_path` | `success`,`message` | 同上；HBA 只写位姿不写点云 |
| （订阅）`/initialpose` | `localizer_node.cpp:83-84` | `geometry_msgs/msg/PoseWithCovarianceStamped` | — | — | **绝对名**，与 `namespace` 无关 |

可复现调用（来自仓库 `README.md:176,188,226,231`，与上表一致；**本轮未执行**）：

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '<dir>', save_patches: true}"
ros2 service call /mapping/save_colored_pcd interface/srv/SaveColoredPcd "{save_path: '<dir>/colored_pcd'}"
ros2 service call /localizer/relocalize interface/srv/Relocalize "{pcd_path: '<dir>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

> 前版文档 §三.3 给出的 `/localizer/relocalize ... {init_pose: {header:..., pose: {...}}}` **字段不存在**，会转换失败；以 `Relocalize.srv` 的扁平字段为准。

---

## 4. timestamp / frame / 同步契约

### 4.1 时间戳提取

`Utils::getSec(header) = header.stamp.sec + nanosec*1e-9`（`utils.cpp:116-119`），`getTime(sec)` 反向（`utils.cpp:120-127`）。**全部用消息头时间戳，不使用接收时刻**。

### 4.2 激光帧时间窗与逐点时间

- Livox `CustomMsg`：`p.curvature = msg->points[i].offset_time / 1e6`，即**相对本帧首点的毫秒偏移**（`utils.cpp:23`，在 `livox2PCL` 内）。
- 通用 `PointCloud2`：仅当 `pcl2_time_field` 非空且 `pcl2_time_scale>0`，且该字段 datatype 为 FLOAT32/FLOAT64 时解析；`p.curvature = (t - t0_sec) * 1000.0`，其中 **`t0_sec` 是第一个“通过距离过滤后保留的点”的时间**，不是帧头时间、也不是被过滤掉的点的最小值（`utils.cpp:98-110`）。
  - 适配含义：若驱动给出的是**绝对时间**（典型如 Ouster `t` 纳秒），代码用“首个保留点”基准做差，得到的是**相对偏移**；scale 必须把字段换算到**秒**（Ouster `1e-9`，RoboSense `timestamp` 为 `1.0`）。字段缺失或类型不是浮点 → `curvature = 0`，即**该模式下不做扫描内运动补偿**（`utils.cpp:110-112`），这是**静默降级**，不是报错。
- 帧时间窗：`cloud_start_time = lidar_buffer.front().first`（帧头时间），`cloud_end_time = cloud_start_time + points.back().curvature/1000.0`（末点偏移换算为秒）（`lio_node.cpp:387-389`）。点云先按 `curvature` 升序排序（`lio_node.cpp:386-387`）。
- **契约前提（条件性，须逐雷达核实）**：`curvature` 在用之前必须满足“相对**消息帧头**毫秒偏移”这一语义。
  - 对 **Livox `CustomMsg`**，`offset_time` 由驱动按帧内相对时间给出，代码只是除 `1e6` 换成毫秒（`utils.cpp:23`），因此**源码层面**成立（依赖驱动契约，非本仓库保证）。
  - 对 **通用 `PointCloud2`**，代码的零点是“**首个通过距离过滤后被保留的点**”的时间 `t0_sec`（`utils.cpp:103,106`），**这不等于消息帧头时间**：若帧头与首点是同一时刻（常见驱动布局）则相等，但只要**抽稀改变了保留点**、**距离过滤删掉了更早的点**、或驱动本身让首点时间晚于帧头（如按列/按环输出、丢包补点），`t0_sec` 就会**偏离帧头**，`curvature` 也随之整体平移。
  - 因此 **不得**断言“上述 pcl2 路径都满足帧首语义”。必须对目标雷达逐项验证：(i) 每点时间字段是否为**相对帧起**或**绝对**时间；(ii) 抽稀/过滤后**保留的第一个点的时刻**与帧头时刻的差是否可忽略；(iii) 该差值是否随帧变化。
  - 若不满足：`cloud_end_time` 会等于“帧头 + 末点相对 t0 的偏移”，从而**整体平移帧窗**，进而错配 IMU 断点与图像对齐（`lio_node.cpp:387-389,403-412`）。**换雷达时这是必须实测的前提项，当前对 Airy/Odin1 未验证。**

### 4.3 IMU/图像与激光的对齐（`syncPackage()`，`lio_node.cpp:376-478`）

三种状态，均以“IMU 是否已覆盖到某个时间点”为门：

1. **无图像，或图像时间 > `cloud_end_time`**（`!has_image || !image_before_cloud_end`，`lio_node.cpp:416`）→ 纯 LiDAR 分支：
   - 要求 `last_imu_time >= cloud_end_time` 才继续（`lio_node.cpp:418`），否则本周期返回 `false` 重试；
   - 弹出 `imu_buffer` 中所有 `time < cloud_end_time` 的样本（`lio_node.cpp:425-429`）；
   - 弹出该激光帧，置 `lidar_end=true`（`lio_node.cpp:434-437`）。
2. **图像在帧窗内**（`image_start_time <= cloud_end_time`，`lio_node.cpp:405`）且 **`image_start_time >= cloud_start_time`** → 视觉子周期：
   - 若 `image_start_time < cloud_start_time`，丢弃该图像并 `return false`（`lio_node.cpp:442-448`）；
   - 要求 `last_imu_time >= image_start_time`（`lio_node.cpp:450-451`）；
   - 弹出 `imu_buffer` 中 `time < image_start_time` 的样本（`lio_node.cpp:464-470`）；弹出图像；`lidar_end=false`（`lio_node.cpp:472-473`）。
3. **`imu_buffer` 空**时返回 `false` 重试（`lio_node.cpp:427-428,448-449`），避免空 deque `front()` UB。

- 图像分支的意图是：**同一激光帧先按图像时间做一次 IMU 传播**，再在帧尾做 LiDAR 更新（`imu_processor.cpp` 的 `propagate_time_end = lidar_end ? cloud_end_time : image_time`）。
- **时钟域契约**：三种流必须同一时钟域；代码只做**单调性检查**（见 §4.5），**不做**统一时钟域校验、不做 PPS/PTP 状态读取。

### 4.4 发布时机与 frame

- 每次 `process()` 成功后：TF 与 odom 的 stamp = `lidar_end ? cloud_end_time : image_time`（`lio_node.cpp:605,594`），frame 为 `world_frame → body_frame`（`lio.yaml:3-4` = `odom → body`）。
- `body_cloud`/`world_cloud`/`lio_path` 仅 `lidar_end` 时发布，stamp = `cloud_end_time`；`color_world_cloud` 仅图像子周期发布，stamp = `image_time`（`lio_node.cpp:609-641`）。
- **相机未被任何仓库 launch 启动**：`sensing.launch.py` / `universe.launch.py` 均无相机节点；`README.md` 的相机命令是 `ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=camera -r __ns:=/vision` → 产出 `/vision/camera/color/image_raw`，而代码默认订阅 `/camera2/camera/color/image_raw`（`lio_node.cpp:35`）。**两者不一致**，故当前默认部署下图像分支不会生效（**待板端验证**：接相机后需显式把 `image_topic` 配成实际话题）。

### 4.5 乱序处理（唯一的时间防护）

四个回调都做 `timestamp < last_*_time` 检查，命中即**清空对应缓冲**并打 `"... Message is out of order"`（IMU `lio_node.cpp:321-325`、LiDAR `336-341` / `350-355`、图像 `364-369`）。
- **纠正前版 §三.2 相关暗示**：源码中**没有**“时间差 > 10 ms 记录警告”“Time inverted 专用日志”的实现；只有上述乱序告警与缓冲清空。任何软同步门限都必须以**新增实现**为前提（见 §5）。

---

## 5. 坐标系与外参契约

### 5.1 约定（源码）

- `ext_il = [t_x,t_y,t_z,q_x,q_y,q_z,q_w]`，读取顺序为 `Quaterniond(ext[6], ext[3], ext[4], ext[5])`，即**构造入参是 (w,x,y,z)**（`lio_node.cpp:251-254,269-273`）。语义 `p_imu = R_il p_lidar + t_il`。
- `ext_lc` 同格式；代码存的是**逆变换** `r_cl = R_lc^T`，`t_cl = -R_lc^T t_lc`（`lio_node.cpp:269-275`），语义 `p_cam = R_cl p_lidar + t_cl`。
- 两者在 `norm()>0` 时**无条件 `normalize()`**（`lio_node.cpp:262,273`），理由是 Eigen 的 `toRotationMatrix()` 假定单位四元数，YAML 文本截断会让 `R` 非正交并触发 Sophus 正交性断言。**契约：外参四元数写入 YAML 时不必手工归一，但必须是同一刚体变换的等价表示；不要依赖非单位四元数表达缩放。**
- 投影链（着色用）：`r_cw = r_cl·r_ilᵀ·r_wiᵀ`，`t_cw = -r_cl·r_ilᵀ·(r_wiᵀ·t_wi + t_il) + t_cl`（`image_processor.cpp:26-52`）。
- TF 树（`lio_node`）：`world_frame → body_frame`（定义 `lio_node.cpp:547-569`，内部 `sendTransform` 在 `:562`；调用点 `:605`），即 `odom → body`。
- `localizer` 的 `local_frame` 会被**入站 odom 的 `header.frame_id` 覆盖**（`localizer_node.cpp:340-341`），因此实际 `map→odom` 中 child 名取决于 `lio_odom` 的 `frame_id`（本仓库为 `odom`），YAML 的 `local_frame` 只是首帧前的初值。
- 相机模型为 **Brown–Conrady 径向+切向 5 参数**（`d0..d4` → `cv::undistortPoints`，`pinhole_camera.cpp:9,47-73`），**不是** KB/等距鱼眼模型。换鱼眼相机必须改模型或用等距去畸变后的内参。
- **外参必须来自本平台标定**：`lio.yaml:49` 的 `ext_il` 是本仓库默认机器人（MID360+内置 IMU）的标定值；部署 profile `config/lio_orin_nx.yaml` **不写** `ext_il`（缺键 ⇒ `loadParameters()` 只 `WARN` 并退回 `r_il=I, t_il=0`，`lio_node.cpp:266`），因为仓库内不存在目标雷达/IMU 的标定。任何静态 TF 启动器都不得写入未标定数值。
- **仿真恒等是仿真专用**：仿真启动文件在 `SIM_OVERRIDES` 里把 `ext_il` 覆盖成单位阵，仅因为合成点云渲染在机体系；这是**仿真专用**恒等，实机与验收中不得沿用，**不是**实机外参。

### 5.2 不变契约 vs 需适配契约

| 契约 | 现状 | 换雷达/平台时必须做的动作 | 依据 |
|---|---|---|---|
| Livox `CustomMsg` 解析 + 逐点 `offset_time`（ms） | 在用 | 无（Livox 系保持） | `utils.cpp:9-30` |
| 通用 `PointCloud2` 接入（`pcl2_time_field/scale`） | 在用（可配置） | 设字段名与**到秒**的 scale；字段须 FLOAT32/64；否则静默不做去畸变；**并须核实“首个保留点 ≈ 帧头”这一条件性前提（见 §4.2）** | `utils.cpp:32-113` |
| 倒序 IMU 插值去畸变 | 在用 | 无 | `imu_processor.cpp` undistort 段 |
| 静态窗 IMU 初始化（`imu_init_window_s>0`） | 在用（`lio.yaml` 开） | 若用 `lio_highres.yaml` 则**未开启**（缺键→0.0，回退 legacy 计数模式） | `imu_init.h:79-143`、`commons.h:53` |
| `imu_acc_scale` | 在用 | 依据驱动单位设 1.0 或 10.0 | `commons.h:39`、`lio.yaml:42` |
| 话题/服务名 | 在用 | 换命名空间需同步外部订阅者（`/pgo/save_maps`、`/mapping/save_colored_pcd`、`/localizer/*` 为绝对名，多数不变） | §2、§3 |
| `world_frame=odom`、TF `odom→body` | 在用 | 与 Nav2/localizer 同域时必须保持**唯一** `map→odom` 发布者 | `README.md:326-328` |
| `ext_il`/`ext_lc` 常量（`esti_il=false`） | 在用（`lio.yaml:49,50-59`） | 重新标定；外参不进入估计，标定误差直接转成建图误差。部署 profile `lio_orin_nx.yaml` **不写** `ext_il`（无标定）；仿真的单位阵恒等是**仿真专用** | `commons.h:68-69`、`lio_node.cpp:251-279` |
| C2 消融开关（`imu_acc_normalize`/`imu_init_mode`/`imu_init_min_samples`） | 源码在用、**缺省关闭**（`lio.yaml` 不写 ⇒ legacy 初始化 + 不归一化） | 只在 opt-in 的 `config/lio_c2_experimental.yaml` 开启；C2 回放**未执行**，**不得**当作已验证修复 | `commons.h:57-66`、`lio_node.cpp:231-238`、`imu_init.h:78-143` |
| 图像分支（着色） | 源码存在、**默认未接线** | 设 `image_topic`、启动相机、核对内外参 | §1、§4.4 |
| 视觉残差 / 光流 / 面元地图 | **未实现** | 属**需求**，非接口：需扩状态量、扩残差、扩地图与雅可比（见 §5.3） | §1.2；死代码仅 `pinhole_camera.*` |
| 视觉融合权重 / 光度权重参数 | **不存在** | 全仓无此类参数或 setter；`CVUtils::weightPixel` 是双线性插值权重（死代码），与“融合权重”无关（见 §5.3 C） | 全仓 grep |
| 足式里程计融合（电机编码器/腿部运动学，非轮式） | **未实现** | `lio_node` 仅订阅 IMU/LiDAR/Image（`lio_node.cpp:108-137`），无 twist/里程计输入；需新增观测模型，且该观测须先独立标定与资格评估 | §7 |

### 5.3 FastLIVO2 升级接口提案：**现有已实现接口** vs **未来适配层**

本节是**提案**，不是现状。现状只以 §1–§5.2 为准；本节存在的唯一目的，是升级时**不要把提案当成接口**、也不要发明不存在的 API。

**A. 已经实现、可直接依赖（升级时必须保持语义不变的接口）**

| 类别 | 已实现内容 | 位置 |
|---|---|---|
| 输入 | `imu_topic`（`sensor_msgs/msg/Imu`）、`lidar_topic`（Livox `CustomMsg` 或通用 `PointCloud2`）、`image_topic`（`sensor_msgs/msg/Image`，**仅供着色**） | `lio_node.cpp:108-137` |
| 输出 | `body_cloud`/`world_cloud`/`color_world_cloud`/`lio_path`/`lio_odom` | §2 |
| 服务 | `/mapping/save_colored_pcd`、`/pgo/save_maps`、`/localizer/*`、`/hba/*` | §3 |
| 配置 | 单个 `config_path` 的**扁平** YAML（`lio.yaml` 为 legacy 缺省；`lio_c2_experimental.yaml` 为消融 opt-in） | 参数文档 §1 |
| 内部可复用构件 | `PinholeCamera`（针孔 + Brown–Conrady 去畸变、`img2Cam`、`dpi`）、`ImageProcessor`（灰度缓存 + 投影着色） | `pinhole_camera.*`、`image_processor.*` |
| TF | `odom→body`（唯一发布者 `lio_node`） | §5.1 |
| 估计链路 | 只有 IMU 传播 + LiDAR 点到面残差；`State` 21 维无任何相机量 | §1.1 |

**B. 升级为视觉紧耦合（LIVO）必须新增的部分——全部【未实现/待设计】**

| 需求 | 为什么必须新增 | 现状 |
|---|---|---|
| 相机与 IMU 的**时间偏移**（估计或标定）+ 同步契约扩展 | 光度/重投影残差对帧间时间极敏感；当前 `syncPackage()` 只按“IMU 是否覆盖到图像时间点”对齐 | 不存在（§4.3） |
| **视觉残差**注入 `IESKF` | 当前 `updateLossFunc` 只有点到面残差 | 不存在（`lidar_processor.cpp:207-270`；全仓无光度/重投影项） |
| **视觉地图结构**（面元/补丁）与维护 | FastLIVO2 依赖 patch map；当前地图只有 ikd-Tree 点云 | 不存在 |
| **状态扩展**（相机外参/内参/时间偏移是否可估） | 21 维状态无相机量，协方差块也无对应项 | 不存在（`ieskf.h:11-12,38-53`） |
| 图像回调/子周期语义重构 | 当前图像只触发一次“按图像时间推进 IMU”的着色子周期 | 需重新设计（§1.1 第 6 条、§4.3） |
| 新的对外可视化/服务契约 | 彩色 PCD 的字段与语义可能随融合结果变化 | 需重新评审 §2/§3 |

**C. 明确不存在的 API（禁止在文档/代码里发明）**

- **不存在**任何“视觉权重 / 光度权重 / LiDAR-视觉融合权重”参数或 setter。唯一带 `weight` 字样的符号是 `CVUtils::weightPixel`（`pinhole_camera.h:15`、`pinhole_camera.cpp:231`）——它计算的是一个像素补丁的**双线性插值权重向量**，且**零调用点**，与融合权重无关。
- **不存在**“视觉质量阈值 / 特征数上限 / patch 尺寸 / 体素面元分辨率”等可调项；把它们写进配置契约是无源码依据的。
- **不存在**把 `ext_lc` 或 `esti_il` 当作“视觉外参在线优化开关”的语义：`ext_lc` 只进着色投影（`image_processor.cpp:26-52`），`esti_il` 在当前 `initialize()` 中**不分支**（始终用配置值写 `r_il/t_il`）。
- **不存在**任何视觉相关的运行证据（相机从未由仓库 launch 启动），因此“升级已完成/精度已提升”类结论一律不得出现。

**D. 升级的落地顺序（建议，非承诺）**：① 先补齐 §4.4 的图像接入与内外参（否则连着色都未验证）；② 再冻结时间同步契约（含相机-IMU 偏移）；③ 最后才谈状态/残差/地图扩展——每一步都需要新的接口评审与板端验证记录。

---

## 6. 结论

1. 04b73f5 的 FastLIVO2 是 **LiDAR-惯性里程计 + 点云着色**：估计链路只含 IMU 传播与 LiDAR 点到平面残差；相机只参与投影着色。
2. 对外接口的**权威清单**是 §2/§3 两张表（含 `body_cloud`/`world_cloud`/`color_world_cloud`/`lio_path` 与 `/pgo/save_maps`），前版文档的 `/mapping/save_map`、`/mapping/colored_cloud`、`file_path`、`RelocalizeCheck` 均已纠正。
3. 时间契约的**唯一硬要求**是三种流同钟域 + `curvature = 相对帧首毫秒偏移`；代码只做乱序检测，**不做**软同步门限与时钟域校验。
4. “视觉融合精度”在本版本**既无实现也无运行证据**；本文只登记源码事实，不给出任何视觉相关性能结论。

## 7. 待验证（需运行/板端，本轮预算 0）

- 各话题实际频率、延迟与消息连续性（无运行证据）。
- 图像分支在实机上是否被触发、`color_world_cloud` 的着色正确性（需相机 + 外参）。
- `pcl2_time_field` 在 Airy/Odin1 上是否存在、datatype 与量纲（见 `04b73f5_param_notes.md` 的“数据集/传感器分开”表）。
- `map→odom` 唯一性在实际 bringup 组合下的成立性（`nav2_bringup` 会拉起 AMCL；见 `04b73f5_p3_deployment.md` §2）。
- **外参标定**：`ext_il` 在目标平台上的实测值（部署 profile 缺键 ⇒ 当前是恒等占位）；`body_base_{x,y,z,roll,pitch,yaw}` 的实测值（且 `universe.launch.py:155` 的槽位缺陷须先修）。
- **C2 消融**：`imu_acc_normalize` / `imu_init_mode` / `imu_init_min_samples` 的实际效果——回放未执行，**无结论**（见 `04b73f5_param_notes.md` §1.5）。
- **视觉接入**：`image_topic` 与 `ext_lc`/`cam_*` 是否与实机相机一致（否则着色分支永不生效；见 §4.4）。
