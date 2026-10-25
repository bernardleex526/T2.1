# T2.1 第二轮审查修复与验证记录（2026-10-25，分支 `fix/t21-review-findings`，基线 `7c0a658`）

适用范围：以下结论来自单元测试、数值差分校验和 Gazebo 仿真（WSL2 Ubuntu 22.04 + ROS 2 Humble +
Gazebo Classic 11，x86 桌面，仿真约 0.35–0.4× 实时）。**不代表 Orin NX 实机**；Gazebo 激光为瞬时成像，
扫描内去畸变未被检验。仿真工具见 `docs/validation/gazebo_regression_20261018.md` §4。

## 1. 修复清单

| 级别 | 问题 | 位置 | 修复 | 验证 |
|:---:|:---|:---|:---|:---|
| 高 | HBA 局部 BA 梯度/Hessian 错误：`D_i` 列索引写成 `m*3`（应为 `m*6`），后半位姿梯度恒为 0；合并点丢掉每位姿点数；`optimize()` 不收敛 | `hba/blam.cpp` `updateJaccAndHess` | 用每个（平面, 位姿）点簇的 n、Σq、Σqqᵀ 闭式计算**精确**梯度与 Hessian（λ_min 一阶/二阶摄动，推导见代码注释） | `test_blam`：梯度与中心差分相对误差 < 1e-4，Hessian 与同一图下二阶差分 < 1e-3；5 个随机场景 `optimize()` 收敛（代价 ~1e-14） |
| 高 | HBA 用 Hessian **非对角块**作 BetweenFactor 信息矩阵（不对称、不定、符号相反、坐标系不对） | `hba/hba.cpp` `getAllFactors` | `BLAM::relativeInformation()`：转到左扰动（世界系）坐标，利用规范不变性的块拉普拉斯分解取 −sym(H_ij)，再经 Ad(T_j) 映射到 GTSAM 相对位姿误差，投影到正定锥 | 对称正定；两位姿窗口下 eᵀΛe 与精确代价的二阶差分一致（误差 < 30%） |
| 高 | 空点云帧（遮挡/全部超出量程/驱动空包）对空 vector 取 `back()`/`end()-1`，堆越界后进程崩溃 | `lio_node.cpp` `syncPackage`，`imu_processor.cpp` `undistort` | 回调里丢弃空帧（节流告警），`syncPackage`/`undistort` 加防护 | 复现：建图中插入 1 s 空帧，修复前 `Floating point exception` 退出；修复后继续运行，之后 76/80 帧里程计 |
| 高 | 腿速杆臂符号写反（`+ω×t_ib`，应为 `−ω×t_ib`）；建图初始化帧未经激光更新就融合腿速，首个位姿偏离地图 15–17 cm | `lio_node.cpp` | `map_builder/leg_odom.h` `legVelocityAtImu()`；首个激光更新之后才融合 | `test_leg_odom`（刚体有限差分）；Gazebo 1.0 m/s：开腿速 ATE 修复前 16.7 cm → 修复后 2.1 cm（同批次不开腿速 2.1 cm） |
| 中 | `universe.launch.py` 把 robot_pose 的 IMU 话题/量纲写死为 Airy（`/imu/data`、1.0），`lidar_vendor:=livox` 时 IMU 外推无数据、`/robot_pose_map` 不发布 | `universe.launch.py` | 启动时从所选 `config_file` 读取 `imu_topic`/`imu_acc_scale`（缺省与 lio_node 默认一致） | airy → (`/imu/data`, 1.0)，lio.yaml → (`/livox/imu`, 10.0) |
| 中 | robot_pose 轮询 gate 用 ROS 时钟 0.5 s、信任期用 steady 1.0 s；localizer 单线程执行器，`relocalize_check` 要等 ICP 结束 → 锁定后周期性 `gate_not_valid` | `map_pose_publisher.cpp`，`localizer_node.cpp` | 轮询改 steady 时钟、`gate_poll_period_s`（默认 0.25 s，须小于信任期）、丢失的请求超时撤销；localizer 改多线程执行器，intake/check 独立回调组 | Gazebo 1.0 m/s：锁定后 `gate_not_valid` 117 次 → 1 次；按接收时刻 RMSE 1.6 cm、最大 3.1 cm |
| 中 | PointCloud2 相对时间字段：curvature 以最早点 t0 为零点，扫描起点却仍取帧头 | `lio_node.cpp`，`utils.cpp` | `Utils::scanStartTime()`：绝对时间取 t0，相对偏移取 帧头 + t0 | `test_utils_preprocess` |
| 低 | 点云解析：未检查 data 长度/字段越界，xyz 只支持 FLOAT32，未对齐 `reinterpret_cast` 读取；Livox `filter_num=0` 死循环、`point_num` 越界 | `utils.cpp` | memcpy 按 datatype 读取，越界字段视为不存在，截断消息只解析完整点 | `test_utils_preprocess` 新增 5 例 |
| 低 | `esti_plane` 用 n'·p = −1 参数化，过世界原点（IMU 初始化高度）的平面退化成"竖直假平面" | `commons.cpp` | PCA（质心 + 最小特征向量），近共线邻域拒绝 | `test_lidar_processor`；数值校验：地面过原点的场景 IEKF 一次更新精确收敛；Gazebo A/B 无回退（见 §2） |
| 低 | 某个包内没有新 IMU 样本时，[末样本, 上一帧末] 被重复积分；首点可能被重复去畸变 | `imu_processor.cpp` | 从上次传播终点继续；首点到达后停止 | `test_imu_processor` |
| 低 | `lio_path` 无界增长；无激光时 IMU 缓冲无界 | `lio_node.cpp` | `path_max_poses`（默认 20000）；IMU 缓冲保留 60 s | 构建 |
| 低 | 时间回跳后 localizer 配对缓存与限频永久卡住 | `pair_cache.h`，`localizer_node.cpp` | 回跳 > 1 s 视为新时间纪元，清空并重置 | `test_pair_cache` 新增 2 例 |
| 低 | preflight 对 Livox PointCloud2 `timestamp`（纳秒，scale 1e-9 正确）误判阻塞 | `tools/preflight_config.py` | 仅 RoboSense 话题判阻塞，其它为提醒 | `tests/test_probe_tools.py` 新增 1 例 |
| 低 | preprocessor 的 `config.yaml` 从未加载（launch 传了节点不读的 `config_path`），且 config 目录未安装 | `universe.launch.py`，`lidar_preprocessor` | 以参数文件加载并安装；yaml 的 `cmd_vel_topic` 与 movement_gate 订阅的 `/cmd_vel_test` 一致 | 节点以参数文件启动、参数可读 |
| 低 | 编码规范/告警：C 风格转换、`-Wformat`、未用诊断变量、成员初始化顺序、PGO 弧度常数 57.324 | 多处 | `static_cast`、格式符、删除死代码、180/π | 干净重编：fork 代码 0 告警（上游 ikd_Tree 114 条未动） |
| 文档 | 21 维状态构成写错（含"重力"、漏外参）、NDT 未实现、腿速"有效消除滑动漂移"无依据、测试数、话题名、LPF 注释、Ouster `t` 语义 | README 等 | 已更正 | `tests/validate_docs.py` |

### 撤回项
第一轮报告的"IMU 初始化剩余段纯递推无上限（7.5 s → 0.75 m）"**归因错误**：那次运行开着腿速，0.75 m 主要来自
首帧腿速更新（已按上表修复）。专门的复现（首帧点云晚于起步 13 s）显示：原"最安静窗 + 递推 17 s"得 ATE 6.2 cm；
改选更近的窗（运动段，或含起步加速段的最近静止窗）反而因重力对齐偏差引入倾角，ATE 12.8 / 17.3 cm。因此窗口选择
保持原样，仅修正了 `static=1` 时仍打印"no static window"的误导日志；该场景作为已知局限写入
`docs/tuning_guide.md`（节点重启后应先静止再起步）。

## 2. Gazebo 结果（修复后代码）

| 用例 | 结果 |
|:---|:---|
| 建图 1.0 m/s，2 圈 120 m | 配准后 T1 RMSE 1.06 cm、p95 2.1 cm、≤5 cm 99.99%；单帧 p50/p95 33/42 ms（最终代码 1 圈：28.5/39.6 ms） |
| 建图 A/B，1.0 m/s 1 圈（站立 20 s） | 里程计 ATE：HEAD 1.3 / 1.9 / 1.4 / 1.8 cm（均值 1.6），最终代码 2.4 / 1.5 / 1.2 cm（均值 1.7），另有"最终代码 + 旧平面拟合"1.8 cm；去地面 RMSE 全部 1.1–1.2 cm。单次运行之间的波动约 1 cm，**无可测回退** |
| 定位 1.0 m/s，生产接线（IMU 外推 0.25 s） | 按接收时刻运动段 RMSE 1.6 cm、p95 2.3 cm、最大 3.1 cm；`gate_not_valid` 1 次（修复前 117） |
| 腿速 1.0 m/s 1 圈 | 开/不开腿速 ATE 2.1 / 2.1 cm（修复前开腿速 16.7 cm） |

说明：按"首帧锚定"的指标还受初始化重力对齐倾角影响（各次运行均约 0.21°，来自仿真加速度计零偏 ≈ 0.029 m/s²），
运行间波动明显；去地面 RMSE（1.1–1.2 cm）与配准后 T1 更能反映地图几何。
建图 2 圈、定位、腿速三项用例跑的 fastlio2 还带着后来撤回的初始化窗口改动；它们都是静止起步、首帧落在静止期，
两种规则选出的都是静止窗，对这些用例没有影响。A/B 与最终代码一行是撤回后的最终二进制。
