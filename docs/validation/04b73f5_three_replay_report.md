# 04b73f5 三次回放执行与预算管理总报告 — 最终交付（v2）

> **本文件取代** 仓库外旧历史输入文档 `docs/validation/04b73f5_three_replay_report.md`（历史输入，未入库，保留原文不改）。
> **勘误/取代范围**：旧文档中“回放 3 有条件通过”“回放 2 大幅优于上游”“回放 1 严重发散（作为官方精度）”等验收性结论**不再继承**；旧表内的延迟/丢帧/ATE 数字凡无来源者已在 v2 标注“未记录”。三次回放的**原始产物路径**保留。
> 基线 `04b73f553918b1dcc751feac69c0a2834f194432`（main）。本轮新增完整回放 **0** 次。

- 产物根：`fork/artifacts/04b73f5_evidence_audit_20260929_5V6fKb`
- 旧产物（只读）：`fork/artifacts/04b73f5_diagnosis_20260929_JWUGeL`
- 证据分级：`[观测]`/`[推断]`/`[条件]`/`[BLOCKED]`；审查状态：**预审，正式验证 NOT_RUN**。
- **交付状态：草案**——待协议测试与最终 review 后由 FinalIntegrator 统一更新（含 hash）。

---

## 一、回放预算执行审计（修订版）

历史硬约束 3 次完整回放，**3/3 已耗尽、结余 0**；本轮（证据审计）**授权 0、消耗 0**，**未启动 ReplayExecutor**。

| 编号 | 目标与完整 commit | 数据序列 | 产物目录 | 回放耗时 | 状态 | v2 裁定 |
|---|---|---|---|---|---|---|
| 回放 1 | 上游 `hku-mars/FAST_LIO` @ `7cc4175de6f8ba2edf34bab02a42195b141027e9`（工作区 CMakeLists 被本地改写，见 `freeze/version.json`） | `mcd_tuhh_night_09` ROS1 原始双包 1.0x | `…/replay_1_upstream` | 197.97 s | COMPLETED | 与 fork **不可公平比较**（世界系定义不同、配置/产物口径不同；上游侧差异量级未定）；旧 151.19 m 不是“官方精度” |
| 回放 2 | fork @ `04b73f553918b1dcc751feac69c0a2834f194432`（建图） | `mcd_tuhh_night_09` ROS2 转换包 1.0x | `…/replay_2_fork` | 243.58 s | COMPLETED | 精度 **BLOCKED**；后端候选瓶颈已定位（本回放内） |
| 回放 3 | fork @ `04b73f5…`（定位） | `tiers_indoor_office1_mid360`（B）对 Office2 图（A） | `…/replay_3_localizer` | 97.42 s | COMPLETED | 定位精度 **BLOCKED**；“1.24 cm 诊断”并入 `map←odom` 常量刚体像，不证 ICP 能力 |

- 外层 `eval/runs/` 另有 **60** 个历史 run 目录（真实在线运行次数远多于“3”）；“3”仅为本轮预算计数（`freeze/replay_budget.json`）。
- 旧冻结 `04b73f5_diagnosis_20260929_JWUGeL/freeze/replay_budget.json` 仍把 replay_3 标 `in_progress`，属**陈旧字段**。

---

## 二、固定列对比表（每列逐一列出，未测写“未记录”）

> 历史数值与新口径重算**分行**，并标用途限制。列定义：**地图点到面 RMSE** 为契约主指标（≤0.05 m）；**C2C** 为双向点到点；**ATE** 为轨迹、自由 SE3 对齐；**延迟**为定义明确的耗时。

### 行 A：replay_1_upstream（历史口径 = 旧 `eval_map.py` ICP 对齐）

| 列 | 值 | 来源/哈希 |
|---|---|---|
| 版本 | 上游 FAST-LIO（本地改写 CMakeLists 的 devel 二进制） | `freeze/version.json` |
| 完整 commit | `7cc4175de6f8ba2edf34bab02a42195b141027e9`（观察值 = 声明值） | `freeze/version.json` |
| 配置/哈希 | `replay_1_upstream/config/mcd_mid70.yaml`：`blind 0.5`、`point_filter_num 2`、`scan_line 4`、`ext_R diag(1,-1,-1)`、`ext_T [0.02,0,0.037]`；`time_sync_en=false`、`offset=0`；**无 `lidar_max_range`**、无 `gravity_align`；`r_wi=I`、重力为状态 | `config/mcd_mid70.yaml` |
| 序列/输入哈希 | `tuhh_night_09_mid70.bag` `e6b422ba…`、`vn200.bag` `96bf38f6…`；参考 `seeded.pcd` `103ba9a2…`；GT `d591ef94…` | `freeze/inputs_manifest.json` |
| 速度统计口径 | 序列同 replay_2；本报告口径（DatasetQualification 1 s 窗口，**待修订稿**）窗口均值 1.560 / 中位 1.714 m/s；旧口径（逐样本差分）1.655/1.722 | `datasets/gt_speed_metrics.json` |
| 地图点到面 RMSE | **151.192 m**（历史口径，ICP 对齐；**非**官方精度） | `replay_1_upstream/eval/map.json` |
| 双向 C2C | est→ref **189.929 m**；ref→est **5.93 m**（`map.json`/`frame_consistency.md` §9） | 同上 |
| 轨迹 ATE 及对齐 | raw 265.46 m；**自由 SE3 对齐 191.992 m**（帧无关） | `trajectory_gtinterp.json:tau_0` |
| 延迟及定义 | **未记录**（4 个日志 `Time cost` 出现 0 次） | `historical_claims.md` S6 |
| 输入帧数 | **未单独记录**（同序列扫描 1848） | — |
| 处理帧数 | 1845（`Log/mat_pre.txt` 行数；前 ~0.3 s/3 帧丢失） | `frontend_parity.md` |
| 输出帧数 | 1845 位姿（`traj/lio_odom.tum`） | `frontend_parity.md` |
| 覆盖率及分母 | 样本关联率 0.9929（1832/1845 估计戳；**非时间 coverage**）；按秒 coverage **未记录** | `trajectory_gtinterp.json` |
| 异常 | 严重发散；导出 `scans.pcd` 8,923,192 点全分辨率 | `frontend_parity.md`、`inputs_manifest.json` |
| 结论 | 与 fork **不可比较**（参考系/配置/产物口径三重不同）→ BLOCKED | — |
| 证据路径 | `…/replay_1_upstream/{eval/map.json,eval/trajectory_gtinterp.json,map/map.pcd}`（前端 175.5394° 探针 `audit/frontend_evidence/07_frame_probe.md` 已被 review 否决为真实系差解释，作者修订中，不得引用） | — |

### 行 B：replay_1_upstream（新口径重算 = v2，identity，诊断 ROI）

| 列 | 值 |
|---|---|
| 地图点到面 RMSE | **1.3077 m**（raw，固定分母 273,091；`pass=null`，`status=DIAGNOSTIC_ONLY_ROI`） |
| 双向 C2C | est→ref **2.2420 m**；ref→est **5.1177 m** |
| 用途限制 | 诊断 ROI = 参考图 AABB（`diagnostic_only`），transforms=identity、`transform_record=null`；**不得作验收** |
| 证据路径 | `evaluation/results/replay_1_upstream_map_accuracy_v2.json` |

### 行 C：replay_2_fork（历史口径 = 旧 `eval_map.py` ICP 对齐）

| 列 | 值 | 来源/哈希 |
|---|---|---|
| 版本 | fork 04b73f5 建图模式 | — |
| 完整 commit | `04b73f553918b1dcc751feac69c0a2834f194432` | `freeze/version.json` |
| 配置/哈希 | `replay_2_fork/config/lio.yaml`(`b1cf4ecd…`)：`lidar_max_range 30.0`、scan/map 0.5、`gravity_align: true`、`point_quality_thresh: 0.9`、`ext_il [0.02,0,0.037,1,0,0,0]`；`pgo.yaml` `1a91a53b…` | `freeze/inputs_manifest.json` |
| 序列/输入哈希 | db3 `021472df6c3f1d4f…`；参考/GT 同上 | `freeze/inputs_manifest.json` |
| 速度统计口径 | 同上（窗口均值 1.560 / 中位 1.714 m/s） | `datasets/gt_speed_metrics.json` |
| 地图点到面 RMSE | **1.6135 m**（ICP 对齐，rot 2.69°/t 0.616 m；**口径受坐标系问题约束**） | `replay_2_fork/eval/map.json` |
| 双向 C2C | est→ref **3.8445 m**；ref→est **3.2082 m** | 同上 |
| 轨迹 ATE 及对齐 | raw 115.97 m；**自由 SE3 对齐 3.1048 m**（LIO 里程计，帧无关） | `trajectory_gtinterp.json` |
| 延迟及定义 | **节点自报单帧处理耗时** `Time cost`：mean 22.14 / p95 38.43 / max 54.04 ms（**非端到端**） | `meta.json.per_scan_latency` |
| 输入帧数 | 1848（bag `/livox/lidar`） | `meta.json` |
| 处理帧数 | 1848（dropped 0） | `meta.json` |
| 输出帧数 | 1797 位姿（LIO）；524 关键帧（PGO） | `trajectory_gtinterp.json`、`map/poses.txt` |
| 覆盖率及分母 | 样本关联率 0.9967（1791/1797；**非时间 coverage**）；地图覆盖率 **未记录** | `trajectory_gtinterp.json` |
| 异常 | accepted=0；`map.pcd`=524 KF 拼接；重访区双走廊中位 6.196 m | `backend_evidence/independent_metrics.json` |
| 结论 | 精度 **BLOCKED**；后端候选瓶颈=correction 门（本回放内已证候选） | — |
| 证据路径 | `…/replay_2_fork/{eval/map.json,eval/trajectory_gtinterp.json,map/map.pcd,map/poses.txt,logs/pgo.log}` | — |

### 行 D：replay_2_fork（新口径重算 = v2，identity，诊断 ROI）

| 列 | 值 |
|---|---|
| 地图点到面 RMSE | **2.2188 m**（raw，固定分母 232,071；`p2pl_frac_within_0p05 = 7.04%`；`pass=null`） |
| 双向 C2C | est→ref **3.8938 m**；ref→est **4.0256 m** |
| 用途限制 | 同上（identity、diagnostic ROI、`transform_record=null`）；**不得作验收** |
| 证据路径 | `evaluation/results/replay_2_fork_map_accuracy_v2.json` |

> 说明：v2 主指标在**未施加任何 SE3** 下计算，故 replay_1 的 v2 值（1.31 m）**低于**其历史 ICP 值（151.19 m）——这正是“旧 ICP 对齐把关系改成了另一种估计量”的表现，**两组数不得混用**；两者均非验收值。

### 行 E：replay_3_localizer（定位，**不是建图精度**）

| 列 | 值 |
|---|---|
| 版本/commit | fork `04b73f5…`；二进制 `localizer_node`（无 build 哈希，`[推断]`==HEAD） |
| 配置/哈希 | `config/localizer.yaml` `f9b40c70…`（只含 base 键；`gate_*` 生效值来自编译默认，恰等于 HEAD 文件值） |
| 序列/输入哈希 | B 包 `tiers_indoor_office1_mid360`；GT `9624461…`；A 图 `667667d9…`；A 标定 `6c9e60bd…` |
| 速度口径 | TIERS ~0.53–0.60 m/s（行走）；**未按新窗口口径复算** |
| 地图点到面 RMSE | **不适用**（定位任务无参考图 Los） |
| 双向 C2C | 不适用 |
| 轨迹 ATE 及对齐 | raw（map 系、无对齐）RMSE **18.976 m** / median 20.760 / p95 23.320 / max 23.456；**刚体对齐诊断残差 0.01243 m**（= 位置轨迹刚体拟合残差，**非姿态真值、非 ICP 能力**） |
| 延迟及定义 | **未记录**（无端到端时延）；仅 wall 事件时刻 |
| 输入帧数 | bag 66.200 s；GT 7664 样本 @115.76 Hz |
| 处理帧数 | 未逐帧记录（accepted 走 DEBUG 未捕获） |
| 输出帧数 | 293 个合成位姿（`/tf` 293 时戳）；里程计位姿 609 中仅 293 被组合 |
| 覆盖率及分母 | 首末跨度比 **91.8%**（60.80/66.20 s，**≠coverage**）；样本关联率 44.0%（GT）/ 48.1%（tf↔odom）；按秒 coverage **BLOCKED** |
| 异常 | 3 次 `lost`、0 次 `recovered`；门控拒收**日志行 22，按计数下界 ≥35**（非真实次数）；30.52 s 静默窗口原因 `[BLOCKED]` |
| 结论 | **BLOCKED**；不宣称“通过/未通过” |
| 证据路径 | `…/replay_3_localizer/{traj,eval,logs,config}`、`audit/localizer_audit.md`、`localizer_evidence/**` |

---

## 三、口径更正清单（旧表 → v2）

1. 上游延迟/丢帧：**撤回**（无来源）。
2. “回放 3 有条件通过”：**撤回**（`map←odom` 常量 ⇒ 1.24 cm 只是里程计的刚体像）。
3. “回放 2 大幅优于上游”倍数：**撤回 → BLOCKED**。
4. 25 项回归中 `test_localizer_gate` 计 **10**（旧表写 9/9 有误），合计 25/25 以 `.gtest.xml` 为准。
5. 覆盖率不得用“293/293”冒充；跨度比、样本关联率、按秒 coverage 三者分列。

---

## 四、未记录项（不得补估计）

replay_1 延迟/丢帧/端到端时延/输入帧计数；上游 IMU 尺度倍率；replay_1 评测命令行（`map.json` 反推可得“显式 --est-map/--ref-map、cwd=<workspace_root>”，但参数无法确证）；replay_2 端到端时延、订阅队列丢帧；replay_3 逐帧 ICP 诊断量、端到端时延、按秒 coverage。
