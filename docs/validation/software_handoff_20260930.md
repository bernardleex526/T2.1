# WP2 T2.1 软件交接文档（去硬件）— 2026-09-30

> **本文定位**：面向"下一位接手软件工程师"的**可执行交接文档**。它**只覆盖无需硬件的软件工作**；硬件/Orin/标定/实机实时性一律在 §13 以一行范围边界带过，不展开、不作为本任务交付。
>
> **写作纪律（必须保持）**：
> - 所有数值来自**当前磁盘上的文件**（验收报告、机器可读 manifest、verdict JSON），不来自对话摘要；被取代的旧口径在 §12 明确撤回。
> - 证据分级沿用验收报告：`[实测]`（artifact/命令输出可读）、`[推断]`、`[条件]`（依赖未验证假设）、`[板端]`、`[阻塞]`。**绝不**把 `[推断]/[条件]` 当作 `[实测]` 引用。
> - **用户需求未全部达成**：建图 5 cm 在本地仿真 PASS；定位**在消息所标注测量时间戳上** PASS，但**当前时钟**要求 **FAIL**（§5）；连续性/可用性协议 **FAIL**。不得写成"全部达成"。
>
> **权威文件（优先级从高到低）**：
> 1. 验收报告：[`local_simulation_acceptance.md`](local_simulation_acceptance.md)（本目录，最终口径，**取代** `04b73f5_*.md` 旧稿的状态声明）
> 2. 详细计划要求与问题解决规范：[`../detailed_plan_requirements.md`](../detailed_plan_requirements.md)（根计划、7项真实验收、指标瓶颈拆解与质量方案）
> 3. 机器可读索引（本地历史证据）：`artifacts/nonhardware_delivery/manifest.json`（注：artifacts/ 与 .omp/ 为本地历史证据，受 .gitignore 保护不随代码仓库发布；关键指标已完全自包含）
> 4. 工作诊断报告（本地历史证据）：`.omp/reports/mapping_accuracy_diagnosis.md`、`.omp/reports/localization_coverage_diagnosis.md`
> 5. 旧稿（仅保源码级事实，状态以验收报告为准）：[`04b73f5_fastlivo2_interface.md`](04b73f5_fastlivo2_interface.md)、[`04b73f5_param_notes.md`](04b73f5_param_notes.md)、[`04b73f5_p3_deployment.md`](04b73f5_p3_deployment.md)、[`04b73f5_localizer_regression.md`](04b73f5_localizer_regression.md)、[`04b73f5_mapping_accuracy_diagnosis.md`](04b73f5_mapping_accuracy_diagnosis.md)、[`04b73f5_speed_accuracy_joint.md`](04b73f5_speed_accuracy_joint.md)、[`04b73f5_three_replay_report.md`](04b73f5_three_replay_report.md)

---

## 1. 范围与真实状态

**范围**：本地合成仿真链路（T1 建图精度 / T2 建图时延 / T3 定位）、受控离线干预（C1/C2）、ROS 2 包/配置/TF/时间/外参契约、T1.2/T1.4 接口契约、IMU/去畸变/调试参数、FastLIVO2 接口。

**基线**：`fork` @ `04b73f553918b1dcc751feac69c0a2834f194432`（分支 `main`）；**工作树脏**（§9.3），所有修复在**未提交**的工作树里。

### 1.1 一句话状态（真实）

| 需求 | 状态 |
|---|---|
| 建图精度 ≤ 5 cm（合成场景） | **PASS** `0.03145 m`（T1，denominator 214506，92.65 % within 5 cm，全部准入前置 true） |
| 建图时延 ≤ 1 个 0.1 s 帧（主机） | **PASS** 观测 input-ready→output-ready p95 `0.0486 s` |
| 定位精度 ≤ 5 cm，**在消息所标注测量时间戳上**（真实 `/robot_pose_map` 消费者） | **PASS** `0.0191 m`（n = 1320）；修复前 `0.1263 m` |
| 定位精度 ≤ 5 cm，**在消费者当前时钟上** | **未达成 / FAIL**：默认模式 `0.1625 m`；opt-in 严格模式**最终** `0.0591 m`（>5 cm）（以上为 2026-09-30 早前轮，权威目录 `artifacts/simulation_20260930/t3_availability_intake_fix/`）。**同日 recovery 修订**新增 `consumer_mode:=predict` 有界预测消费者：当前时钟 `0.0203 m`（n=1064）**MET**、后锁 20 Hz 及时输出 `0.9953` MET，但**全段（含启动）`0.8858` 未达标**；同修订 `current` 模式 `0.0482 m`（n=1075）与 0.0591 m **不同修订/不同 tick 普查，二者并存、旧值不被覆盖**（证据 `artifacts/software_metrics_recovery/step4_runtime/`） |
| 可用性 95 % / 0.1 s 龄期（审计附加协议，非用户 5 cm 需求） | **FAIL** 龄期门控可用性 `0.0000`（全 60001 查询跨度） |
| C1 / C2 受控实验与 80 运行重复稳定性 | **已执行**：C1 内部指标 `6.196 m → 0.153 m`；**Step 5 真实完成 80 运行（75 主路径 + 5 跨路径留出）串行实验闭环**（独占域 137，10 项前置门全 true）：**baseline** 15 运行稳定 PASS（T1 均值 0.02768 m，95% bootstrap CI [0.02712, 0.02825] m ≤ 0.05 m）；**c2_2** 15 运行稳定 PASS（0.02398 m，CI [0.02385, 0.02413] m）；**c2_3** 15 运行稳定 PASS（0.02808 m，CI [0.02736, 0.02877] m，成对差 CI [-0.00051, 0.00146] m 跨 0 **未证明改进**）；**c2_1**（14 PASS / 1 BLOCKED）与 **c2_combined**（14 PASS / 1 BLOCKED）各 1 次因评测器 1 个图外点触发 `BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS`（`pass=null`，如实留档不重试）；**主路径总体 stable_meets_target = False**（73 PASS / 2 BLOCKED）；**跨路径留出** 5/5 PASS（0.02991 m，CI [0.02836, 0.03145] m，不声称跨场景泛化）。详见 §4.3 及 [`step5_repeats/`](../../artifacts/software_metrics_recovery/step5_repeats/) |
| 历史数据集验收（MCD/HILTI 测绘底图） | **BLOCKED**（坐标链/ROI/独立变换未闭合，§7） |
| 硬件 / Orin / 标定 / 实时 | **NOT RUN**（§13，`[板端]`） |

> **不得声称需求达成**：当前时钟 5.91 cm / 连续性可用性 0 均 FAIL（§5）。建图 5 cm 与"消息所标注测量时间戳定位 5 cm"是**在合成场景**下 PASS。**同日 recovery 修订**下新增 `consumer_mode:=predict`：当前时钟 0.0203 m MET、后锁 0.9953 MET，但**全段（含启动）0.8858 未达标**，且旧 `0.1 s`/95% 协议仍 FAIL——该结论只限本合成场景/本轨迹/单种子，不代表真实数据、整机或稳定泛化。

### 1.2 需求来源 vs 附加协议（口径边界）

- **用户目标**：建图精度 ≤ 5 cm；联合目标"速度 0.5–1.0 m/s **且**精度 ≤ 5 cm"；定位精度（消费者口径）。用户**未指定度量**。
- **采用的度量/统计契约 = 本交付选定的操作化（不是用户逐字要求）**：建图 = 估计图 → 合成参考几何的**点到面 RMSE** 阈值 `0.05 m`（`raw_p2pl_rmse_m`）；定位 = 真实 `/robot_pose_map` 的**时间关联 ATE** 阈值 `0.05 m`。度量选择（点面 vs 其它、ATE 关联规则、阈值）由本交付确定并冻结；达标/未达标均须带该口径名。
- **附加协议（评审引入，非用户需求）**：`min_coverage ≥ 0.95` 覆盖门、`hold_validity_s = 0.1 s` 修正确正龄期契约。二者是**独立的协议选择**，失败不等于用户 5 cm 目标失败，但**必须如实报告**。
- **易混的两个 0.1 s**：T2 的 **0.1 s 帧周期时延**判据 ≠ T3 的 **0.1 s 修正龄期**契约。**不得混用**。

---

## 2. 需求 / 证据 / 完成度矩阵（实现 / 执行 / 验收 三分）

| # | 项目 | 实现（代码/配置） | 执行（是否跑过） | 验收（结论） | 权威证据 |
|---|---|---|---|---|---|
| R1 | T1 建图精度 ≤ 5 cm | 仿真生成器+冻结评测器 | 已执行 | **PASS** 0.03145 m（合成场景，单场景单种子） | [`t1_verdict.json`](../../artifacts/mapping_corrected_20260930/baseline/mapping/t1_verdict.json) |
| R2 | T2 建图时延 | 记录器+时延判据 | 已执行 | **PASS**（主机）p95 0.0486 s | [`t2_verdict.json`](../../artifacts/mapping_corrected_20260930/baseline/mapping/t2_verdict.json) |
| R3 | T3 定位@**消息所标注测量时间戳**（真实消费者） | `map_pose_stamp` 修复 | 已执行 | **PASS** 0.0191 m（n=1320） | [`consumer_stamp_fix_acceptance/t3_verdict.json`](../../artifacts/simulation_20260930/t3_availability_intake_fix/consumer_stamp_fix_acceptance/t3_verdict.json) |
| R4 | T3 定位@**当前时钟（默认）** | 无传播契约 | 已执行 | **未达成** 0.1625 m | 同上（`current_clock_error_m`） |
| R5 | T3 定位@**当前时钟（opt-in `current_pose_mode`）** | 已实现且评审通过（无 mustfix） | 已执行 | **FAIL** 0.0591 m（>5 cm）；消息标注戳 0.0223 m（MET） | [`current_pose_mode_acceptance/t3_verdict.json`](../../artifacts/simulation_20260930/t3_availability_intake_fix/current_pose_mode_acceptance/t3_verdict.json) |
| R6 | 可用性 95 % / 0.1 s 龄期（附加协议） | 审计协议 | 已执行 | **FAIL** 龄期门控 0.0000；因果可用 0.8900 全段 / 1.0000 锁后 | 同上（`availability_*`） |
| R7 | C1 受控回访干预 | 离线干预 harness | 已执行 | **MEASURED（仅内部指标）** 6.196→0.153 m | [`provenance.json`](../../artifacts/local_loop_intervention/results/provenance.json) |
| R8 | C2 单因素五臂消融与 80 运行重复稳定性 | 实验开关（opt-in）+ repeat runner | 已执行（全 80 运行串行闭环） | **主路径 73 PASS / 2 BLOCKED，留出 5 PASS**：baseline(15/15 PASS, T1 0.02768 m, CI [0.02712, 0.02825] m)、c2_2(15/15 PASS, 0.02398 m, CI [0.02385, 0.02413] m)、c2_3(15/15 PASS, 0.02808 m, CI [0.02736, 0.02877] m，成对差 CI 跨 0 未证明改进)；c2_1 与 c2_combined 各 1 次被评测器阻断(pass=null)；主路径全臂稳定达标未成立，留出路径 5/5 PASS (0.02991 m，CI [0.02836, 0.03145] m) | 80 run 证据根：[`step5_repeats/`](../../artifacts/software_metrics_recovery/step5_repeats/)（汇总 [`repeat_metrics.json`](../../artifacts/software_metrics_recovery/step5_repeats/repeat_metrics.json)、清单 [`repeats_manifest.json`](../../artifacts/software_metrics_recovery/step5_repeats/repeats_manifest.json)）；单 run 历史表：[`c2_arm_table.json`](../../artifacts/mapping_corrected_20260930/c2_arms/c2_arm_table.json) |
| R9 | T1.2 传感器抽象（Airy/Odin1） | 参数化配置+话题适配 | 部分 | **PARTIAL / BLOCKED**（P1–P7 板端） | 验收报告 §9.6 |
| R10 | T1.4 足式状态估计依赖 | **仅接口契约** | 未执行（外部依赖不可用） | **EXTERNAL DEPENDENCY NOT AVAILABLE/UNQUALIFIED** | [`04b73f5_p3_deployment.md`](04b73f5_p3_deployment.md) §7.2 |
| R11 | IMU 初始化/去畸变/调试参数 | 已实现（源码级） | 部分 | **DOCUMENTED**（C2 策略未验证） | 验收报告 §10 |
| R12 | FastLIVO2 接口 | 接口契约文档 | — | **DOCUMENTED** | [`04b73f5_fastlivo2_interface.md`](04b73f5_fastlivo2_interface.md) |
| R13 | 历史数据集验收 | — | **未执行** | **BLOCKED** | §7、`next_experiment_card.md` |
| R14 | 硬件 / 实时 | — | 未执行 | **NOT RUN** `[板端]` | §13 |

> **"实现了"≠"执行了"≠"验收通过"**：R5 实现了且评审通过，但**执行结果 FAIL**；R10 只有接口契约（实现≠依赖存在），执行与验收都不成立。

---

## 3. 已完成的源码修改（按组件分组）

> 全部位于**未提交的工作树**（§9.3）。改动清单与 sha256 见 [`manifest.json`](../../artifacts/nonhardware_delivery/manifest.json) 的 `source_changes_delivering_the_interfaces`。

### 3.1 仿真器 / 数据生成器（`simulation/`）
- `synthetic_data/traj_common.py`：修复闭环航向表按自身长度取模插值造成的 ~360° 偏航混入（实测修复前 `gyro_meas` −309 rad/s、18 °/ms）；修复站着→行走的速度阶跃（实测 251 m/s² = 25 g）；**逐射线精确渲染**取代 twist 近似渲染（修复前均值差 1–5 cm、p95 3–19 cm，剪影射线最大 9.1 m；修复后 30720 射线最大误差 2.2e-6 m）。
- `synthetic_data/generate_test_bag.py`：**就绪发布**时间契约（每帧在**末点就绪**时发布，header 保留首点测量时刻）；`--check` 闭环自检，任何运动学不连续即 fail-closed。
- `synthetic_data/scene_reference.py`：参考场景/ROI/GT 生成。
- 关键点：生成器**值确定**（解码后点数组逐位相同、参考图与 GT TUM 字节相同），但**bag 文件哈希不是可复现判据**（差异在 CDR 元数据填充区）；**节点输出非逐位可复现**。

### 3.2 评测器（`artifacts/.../evaluation/`，冻结、未改动阈值）
- `map_accuracy_eval.py`（sha256 `408c4786…`）、`gt_time_assoc_eval.py`（sha256 `a512e506…`）、`cross_check_map_eval.py`、`evaluator_verification.json`。
- **未改动**评测器、阈值、门（同一文件、同一 sha256）；PASS 来自修复生成器/传感器模型，不是放宽门限。
- `map_accuracy_eval.py` 升级到 **v2.3.1** 的 R38 处（**属于历史数据集审计侧**，见 §7/§12）：outside-attribution 由**声明式**改为**实测成员几何**、ROI 取代（supersession）强制。

### 3.3 IMU 实验开关（opt-in，非默认）
- `lio.yaml` **不含**任何 C2 键（保持 legacy 默认）；C2 键只出现在 `lio_c2_experimental.yaml`：`imu_acc_normalize`、`imu_init_mode`/`imu_init_min_samples`、`pcl2_filter_phase`。
- 源码：`map_builder/commons.h`、`map_builder/imu_processor.{h,cpp}`、`map_builder/imu_init.h`、`utils.{h,cpp}`、`lio_node.cpp`。
- **性质**：opt-in 消融开关，**不是已验证的修复**；C2.3 在合成排序上是**预期零结果**，不得外推到厂商 bag。

### 3.4 定位器（`src/localization/localizer/`）
- `src/localizers/pair_cache.h`（新）：有界、按 stamp 键控的近似时间配对缓存，解决 `ApproximateTime` 卡死。
- `src/localizers/lock_validity.h`（新）：带新鲜度界的锁有效性，避免 `relocalize_check` 用陈旧证据回 `valid`。
- `src/localizers/icp_localizer.{h,cpp}`、`fitness.h`：每分辨率共享预建搜索索引 + `force_no_recompute`，消除每次更新的目标 KD-tree 重建；分阶段计时。**经授权**的 ICP 代价改动 `refine_max_iteration 15 → 8`（平均对齐 160 → 102 ms，达 10.0 Hz，精度无损失）。
- `localizer_node.cpp`：配对缓存/锁有效性接线、latest-sample 摄入、**仅当已发布 `map←odom` 修正真正移动时**才发 TF 样本（P2a 修复）、可用性报告。
- `test/test_pair_cache.cpp`(7)、`test_lock_validity.cpp`(5)、`test_localizer_gate.cpp`(12)：摄入/有效性回归 + 两条"广播不变式"门回归。
- `config/localizer.yaml`：`validity_timeout_s`、`report_period_s`、实测 `refine_max_iteration: 8`。
- `CMakeLists.txt`：去掉 `message_filters`，加入两个回归目标。

### 3.5 TF（`src/launch/universe_launch/launch/universe.launch.py`）
- 修复 `body→base_link` 静态发布器的 **roll/pitch 槽位互换**（旧位置式参数把 roll/pitch 传进 pitch/roll 槽），改用 Humble **具名参数**；非零角度 scoped smoke 精确复现四元数（`--roll 0.3 --pitch -0.2 --yaw 0.1` ⇒ `(0.153439,-0.091158,0.064071,0.981856)`）。
- TF 契约（源码核验）：`map→odom` 有且仅有一个发布者/周期（`pgo` 与 `localizer` **不同时启动**）；`odom→body` 仅 `lio_node`；`body→base_link` 仅 `universe_launch` 的静态发布器；**无 `body→lidar`/`body→imu` 传感器 TF**（`body` 即 IMU 机体系，`ext_il` 语义）。

### 3.6 消费者（`src/localization/robot_pose/`）
- `src/map_pose_stamp.hpp` + `map_pose_publisher.cpp` 的**时间戳修复**：发布**解析变换自身的 header stamp**，不再用 `now()`。同一输入下消息标注戳 ATE `0.1263 → 0.0191 m`。
- `src/map_pose_current.hpp`（新）：`GateTrust` + 三链组合 `T_map_odom · T_odom_child · T_child_base`，opt-in `current_pose_mode:=true`（默认 `false`），严格 fail-closed、不外推。
- `map_pose_publisher.cpp`：门答案**接收时**派生、1.0 s 到期（即使请求待处理），尊重 `odom.child_frame_id`，`child→base_link` 在精确 odometry stamp 查询；暴露 `/robot_pose_map/status`。
- `test/test_map_pose_stamp.cpp`(1)、`test/test_map_pose_current.cpp`(7)：新增回归。

### 3.7 记录/评测脚本（`simulation/scripts/`）
- `record_localization.py`：接收因果组合 + 双普查 + `/clock` 锚定在线流 + 修正龄期/时钟滞后统计 + `tf_availability.json` 时间线。
- `record_consumer.py`（新）：记录**真实** `map_pose_publisher` 的 `/robot_pose_map` + `/tf` + `/clock`，20 Hz 查询普查。
- `test_t3_localization.py`：P2b 修复（显式读 `stages.localization`，路由/先验溯源缺失即 **fail-closed**）；全 GT 支撑可用性门、到达因果普查、在线（查询时钟）精度为用户判据、时间戳关联精度为诊断。
- `test_t1_accuracy.py`、`test_t2_speed.py`、`check_bag_availability.py`、`make_report.py`、`run_{mapping,localization,eval_pipeline,all_tests}.sh`、`persist_evidence.sh`、`finalize_runs.sh`。
- 诊断工具：`simulation/diagnostics/*`（见 [`README`](../../simulation/diagnostics/README.md)），含 C2.3 排序判定、缝合隔离、残差归因、评测器地板检查。

---

## 4. final 5 臂映射：数值表、计时与统计局限

**唯一权威执行表**：[`c2_arms/c2_arm_table.json`](../../artifacts/mapping_corrected_20260930/c2_arms/c2_arm_table.json)（run root `/tmp/sim_abl_final`，一次性 build、顺序执行、隔离 domain 137、`--skip-localization`）。

### 4.1 五臂表（[实测]）

| 臂 | T1 raw p2pl (m) | T1 判定 | within 5 cm | T2 p95 (s) | T2 判定 | 前置全 true | bag 契约全 true |
|---|---|---|---|---|---|---|---|
| baseline | 0.02619 | PASS | 0.9745 | 0.0499 | PASS | ✔ | ✔ |
| c2_1 加速度归一化 | 0.02925 | PASS | 0.9484 | 0.0540 | PASS | ✔ | ✔ |
| c2_2 first-batch 初始化 | 0.02398 | PASS | 0.9923 | 0.0487 | PASS | ✔ | ✔ |
| c2_combined | 0.02250 | PASS | 0.9927 | 0.0518 | PASS | ✔ | ✔ |
| c2_3 采样相位 | 0.02555 | PASS | 0.9788 | 0.0567 | PASS | ✔ | ✔ |

### 4.2 读法与统计局限（必须随表引用）

- **五臂判定全为 PASS**：每臂 `t1_pass_preconditions_all_true = true`、`bag_availability_all_true = true`，评测器状态 `PASS`。
- **数值不等（0.0225–0.0293 m），但落在"同代码 run-to-run 波动"内**（两次独立 baseline 运行区间 0.024–0.032 m）。**未做显著性检验** ⇒ 正确表述是"**没有臂显示出超过该波动的改进**"；这**不是**等价/零假设证明。
- **`0.03145 m` 与 `0.02619 m` 不矛盾**：前者是另一次运行（`/tmp/sim_fixed3` baseline）的独立测量，同一代码同一 bag 的不同运行。**引用时须带各自 run root**。
- **C2.3 是合成排序上的零结果**：合成生成器按发射时间序发出、每方位 48 个 beam 共享发射时刻，phase 0/1 保留同一首点、`t0` 位移实测 `0.000000 s` ⇒ 零结果是**预期**，**不得**外推到厂商 bag。厂商侧（IndoorOffice1 `/mid360/livox/lidar` 为 `line == i % 4`）phase 行为确有不同（`t0` 位移 4.768 µs）。
- **T2 为主机口径**，测量期间无其它 ROS 运行。
- 来源：C2 五臂是**单因素节点配置覆盖**，非因果消融设计；合成结论**不得**推广到厂商数据。

### 4.3 Step 5: 80 运行重复稳定性与 C2 证据闭环（真实执行 2026-09-30）

依据计划 Step 5，在独立构建 overlay（`build_workspace/install/setup.bash`）及独占域 ROS_DOMAIN_ID=137 下完整执行并闭环 80 运行串行实验（主路径 75 次 + 跨路径留出 5 次），**不因运行时间缩减，不自动重试，失败/BLOCKED 全链留档**。

**权威证据目录**：[`artifacts/software_metrics_recovery/step5_repeats/`](../../artifacts/software_metrics_recovery/step5_repeats/)
- 汇总结算：[`repeat_metrics.json`](../../artifacts/software_metrics_recovery/step5_repeats/repeat_metrics.json)（sha256 `4c0ef8f19a45f68f2644637cbf7d7c3c40fcd7d352c72b14e9fcc125081420fb`）
- 执行清单：[`repeats_manifest.json`](../../artifacts/software_metrics_recovery/step5_repeats/repeats_manifest.json)（sha256 `f190d88b952191e50accf16ca7fb416ef8470b00916f829651f7e9d2172ce487`）
- 运行脚本：`simulation/scripts/run_metric_repeats.py`（sha256 `eec3fa03b9bb10ad039c5b432902b19c5723493e34deb392aacd738356b45d4d`）
- 总体计数：`runs_executed: 80`，`runs_complete: 80`，`runs_pass: 78`，`runs_blocked: 2`，`runs_fail: 0`，`runs_gates_failed: 0`，`runs_attempted_more_than_once: 0`。

#### 4.3.1 主路径 75 次运行（5 臂 × 5 种子 × 3 重复）

每 seed 轨迹仅在 `imu_noise.seed` 变化（`traj_common.apply_variant`，geometry_fingerprint 断言严格一致 `eb1237fcfecd79f5703cf0fa761fbb10f0c6c1a8ab70868a4a719515a533997e`）；每 seed 的 bag 仅生成 1 次并由 5 臂 15 次运行重用；全部 75 运行 10 项前置门（bag_reused/odom/latency/interface/bag_availability/frame_chain/t1_inputs/seed_geometry/bag_files/bag_content）全部为 true。

Bootstrap 统计规范：抽样单元为 **seed 索引**（5 个索引有放回抽样，固定 RNG seed `20260930`，10000 次），各臂共享同一组抽样索引矩阵（sha256 `b6a863e9b958d24d1405cb31e4c9a0934f9b553905493723c224664f26ecdc19`），置信区间取 2.5% 与 97.5% 分位点。

| 臂 | 运行数 | 判定通过数 | T1 RMSE 均值 (m) | 95% Bootstrap CI (m) | T1 ≤ 5cm 上界 | T2 p95 均值 (s) | 95% Bootstrap CI (s) | 单臂稳定达标 |
|---|---|---|---|---|---|---|---|---|
| **baseline** | 15/15 | 15 PASS | **0.02768** | [0.02712, 0.02825] | 0.02825 ≤ 0.05 (PASS) | **0.05228** | [0.04834, 0.05535] | **TRUE** |
| **c2_1** (acc_normalize) | 15/15 | 14 PASS / 1 BLOCKED | — (seed45 未完备) | — (不可插值) | — | 0.05108 | [0.04876, 0.05339] | **FALSE** (BLOCKED 阻断) |
| **c2_2** (first_batch) | 15/15 | 15 PASS | **0.02398** | [0.02385, 0.02413] | 0.02413 ≤ 0.05 (PASS) | **0.05093** | [0.04744, 0.05450] | **TRUE** |
| **c2_3** (sampling_phase) | 15/15 | 15 PASS | **0.02808** | [0.02736, 0.02877] | 0.02877 ≤ 0.05 (PASS) | **0.05117** | [0.04985, 0.05266] | **TRUE** |
| **c2_combined** | 15/15 | 14 PASS / 1 BLOCKED | — (seed42 未完备) | — (不可插值) | — | 0.05643 | [0.05507, 0.05755] | **FALSE** (BLOCKED 阻断) |

*各 seed 均值明细 (T1 RMSE, m)*：
- baseline: seed41=0.02858, seed42=0.02689, seed43=0.02813, seed44=0.02783, seed45=0.02700
- c2_1: seed41=0.02945, seed42=0.02907, seed43=0.03107, seed44=0.02881, seed45=null (BLOCKED)
- c2_2: seed41=0.02424, seed42=0.02386, seed43=0.02393, seed44=0.02381, seed45=0.02405
- c2_3: seed41=0.02847, seed42=0.02737, seed43=0.02699, seed44=0.02827, seed45=0.02932
- c2_combined: seed41=0.02380, seed42=null (BLOCKED), seed43=0.02382, seed44=0.02356, seed45=0.02322

#### 4.3.2 成对 Bootstrap 差值检验 (vs baseline)

- **c2_2 vs baseline**：成对均值差 **-0.00371 m**，95% CI **[-0.00421, -0.00319] m**。
  * 纪律断言：CI 全部落在 0 以下，记录为 `ci_below_zero_no_equivalence_claim`；按计划契约，**不得声称等价或宣称算法因果改进**。
- **c2_3 vs baseline**：成对均值差 **+0.00040 m**，95% CI **[-0.00051, +0.00146] m**。
  * 纪律断言：**CI 跨越 0**，严正判定为 **`improvement: not_demonstrated`**（无统计显著改进）。
- **c2_1 与 c2_combined vs baseline**：因存在 BLOCKED 运行导致成对种子集不完备，按规约阻断输出，**不通过删除未完备 seed 制造虚假改善**。

#### 4.3.3 阻断事件根因与客观留档 (2 次 BLOCKED)

全 80 运行中无一例崩溃、无一例超时、无一例时延越界（T2 全部 PASS）、无一例前置门失败（10 项门全 true）。2 次未通过全部由冻结评测器 T1 的空间支持域边界阻断（`BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS`）：
1. `main/seed42/c2_combined/repeat3`: 227,138 个估计采样点中恰有 **1 个点**（0.0004%）落在 frozen support region 外且无成员几何归因，T1 verdict = BLOCKED（raw RMSE 0.02486 m，coverage 98.66%）。
2. `main/seed45/c2_1/repeat2`: 212,084 个估计采样点中恰有 **1 个点**（0.00047%）落在 frozen support region 外且无成员几何归因，T1 verdict = BLOCKED（raw RMSE 0.03165 m，coverage 92.21%）。
- **主代理与交接契约执行纪律**：严禁扩大 ROI、严禁调整支持域、严禁重试以覆盖。**两组阻断完整留档，全实验主路径 `all_runs_pass = False`、`stable_meets_target = False` 如实成立**。

#### 4.3.4 跨路径留出检验 (5 次运行)

使用独立留出映射路径 `test_trajectory_localization.json`（60s，geometry_fingerprint `36ef7077e3070bf3aa1f82b0cc8617e97c795e7fc67d1bcbb363ae2c8a10d361`），baseline 覆盖 seeds 41–45 各 1 次：
- **5/5 PASS**（0 失败，0 阻断，10 项门全 true）。
- T1 RMSE 均值 **0.02991 m**，95% Bootstrap CI **[0.02836, 0.03145] m ≤ 0.05 m**。
- T2 p95 均值 **0.06141 s**，95% Bootstrap CI **[0.05662, 0.06663] s ≤ 0.1 s**。
- 各 seed 明细：seed41=0.03136, seed42=0.02757, seed43=0.02991, seed44=0.03231, seed45=0.02839 m。
- **边界**：此路径为同一合成场景下的另一条运动路径，属于**跨路径留出**，**严禁声称跨场景泛化**。

#### 4.3.5 输入独立性与解码内容哈希

10 个独立 bag（主路径 5 个 + 留出路径 5 个）经 `bag_content_hash` 解码校验，排除 CDR padding 干扰（`padding_free=true`），10 个内容哈希彼此互异（`seed_inputs_distinct = True`），证实随机性真实注入：
- 主路径：seed41 (`bf8022ac...`)、seed42 (`cf6bb446...`)、seed43 (`a372eefd...`)、seed44 (`41cb0d8a...`)、seed45 (`f5aa29b7...`)。
- 留出路径：seed41 (`486290ed...`)、seed42 (`f125a24f...`)、seed43 (`33716d89...`)、seed44 (`0e9a17a3...`)、seed45 (`cec56ebc...`)。

---

## 5. 定位指标：默认 / opt-in 消息所标注测量时间戳 / 当前时钟（三种口径，绝不混用）

### 5.1 三口径表（权威目录：`artifacts/simulation_20260930/t3_availability_intake_fix/`）

| 口径 | 定义 | 实测 | 结论 |
|---|---|---|---|
| **消息所标注测量时间戳精度（默认模式）** | 真实 `/robot_pose_map` 消费者（20 Hz 普查），在**其消息所标注的测量时间戳**上对 GT 评分；节点发布解析变换自身的 header stamp | **0.0191 m**（n=1320） | **PASS（用户判据）** `user_requirement_met = true` |
| **当前时钟误差（默认模式）** | 发布位姿与 GT 在**消费者到达时钟**比较 | **0.1625 m**（n=1320），位姿龄期中位数 **0.219 s** / p95 0.319 s | **未达成** |
| **当前时钟误差（opt-in `current_pose_mode:=true` 严格模式）** | 门答案接收时派生新鲜度（1.0 s 到期）+ 全链组合，按 odometry 自身 stamp 发布，不外推 | **消息标注戳 0.0223 m（n=1082）MET**；**当前时钟 0.0591 m（n=1082）= FAIL（>5 cm）**；消息标注戳龄期中位数 **0.093 s**；普查 **1013/1459 有输出 = 69.4 %**（446 无输出） | **实现评审通过（无 mustfix）；当前时钟要求未达成** |
| 修复前（默认，`stamp = now()`） | 消息标注戳 ATE | **0.1263 m**（n=1320） | FAIL（修复所消除的缺陷） |
| **当前时钟误差（`consumer_mode:=predict`，有界预测；2026-09-30 recovery 修订）** | 两帧 odom 短窗外推（≤ `max_prediction_s` 0.12 s），位姿 stamp 即 query 时钟；预测面 fail-closed（帧/时序/门/龄期任一不满足即拒绝） | **当前时钟 = 消息标注戳 0.0203 m（n=1064）= PASS**；后锁 20 Hz 及时输出 **0.9953（PASS）**；**全段 0.8858 = 未达标**（启动 6.62 s 计入分母）；姿态误差中位 0.0066 rad（p95 0.024）；`prediction_dt` 中位 0.103 / p95 0.1173 / 最大 0.1175 s；拒绝 `odom_not_ready 108 / no_correction 21 / gate_not_valid 3 / prediction_dt_out_of_range 3 / odom_interval_out_of_range 2` | **用户当前时钟 5 cm：本合成场景 MET**；全段 95%（含启动）**未达标**；非硬件、单种子 |
| **当前时钟误差（`consumer_mode:=current`；2026-09-30 recovery 修订）** | 严格 current 模式（同 §5.2 语义） | **0.0482 m（n=1075）**；消息标注戳 **0.0180 m**；龄期中位 0.0897 s | 与早前轮 0.0591 m **不可直接比较**（源码修订 + tick 普查不同）；两者并存，**旧值不被覆盖** |
| 查询时钟代理（surrogate，诊断） | 持旧 `T_map_odom` ∘ 最新 odometry，按 `/clock` 重戳 | 0.0344 m（默认）/ 0.0355 m（opt-in） | **仅诊断，非验收判据** |
| 延迟时间戳关联 ATE（诊断） | 接收因果组合流，按 odometry stamp 评分 | 0.0202 m（默认，n=534）/ 0.0224 m（opt-in，n=530） | 诊断 |
| 跨路由留出（独立先验） | 不同路由/先验，非消费者测量 | **0.0414 m** @ 10.0 Hz（n=832） | 诊断/留出 |

> 权威目录：上表中**标有「2026-09-30 recovery 修订」的两行**为同日新证据（`artifacts/software_metrics_recovery/step4_runtime/`，含机器索引与逐文件哈希），其余各行仍为 2026-09-30 早前轮（`artifacts/simulation_20260930/t3_availability_intake_fix/`）。两套数字并存，互不覆盖。

### 5.2 opt-in 模式的关键事实（[实测]）

- **`current_pose_mode` 默认 `false`**；opt-in 严格模式的`published-stamp ATE 0.0223 m` 两轮均 MET；**当前时钟误差最终轮 0.0591 m FAIL**。
- **早期一轮同路由读数 0.0186 / 0.0457 m 已被取代**：该轮**早于两处评审 P1 修复**，**不得**作为通过口径引用。
- **普查 1013/1459 = 69.4 %**（修复前对比 1227/1461 = 84.0 %）；严格门对余下 446 tick 合法地"无答案"。
- **取消测试**：wall 60 s 杀死 localizer，最后一次发布于 59.61 s，随后状态 `gate_valid=false`（答案龄期 1.0001 s），**之后零位姿发布**（[`consumer_status.jsonl`](../../artifacts/simulation_20260930/t3_availability_intake_fix/current_pose_mode_cancellation/consumer_status.jsonl)）。
- **不外推**：该模式从不查询未来 TF，度量确实落在/越过边界，不呈现为"通过"或"临界"。
- **2026-09-30 recovery 修订（`consumer_mode` 三档，同一 bag/同一冻结 prior，各自独立 run-dir）**：`stamp` 消息标注戳 0.0181 m / 当前时钟 0.1878 m（n=1076）；`current` 0.0180 / 0.0482 m（n=1075）；`predict` 0.0203 / 0.0203 m（n=1064）。
- **`predict` 独立 verdict（`predicted_service`）**：PASS —— 当前时钟 ATE 0.0203 m ≤ 0.05（n=1064）+ 后锁 20 Hz 及时输出 0.9953 ≥ 0.95（1062/1067 格）；同时**如实保留**全段 0.8858、启动 6.62 s、姿态中位 0.0066 rad、`prediction_dt` 中位 0.103 s、拒绝计数；**未被**当作旧协议的通过口径。
- **旧 `0.1 s`/95% 协议**：三模式 FULL/post-lock 仍 0.0000 → **仍 FAIL**，不因新指标 PASS 改写（§5.3）。
- **四种故障（真实执行、真实证据）**：停 localizer、停 odom、暂停 clock（SIGSTOP player 5 s，窗口内零 status 零位姿）、回跳 clock（2 个 epoch，回跳后未在输入重新就绪前发布任何位置姿）——均在门限过期后**零位姿发布**。
- **真实服务两码一致**：303 次真实 `/localizer/relocalize_check` 调用（code 0 与 code 1 各半）：无锁 false → 操作员 relocalize 后 true（操作员路径直接采纳，自主连续资格由单元测试与启动期覆盖）→ 冻结 cloud/odom 输入 ~1 s（`validity_timeout_s`）后 false。旧 `code == 1` 无条件旁路已不存在。
- 证据与哈希：`artifacts/software_metrics_recovery/step4_runtime/`（`step4_report.md`、机器索引 `step4_index.json`、修订 `step4_attestation.json`）。

### 5.3 `0.1 s` 龄期审计 vs `1.0 s` opt-in 门策略（两种不同、分别测量的策略）

| 量 | 定义 | 实测 | 含义 |
|---|---|---|---|
| **龄期门控可用性**（协议 `hold_validity_s = 0.1`） | 已到达修正且 `query_stamp − tf_message_stamp ≤ 0.10 s`，且有效性答案已到达 | **0.0000** 全段 / **0.0000** 锁后（60001 查询：`no_odom_frame` 5200、`no_tf_received_before` 1200、`too_old` 53401、`no_valid_answer` 0、`gate_not_valid` 200） | 声明的 0.1 s 新鲜度契约**在任何可达输出速率下不可行** |
| **因果可用（无龄期界，诊断）** | 已到达修正，龄期无界 | **0.8900** 全段 / **1.0000** 锁后 | 流可用；失败的是**契约**，非物理 |
| **修正龄期** | 已接受 `map←odom` 修正的龄期 | 中位数 **0.261 s** / p95 0.377 s（n=53601） | 物理无害，但使消费者共时位姿在 0.5 m/s 下陈旧约 12 cm |
| **opt-in 门/里程计 1.0 s 策略** | 独立、分别测量的策略 | 见 §5.2 | **不是** 0.1 s 连续性契约的修复 |

- **最小可达龄期 ≈ 0.2 s**（refine ICP 8 迭代 ~68 ms + 传输 + 一个 0.1 s 帧周期）⇒ 龄期门控可用性**按构造为 0**。**未放宽任何门**。
- **启动期单独报告**（GT 前 5.2 s 无 odometry frame；首个定位位姿再 +1.6 s），保留在主分母：即使完美定位器，全段覆盖也 ≤ ~0.91。
- **覆盖率 ≠ 关联率**：`association_rate_est = 1.0`；旧"293/293 = 100 %"**已撤回**（§12）。
- **不混用定义**：T2 的 0.1 s（帧周期时延）与 T3 的 0.1 s（修正龄期）是两个不同量；`median_est_period_s = 0.1000 s` 是采样栅格假象，真实测量是 `distinct_tf_stamp_rate_hz`。

---

## 6. C1 受控干预（仅内部指标，非 GT 验证）

- **性质**：在冻结的 `replay_2` artifact 上做的**受控离线干预**（非新一轮回放，非在线 A/B）；用生产门配置重跑注册级联，强制一个候选（`kf522 ← kf6`）以测量 `correction` 门单独的作用。
- **内部回访指标 [实测]**：baseline **0 接受**，回访 NN 中位数 **6.196 m**（p90 10.087 m，≤0.1 m 比例 0.13 %）→ candidate **1 接受**，NN 中位数 **0.153 m**（p90 4.572 m，≤0.1 m 41.1 %、≤0.5 m 67.5 %）。位姿位移：kf522 = 10.179 m，kf6 = 0.000 m，RMS 4.127 m，max 10.218 m。
- **溯源区分 [实测]**：生产日志只记录标量（`rel_t=1.067, corr=10.180, allow 2.80, dyaw=−4.01°, p2pl=0.0263, overlap=0.434`；唯一失败项 `correction`），**不记录 6-DoF 矩阵**，故原始 SE(3) **不**从日志断言。
- **边界**：`full_gt_improvement_claim = false`；回访窗口自洽**不是** GT 验证，无留出帧有效建图精度断言；留出评测维持 `BLOCKED_PENDING_EVALCONTRACT`。
- **生产惰性**：源码改动仅对指定对启用强制钩子 + 可选审计捕获，私有标志默认 `false/0`，`pgo_node` 不调用；`test_gate_math` 6/6、`test_scan_context_yaw` 3/3 通过。
- 证据：[`provenance.json`](../../artifacts/local_loop_intervention/results/provenance.json)；驱动：[`run_intervention.sh`](../../artifacts/local_loop_intervention/run_intervention.sh)。

---

## 7. 全部 NONHARDWARE 剩余任务（按优先级）

> 原则：**不假定未执行项已完成**；已执行但 FAIL 的按 FAIL 记录；外部依赖只交接口契约，**不虚构依赖实现**。

### P0 — 直接阻塞用户"当前时钟 ≤ 5 cm"与连续性
1. **当前时钟定位精度的根因分析 + 有界因果传播契约**。现状：opt-in 严格模式最终 `0.0591 m` FAIL、默认 `0.1625 m`；缺的是**有界、fail-closed 的已接受修正传播契约**（§5.2 已实现 opt-in，但仍越界）。待补：(a) 明确当前时钟误差的根因拆分（门答案接收时延 / 里程计传播 / 组合链）；(b) 明确 `Δt_max` 与 `hold_validity_s_contract`（**重新推导，不继承 0.1 s**）；(c) 证据：20 Hz 查询时钟 ATE、`Δt` 分布、拒绝计数、被传播（而非消息标注戳）服务的查询比例。
2. **连续性契约重述或降时延**。0.1 s 修正龄期**物理上不可行**（最小可达 ~0.2 s）。要么把契约重述为物理有意义值（≥ 0.3 s），要么降低整条管线时延（ICP 重写）。**同时**把"95 %/0.1 s 审计协议"与"opt-in 1.0 s 门/里程计策略"固化为**两种不同、分别测量**的策略，文档中不得混用定义。

### P1 — 证据充分性（统计 / 留出 / 历史 A/B）
3. **重复与留出 + 统计证据（已闭环交付）**。Step 5 真实执行并闭环 80 运行（75 主路径 + 5 留出路径）串行实验（独占域 137，`ROS_LOCALHOST_ONLY=1`，overlay `build_workspace/install/setup.bash`）：baseline 15 运行稳定 PASS（T1 均值 0.02768 m，95% CI [0.02712, 0.02825] m ≤ 0.05 m）；c2_2 15 运行稳定 PASS（0.02398 m，CI [0.02385, 0.02413] m）；c2_3 15 运行稳定 PASS（0.02808 m，CI [0.02736, 0.02877] m，成对差 CI [-0.00051, 0.00146] m 跨 0 未证明改进）；c2_1 与 c2_combined 各 1 次触发评测器 `BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS`（1/212085 与 1/227138 点，未调门/未重试/全链留档）；主路径总体 `stable_meets_target = False`；跨路径留出 5/5 PASS（0.02991 m，CI [0.02836, 0.03145] m，不声称跨场景泛化）。证据见 [`../../artifacts/software_metrics_recovery/step5_repeats/`](../../artifacts/software_metrics_recovery/step5_repeats/)。
4. **历史数据集：归档重评已执行，全新受控 A/B 未执行（预算未授权，前置门禁阻断）**。先区分两件事：**(a) 归档数据的重新评测/更正已执行**——ROI 取代（supersession）强制、`map_accuracy_eval.py` v2.3.1 的 outside-attribution 实测成员几何与 v2 残差、历史报告 3.23x/50–90x 等撤回，证据见 [`../../artifacts/04b73f5_evidence_audit_20260929_5V6fKb/`](../../artifacts/04b73f5_evidence_audit_20260929_5V6fKb/)（评测器 58 用例通过）；**(b) 全新受控 A/B 回放未执行**（本轮 0 次回放）。前置门禁逐项**全部 BLOCKED**（见 [`step7_historical_admission_report.md`](../../artifacts/software_metrics_recovery/step7_historical_admission_report.md) 与 [`step7_data_admission_gates.json`](../../artifacts/software_metrics_recovery/step7_data_admission_gates.json)）：
   - **权威预定义 ROI（独立于估计/残差）**：已更正为 `roi_refframe_corrected_mcd_tuhh_night_09.json`，虽标权威但下游多臂前置未齐。
   - **独立来源与严格空间不相交门**：**BLOCKED**。各对比臂必须具备独立来源的 $T_{target\_source}$，且拟合区与评测区必须严格满足空间不相交（$\text{AABB}_{fit} \cap \text{AABB}_{eval} = \emptyset$）。现有 `transform_independent_source.json` 仅在 `mcd_tuhh_night_09-fork-smoke` 估计轨迹上拟合（不可跨臂复用）；经 `make_transform_independent_source.py:288-292` 实测 KDTree 计算，1332 个评测样本中有恰好 100 个点（7.51%）落在拟合区 1.0 m 邻域内，空间不相交未闭合。上游与 C2 各臂完全缺失属于自己的独立变换。
   - **R38 域外点成员几何归因门**：**BLOCKED**。权威纠正 ROI 下 Fork 残余 57,335 点（占图外点 24.31%）、上游残余 1,588,007 点（占图外点 90.63%）未被声明成员几何覆盖；诊断 ROI 下亦有 67,717 / 1,542,589 点未归因。R38 严格 fail-closed 维持 `pass=null`，契约严禁通过在估计图上做 ICP 拟合或随意扩大 ROI 来强行买 PASS。
   - **独立坐标与外参资格核验**：MCD 官方 UserManual 逐字明示 TUHH Body 系直接重合于 VN-200 IMU（$T_{body \to vn200} = \mathbf{I}_{4 \times 4}$，见 `datasets/raw/mcd/hhs_calib.yaml`）；旧“mocap body→IMU 外参缺失”系对 TIERS/IndoorOffice 动捕缺陷的机械套用，对 TUHH 不适用。但 Body≠LiDAR（存在杆臂与 180° 翻转），且 MCD GT 为测绘底图注册派生，非完全独立于底图。
   - **产物口径不可比**：**BLOCKED**。上游 `PCD/scans.pcd`（8.9M 原始扫描直接累积）与 fork `map.pcd`（2.3M PGO 关键帧闭环优化图）口径不可比；公平对比必须基于统一离线 0.05 m 统计体素累积去畸变 world scan（上游 `/cloud_registered` vs fork `world_cloud`）。
   - **运动工况高动态压力边界**：TUHH night_09 实测滑窗均值 1.560 m/s、中位 1.714 m/s，以 179 个有效 1.0s 滑窗为分母，超速（>1.0 m/s）达 89.94%（161.0s），低速/停顿（<0.5 m/s）占 8.38%（15.0s，含静止 11.0s），常规包线（0.5–1.0 m/s）仅占 1.68%（3.0s），属于典型高动态压力测试，严禁降速播放伪装。
   - **可达软件配置基础设施（已完成）**：父目录 `eval/tools/gen_config.py` 与 `eval/run_slam.sh` 已实现 `--extra-config PATH`，严格白名单过滤 C2 消融键集合（`imu_acc_normalize`, `imu_init_mode`, `imu_init_min_samples`, `pcl2_filter_phase`），未知键在 CLI 与 runner 均以 exit 1 立即阻断，生成并落盘 `effective_config.json`，安装默认配置零修改；详见 `eval_extra_config_verification.json/.log`。
   - **本轮授权与实际执行：0 次完整回放**（前置未齐，严禁授权启动伪实验卡；BLOCKED 属门禁阻断，不是历史精度 FAIL）。
### P2 — 代码正确性（已知缺陷，未修）
5. `relocalize_check` 仍把 `request->code == 1` **无条件**返回 `valid=true`；该强制路径应带同样的新鲜度界或移除。
6. **首次自锁**在操作员 `relocalize` 之前报 invalid（门仅在服务请求或 lost+recovery 后置 valid）——对 harness 可接受，对自主部署**错误**。

### P3 — 评测器残余未知（按证据判定为未解决）
7. **R14**：历史定位证据 `metrics.json` 的 `ate_over_gt_samples_denominator` **字段名与实际计算不一致**（复用 293 估计戳误差），引用该字段须标 `[条件]`，**按 reviewer R14 以撤回结案**：该字段名已被机读撤回（`metrics.retracted_fields`），**正确名称为 `ate_over_est_subset_rmse_m`**；现行字节证据与命令/版本索引见 [`../../artifacts/software_metrics_recovery/evaluator_step6/`](../../artifacts/software_metrics_recovery/evaluator_step6/)（报告 `evaluator_step6_report.md`、映射 `evaluator_r01_r38_current_byte_mapping.json`、复核 `review/`）。详见 [`04b73f5_localizer_regression.md`](04b73f5_localizer_regression.md) §九。
8. **R01–R07 / R12**：`CLOSED_PENDING_FORMAL_VERIFICATION`（尚未正式验证）；**R38**：`CLOSED_BY_OWNER_SELF_CHECK_PENDING_FORMAL_VERIFICATION`（属历史数据集审计侧）。
9. `evaluator_verification.json`：`formal_status = PASSED_WITH_DEVIATION_NOTED`，V2 两臂 `pass=null`、上游状态偏差已记录。

### P4 — 外部接口契约（只定义，不实现依赖）
10. **T1.2 传感器抽象（Airy/Odin1，非 Livox 产品）— 软件侧剩余项**：**软件交付已完成**（参数化配置 `lio_orin_nx.yaml` + 通用 `PointCloud2`/`Imu` 话题适配）；仓库内**无** Airy/Odin1 驱动/标定代码。软件侧唯一剩余是**纪律约束**：在板端 P1–P5 确认前，**不得**给出 `pcl2_time_field/scale`、`imu_acc_scale` 等"推荐值"，也不得把 `lio_orin_nx.yaml` 当可用 profile（`ext_il` 缺键 ⇒ `WARN` + 恒等占位，见 §11.3）。板端 P1–P7 验证属**硬件范围**（§13），不在本剩余清单内。
11. **T1.4 足式状态估计依赖**：这是**具名外部输入/依赖，不是要在 SLAM 侧虚构足式融合模型**。本仓库**只**提供该输入必须满足的**接口契约**（Q1–Q5：带时间戳的机体系线速度 + 协方差、其独立精度/时延/打滑资格、IMU/LiDAR 时钟域对齐、零速/触地处理、机体系外参）。**输入本身不存在且未资格评估** ⇒ 这是**依赖缺口**，不是 SLAM 交付遗漏；**不得**在此虚构观测模型。

### P5 — 文档口径
12. **FastLIVO2 接口文档 vs 实际集成范围**：接口文档是**权威的 topic/service/TF/param 表**；而**仓库实际集成 = LiDAR-惯性里程计 + 点云着色**，状态 21-D 无相机量，图像通道仅用于投影着色，**无**视觉/光度/融合权重 API（`CVUtils::weightPixel` 是零调用点的死代码）。上游是 `hku-mars/FAST-LIVO2`（与 `hku-mars/FAST_LIO` 不同代码库）；本 fork **不对应**视觉紧耦合分支。交接时**不得**把接口契约误读为"已实现视觉紧耦合"。

---

## 8. 复现命令 / 依赖 / 输出目录

> 所有命令**已对照当前磁盘脚本核对**（`--help` / 静态读参），**不**要求重跑整套。以下为**已验证存在的选项**；完整清单见验收报告 §13。

### 8.1 环境与构建依赖
- ROS 2 **Humble**（`/opt/ros/humble/setup.bash`）；`colcon`；PCL；Eigen；**GTSAM 4.2**；`rosbag2_py`；numpy；Python 3。
- 构建（**不要与其它 agent 的构建并发**）：
  ```bash
  source /opt/ros/humble/setup.bash
  MAKEFLAGS=-j3 colcon build --parallel-workers 3 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
  source install/setup.bash
  ```
- **domain 隔离**：每个 ROS 运行**独占** `ROS_DOMAIN_ID` 且 `ROS_LOCALHOST_ONLY=1`（同 domain 的两 agent 会**静默**交叉污染）。运行前检查 domain 为空。

### 8.2 无 ROS（纯生成/自检）
```bash
# 生成合成 bag（同时写场景参考）；--check 在任何模型不连续处 fail-closed
python3 simulation/synthetic_data/generate_test_bag.py --out /tmp/sim_bag
python3 simulation/synthetic_data/generate_test_bag.py --check
python3 simulation/synthetic_data/generate_test_bag.py --check \
        --trajectory simulation/synthetic_data/test_trajectory_localization.json
```
可用选项：`--out --trajectory --duration --check --clean-start --seed --min-range --max-range --no-overwrite`。

### 8.3 一键管线 / 单次运行（有界墙钟、PID 限定清理、manifest+哈希）
```bash
# 全合成管线（重生成 bag+参考+T1/T2/T3）；variants 顺序执行
SIM_DOMAIN_ID=137 bash simulation/scripts/run_eval_pipeline.sh --out-root /tmp/sim_fixed3 \
     --variants baseline --with-trajectory-diagnostic --domain-id 137
# 选项：--out-root --ref-dir --fit-until-s --rate --domain-id --variants
#       --skip-mapping --skip-localization --with-trajectory-diagnostic
#       --duration --loc-duration --t3-coverage-window --dry-run

bash simulation/scripts/run_mapping_sim.sh --run-dir /tmp/sim_map \
     [--extra-config simulation/ablations/c2_1_acc_normalize.yaml]
# 选项：--run-dir --work-dir --bag-dir --trajectory --duration --rate
#       --extra-config --fit-until-s --domain-id --start-delay --skip-bag --launch-timeout-s

bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc \
     --prior-map /tmp/sim_fixed3/baseline/mapping/frozen_map.pcd
# 选项：--run-dir --work-dir --bag-dir --trajectory --prior-map --duration --rate
#       --domain-id --start-delay --skip-bag --launch-timeout-s
```

### 8.4 指标测试（退出码 0 PASS / 1 FAIL / 2 BLOCKED）
```bash
python3 simulation/scripts/test_t1_accuracy.py  --run-dir <run> --ref-dir <ref>
python3 simulation/scripts/test_t2_speed.py     --run-dir <run>
python3 simulation/scripts/test_t3_localization.py --run-dir <run> --ref-dir <ref>
bash simulation/scripts/run_all_tests.sh   # 用 SIM_RUN_DIR / SIM_REF_DIR / SIM_LOC_RUN_DIR 环境变量

# bag 时间契约（T2 直接测量的前置）
ROS_DOMAIN_ID=99 python3 simulation/scripts/check_bag_availability.py --bag <bag> --out /tmp/bag_availability.json
```
`test_t1_accuracy.py` 选项：`--run-dir --ref-dir --est-map --ref-map --roi --transform-json --outside-attribution --stat-voxel --sequence --run-name --eval-dir --out --threshold-m`。
`test_t2_speed.py` 选项：`--run-dir --evidence --record --trajectory --duration --threshold-s --target-mps --max-mps --max-delivery-loss --min-rtf --out`。
`test_t3_localization.py` 选项：`--run-dir --ref-dir --est-tum --gt-tum --record --manifest --frame --entity --max-gap-s --coverage-tolerance-s --min-coverage --coverage-window {full_gt_span,common} --availability-file --min-availability --eval-dir --work-dir --out --threshold-m`。

### 8.5 就绪启动（当前源码，无硬件）
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
ros2 launch simulation/launch/sim_fastlio2.launch.py bag_path:=/tmp/sim_bag \
     work_dir:=/tmp/sim_map rviz:=false
ros2 launch simulation/launch/sim_localization.launch.py bag_path:=/tmp/sim_bag \
     work_dir:=/tmp/sim_loc prior_map:=/tmp/sim_ref/baseline/mapping/frozen_map.pcd \
     localizer_config:=/tmp/sim_loc/loc_sim.yaml consumer_mode:=predict
# consumer_mode:=stamp|current|predict（默认 stamp）；launch 会启动**恰好一个**真实消费者
# robot_pose/map_pose_publisher，并加入仿真专用恒等静态 TF body→base_link —— 不要再手动起第二个。
python3 simulation/scripts/record_consumer.py --out-dir /tmp/sim_loc --duration 90 --pose-mode predict
# --pose-mode 必须与 launch 的 consumer_mode 相同；runner 会先起录制器、等 consumer_ready.json，
# 再启动 ros2 bag play（录制先于回放）。
```
launch 参数：`sim_fastlio2.launch.py` = `bag_path work_dir extra_config config_file rate start_delay rviz log_level play_bag`；`sim_localization.launch.py` = `bag_path work_dir config_file extra_config rate start_delay localizer_config prior_map log_level consumer_mode play_bag shutdown_gate shutdown_gate_timeout_s`。四个 runner 均支持 `SIM_INSTALL_SETUP=<path>/install/setup.bash` 选择被叠加的构建（默认本仓库 `install/setup.bash`），避免旧 install 静默覆盖。

### 8.6 证据持久化 / 报告
```bash
bash simulation/scripts/persist_evidence.sh --out-root /tmp/sim_pipeline --dest fork/artifacts/simulation_<utc>
bash simulation/scripts/finalize_runs.sh    --out-root /tmp/sim_pipeline --dest fork/artifacts/simulation_final
python3 simulation/scripts/make_report.py --out-root /tmp/sim_fixed3 --out /tmp/report_check.md   # 选项 --out-root --out --evidence-dir
bash artifacts/local_loop_intervention/run_intervention.sh both --save-map   # 用法 <baseline|candidate|both|verify> [--save-map]
```

### 8.7 单元测试 / 诊断
```bash
colcon test --packages-select localizer
colcon test --packages-select robot_pose
colcon test -R "test_imu_init|test_utils_preprocess"
# 评测器测试（冻结）：
cd artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation && python3 -m pytest -q .
# 诊断（只读，见 simulation/diagnostics/README.md）：
python3 simulation/diagnostics/c2_3_sampling_phase.py
python3 simulation/diagnostics/residual_attribution.py <run-dir>/mapping <ref-dir>
```

### 8.8 进程清理纪律（重要）
- **禁止广域 kill**（`pkill -f lio`、`killall`、`ros2 node kill /`、按 `ROS_DOMAIN_ID` 无关的批量清理）——会误杀并发 agent 的进程。
- 只用运行脚本内置的 **PID 限定、有界墙钟**清理；手工排查用 `ps -o pid,ppid,cmd -C <exe>` 精确定位后再按 PID 处理。

### 8.9 输出目录约定
- 运行根（临时）：`/tmp/sim_fixed3`（T1/T2 基线）、`/tmp/sim_abl_final`（C2 五臂）、`/tmp/loc_diag/*`（T3 消费者）。
- 持久证据：`artifacts/mapping_corrected_20260930/`、`artifacts/simulation_20260930/t3_availability_intake_fix/`、`artifacts/local_loop_intervention/results/`。
- **大 bag（>500 MB）按设计排除**，其哈希写入各 `run_manifest.json`。

### 8.10 历史配置生成与离线驱动（父目录 `eval/`）
```bash
# 生成 sequence 专属生效配置并支持 C2 消融覆盖（严格白名单校验）
python3 eval/tools/gen_config.py --sequence tiers_indoor_office1_mid360 \
     --manifest datasets/MANIFEST.json --ws fork \
     --out-dir /tmp/wp2_extra_cfg_test/out_c2_1 \
     --extra-config fork/simulation/ablations/c2_1_acc_normalize.yaml
# 产物落盘：<out_dir>/config/effective_config.json, effective_config.yaml, lio.yaml, resolved.json

# SLAM 回放驱动透传（未知键立即以 exit 1 阻断退出）：
bash eval/run_slam.sh --sequence tiers_indoor_office1_mid360 \
     --extra-config fork/simulation/ablations/c2_1_acc_normalize.yaml
```
`--extra-config` 选项：仅允许 C2 白名单键（`imu_acc_normalize`, `imu_init_mode`, `imu_init_min_samples`, `pcl2_filter_phase`），非法键 fail-closed 阻断。

---

## 9. 代码与报告索引、证据存储、git 状态

### 9.1 报告索引（相对本文件的链接）
| 文档 | 路径 / 状态说明 |
|---|---|
| 详细计划要求与问题解决规范 | [`../detailed_plan_requirements.md`](../detailed_plan_requirements.md)（核心规范：7项真实验收、指标瓶颈、历史数据准入、Lint与阻断解决） |
| 验收报告（最终口径） | [`local_simulation_acceptance.md`](local_simulation_acceptance.md) |
| FastLIVO2 接口 | [`04b73f5_fastlivo2_interface.md`](04b73f5_fastlivo2_interface.md) |
| 参数单位/加载优先级 | [`04b73f5_param_notes.md`](04b73f5_param_notes.md) |
| 部署/T1.2/T1.4 契约 | [`04b73f5_p3_deployment.md`](04b73f5_p3_deployment.md) |
| 定位器回归/门控 | [`04b73f5_localizer_regression.md`](04b73f5_localizer_regression.md) |
| 历史建图诊断（旧稿） | [`04b73f5_mapping_accuracy_diagnosis.md`](04b73f5_mapping_accuracy_diagnosis.md) |
| 历史证据 manifest（本地历史产物） | `artifacts/nonhardware_delivery/manifest.json`（本地历史证据，不随仓提交） |
| 建图诊断报告（本地历史产物） | `.omp/reports/mapping_accuracy_diagnosis.md`（本地历史证据，不随仓提交） |
| 定位覆盖诊断（本地历史产物） | `.omp/reports/localization_coverage_diagnosis.md`（本地历史证据，不随仓提交） |
| 仿真集成报告（本地历史产物） | `.omp/reports/simulation_integration.md`（本地历史证据，不随仓提交） |
| 历史 A/B 更正（本地历史产物） | `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/reports/...`（本地历史证据） |
| 下一步实验卡（本地历史产物） | `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/reports/...`（本地历史证据） |

### 9.2 证据存储（不可变语义）
- **逐文件 sha256** 记录在 [`manifest.json`](../../artifacts/nonhardware_delivery/manifest.json)；大型证据根另有仓库惯例的 `EVIDENCE_INDEX.json`（列出每个持久文件）；**bag 按设计排除**并在各 `run_manifest.json` 内哈希。验收报告的 sha256 **单向绑定**在 manifest 中（报告本身不自带哈希）。
- **冻结声明**：源码冻结后，manifest 中每个 sha256 均在冻结树上重算并与磁盘一致。
- **追溯性保留**：被取代的目录（如 `final_acceptance/` 查询时钟代理、`final_coherent_fixed3/` 仅延迟、`real_consumer_acceptance/` 修复前消费者、`artifacts/simulation_20260930/EVIDENCE_INDEX.json` 前缀集）**保留但不得作为判据引用**。
- **注意**：这些证据目录在 git 中是**未跟踪**的（§9.3），"不可变"指**磁盘 + sha256 记录**，不是 git 提交不可变。

### 9.3 当前 git 基线与脏改动（**不要假装已提交**）
- HEAD = `04b73f553918b1dcc751feac69c0a2834f194432`（`main`），提交时间 `2026-09-29 12:11 +0800`（"docs(readme): state the rejected-candidate publication policy"）；仓库共 30 个提交。
- **工作树脏**：24 个已跟踪文件被修改（`git diff --stat`：993 insertions / 167 deletions），15 条未跟踪项：
  - 未跟踪目录：`.omp/`、`artifacts/`、`docs/`、`simulation/`（**全部证据与报告都在未跟踪目录里**）。
  - 未跟踪源码/配置：`src/localization/localizer/src/localizers/{lock_validity.h,pair_cache.h}`、`.../test/{test_lock_validity.cpp,test_pair_cache.cpp}`、`src/localization/robot_pose/{src/map_pose_current.hpp,src/map_pose_stamp.hpp,test/}`、`src/sensing/fastlio2/config/{TIME_SYNC_NOTES.md,lio_c2_experimental.yaml,lio_orin_nx.yaml}`、`src/sensing/fastlio2/test/test_utils_preprocess.cpp`。
- **交接要求**：接手前先 `git status` / `git diff` 自查；**不要**把上述修复当成"已合入 main"。是否提交由协调者决定；本任务**未**提交任何改动。

---

## 10. 测试范围（对照报告，区分历史失败/lint）

> **绝不宣称整套 `colcon test` 全绿**：聚合 `colcon test-result` 仍报**先前存在**的 lint/style 失败与一个陈旧（20260928）`Test.xml`。

| 套件 | 用例数 | 命令 | 状态 |
|---|---|---|---|
| 评测器/契约（`evaluation/test_*.py`） | **58 passed / 0 failed**（map 34、time 12、parity 12） | 在 `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation/` 下 `python3 -m pytest -q .` | `[实测]` |
| `fastlio2` `test_imu_init` | **7** | `colcon test --packages-select fastlio2 -R test_imu_init` | `[实测]`（源码计数） |
| `fastlio2` `test_utils_preprocess` | **11** | `colcon test --packages-select fastlio2 -R test_utils_preprocess` | `[实测]`（源码计数） |
| `localizer` | **24**（gate 12 + pair_cache 7 + lock_validity 5） | `colcon test --packages-select localizer` | `[实测]`；P2a 前为 22 |
| `robot_pose` | **8**（stamp 1 + current 7） | `colcon test --packages-select robot_pose` | `[实测]`（源码计数） |

- **小计**：58 + 7 + 11 + 32 = **108** 个用例（32 = localizer 24 + robot_pose 8）。
- **历史 4 目标回归集**：25/25（`test_imu_init` 6、`test_gate_math` 6、`test_localizer_gate` 10、`test_scan_context_yaw` 3），来自 `.gtest.xml`，**属历史**（计数与当前源码不同）。
- **历史失败/lint**：先前存在的 lint/style 失败与陈旧 `Test.xml` **不得**被当作本轮回归；也不得被本轮修复宣称为"已修好"。
- 评测器正式验证：`formal_status = PASSED_WITH_DEVIATION_NOTED`（V1 49 pytest、V3 cross-check strict、V4 两 CLI 测试通过；V2 两臂 `pass=null`、上游状态偏差已记录）。

---

## 11. 交接清单、DoD、风险与回滚

### 11.1 接手首日清单
1. `git status` + `git diff`（§9.3）：确认工作树脏、未提交；核对基线 HEAD。
2. 读 [验收报告](local_simulation_acceptance.md) §0（撤回声明）与 §1（需求 vs 附加协议），再读 [manifest.json](../../artifacts/nonhardware_delivery/manifest.json)。
3. 校验证据哈希：按 manifest 逐个 `sha256sum`（**不必**重跑）。
4. 从 §8.2 的 `generate_test_bag.py --check` 与 §8.3 的 `--dry-run` 起步，确认环境可用（**不要**先跑整套）。
5. 选一个 P0 任务（§7）开工；**任何新回放/新实验前先确认授权**。

### 11.2 每个软件任务的 DoD（Definition of Done）
- 明确**用户需求 vs 附加协议**归属；不得把附加协议失败写成用户需求失败（反之亦然）。
- 变更**落地到磁盘**并**自验证**（针对性测试/复现，不跑全量）；数字带**run root** 与**口径名**。
- 证据进入 `artifacts/<root>/` 并**带 sha256**（`EVIDENCE_INDEX.json` 或 manifest）；被取代的旧文件**保留**并标注 SUPERSEDED。
- 涉及数字的口径**先冻结**（阈值/门/容差**不得**在看到分数后放宽）；文档同步更新验收报告对应节。
- 不新增**脚手架/占位/伪回退**；外部依赖**只交接口契约**。

### 11.3 风险、默认值与回滚
- **实验默认值（生产安全）**：C2 开关**不在** `lio.yaml`（legacy 默认），仅在 `lio_c2_experimental.yaml`；`current_pose_mode` 默认 `false`；`pcl2_filter_phase` 默认 `0`（= 上游行为）。
- **外参风险（部署阻断）**：`config/lio_orin_nx.yaml` **有意省略** `ext_il`；缺键仅 `WARN` 并退回 `r_il = I, t_il = 0` ⇒ **静默生成系统性错误的图**。标定**明确不可用且未虚构**；**部署 profile 不得原样运行**。`lio.yaml` 的 `ext_il` 是仓库默认机器人值；仿真恒等外参**仅限仿真**。
- **仿真/硬件隔离**：仿真恒等 `ext_il` 必须在硬件上弃用。
- **回滚**：已跟踪文件的改动用 `git diff` 审阅后按需 `git checkout -- <path>`；**未跟踪**的新文件/目录（含 `docs/`、`simulation/`、`artifacts/`、`.omp/`）不受 `git checkout` 影响，需人工判定保留。**不要**运行破坏性 git 命令。
- **TF 风险**：`map→odom` 两生产者互斥；同 domain 同时启动会被静默覆盖。

---

## 12. 已撤回 / 作废的说法（不得再引用）

| # | 旧说法 | 现状（以当前文件为准） |
|---|---|---|
| W-1 | "fork 精度领先 **3.23 倍**" | **撤回**：1.0728 m（fork）与 3.4614 m（upstream）用的是**同一个在 fork 上拟合的变换**套到两个不同的 `est_map_frame`；`BLOCKED/pass=null` 不是 PASS。 |
| W-2 | "比官方好 **50–90 倍**" | **撤回**：配置/产物类型/参考系三重不可比（量程 30 m vs 无上限、PGO 图 vs 扫描累加云、世界系未证相同）⇒ 判 **BLOCKED**，不比较倍数。 |
| W-3 | "漂移/精度**根因已证**" | **撤回**：仅两个 **harness** 缺陷被直接测量并修复（缝合取模插值、twist 渲染器）与一个记录解析缺陷；**数据集级**原因（`correction` 门为"决定性瓶颈"、前端漂移根因、世界系不匹配）仍是**假设 / BLOCKED**（单序列、无消融）。**不得**声称"已证明 IMU 漂移根因"。 |
| W-4 | "已完成硬件标定/有 Airy 标定值" | **撤回**：**无**任何硬件标定。`lio_orin_nx.yaml` 曾写入的 `ext_il=[0.020,0,0.037,1,0,0,0]` 是从 MCD 回放配置抄来的**臆造外参**，**已删除**；无 Airy/Odin1 标定、无 IMU 单位确认、无时间同步测量。 |
| W-5 | "替代（surrogate）在线 ATE 证明了在线 5 cm 需求" | **撤回**：尝试 1（延迟组合 0.0224 m）、尝试 2（`/clock` 重戳 0.0260 m）均被评审否决；判据已移到**真实** `/robot_pose_map`。§5 表中的 surrogate 数值**仅诊断**。 |
| W-6 | "**0.0457 m** 是在线/当前时钟的通过结果" | **撤回/取代**：该轮**早于两处评审 P1 修复**；最终轮为 **0.0591 m FAIL**（消息标注戳 0.0223 m MET）。**不得**把 0.0457 当作通过。 |
| W-7 | "1.24 cm 诊断证明 ICP 具备厘米级能力" | **撤回**：1.24 cm 是里程计在**冻结** `map←odom` 下的刚体像；冻结变换不是定位结果。 |
| W-8 | "293/293 = 100 % 定位覆盖" | **撤回**：跨度比、样本关联率、逐秒覆盖是三个不同量，分别报告。 |
| W-9 | "T2 交付比 846/894 = 0.9988" | **撤回**：该行混淆两个分母；比值是 846/847 = 0.9988，894 是含首帧前 47 帧的整条流（JSON 一直自洽，仅文字错误）。 |
| W-10 | "对上游 FAST-LIO 做过受控 A/B" / "共享 fork 变换可跨栈比较" | **撤回**：归档的 `04b73f5_controlled_ab_20260929` 非新一轮受控 A/B（世界系/配置/时窗/产物类型均不同）；fork 派生变换套到其它栈**无效**。 |
| W-11 | "localizer 每测量帧发一个修正样本、分离路由前置被强制" | **撤回（并已修）**：`sendBroadCastTF(frame_time)` 曾在**每次**成功 ICP（含 `accepted == false`）后重戳旧可信修正（P2a）；`test_t3_localization.py` 曾在 manifest 顶层读路由/先验致同路由拒绝静默不执行（P2b）。二者在**当前树已修**。 |
| W-12 | "C2 开关是已验证修复" | **撤回**：opt-in 消融开关；五臂全 PASS 但**无改进被证明**（无显著性检验，差异在 run-to-run 波动内）。 |

---

## 13. 硬件范围边界（仅一行声明，不展开）

硬件 / Orin NX / 标定 / 实机实时、T1.2 P1–P7、T1.4 Q1–Q5、`body→base_link` 全量 bringup 的 TF 唯一性、三源时钟一致性、足式动力学与资源/温度/调度、相机接线与着色——**全部 `【board-only】`，本任务 NOT RUN**；清单见 [manifest.json](../../artifacts/nonhardware_delivery/manifest.json) 的 `hardware_only_remaining_tests` 与验收报告 §13.2。桌面回放**不得**替代整机验收。

---

*文档结束。权威数值与结论以 [`local_simulation_acceptance.md`](local_simulation_acceptance.md) 与 [`../detailed_plan_requirements.md`](../detailed_plan_requirements.md) 为准（本地机器可读历史汇总见 `artifacts/nonhardware_delivery/manifest.json` 与 `final_metrics_summary.json`）。*
