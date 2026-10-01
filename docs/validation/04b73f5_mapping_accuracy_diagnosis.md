# 04b73f5 建图精度（≤5 cm）证据诊断 — 最终交付（v2）

> **本文件取代** 仓库外旧历史输入文档 `docs/validation/04b73f5_mapping_accuracy_diagnosis.md`（历史输入，未入库，保留原文不改）。
> **勘误/取代范围**：旧文档中的全部“根因”结论、倍数对比（93.7×/49.4×/61.8×）、上游延迟/丢帧数字、把 151.19 m 当作“官方精度”、把 `map.json` 当作 04b73f5 验收、把窗口 ATE 当作“局部几何优秀”、以及“删大残差/放宽门限即可改善”的口径，**均不再继承**。
> 旧文件的**原始数值**在本文中作为“历史口径”逐条保留并注明用途限制与证据路径。
> 本轮**验收状态**：建图精度主指标 **BLOCKED（无有效结论）**，不是“未达标”，也不是“达标”。
> 基线：`04b73f553918b1dcc751feac69c0a2834f194432`（main）。本轮新增完整回放 **0** 次（历史 3/3 已耗尽）。

- 产物根（本地历史证据目录）：`artifacts/04b73f5_evidence_audit_20260929_5V6fKb`
- 旧产物（本地只读）：`artifacts/04b73f5_diagnosis_20260929_JWUGeL`
- 证据分级：`[观测]`（产物/源码可直读）· `[推断]` · `[条件]`（依赖未验证假设）· `[BLOCKED]`。**不得把 `[推断]`/`[条件]` 当 `[观测]` 引用。**
- 审查状态：**预审（pre-review）**，正式统一验证 **NOT_RUN**（见 `reports/independent_review.json`）。
- **交付状态：草案**——本文件与 `reports/**`、`final_delivery/**` 将在**协议测试与最终 review** 后由 FinalIntegrator 统一更新（含 hash）。**不要提前视为最终完成。**

---

## 0. 门禁总表（先看这张）

| 门禁 | 状态 | 含义 |
|---|---|---|
| 源 HEAD = 冻结基线 | `PASS_RECORDED` | `git rev-parse HEAD` = 04b73f5…（`freeze/version.json`） |
| 源码 ↔ 二进制绑定 | `SUPPORTED_NOT_PROVEN` | 只有时间序 + 洁净性；Release 无 DWARF、无内嵌 revision，不能密码学绑定 |
| 历史精度可继承 | `BLOCKED` | 1.95/4.79 cm、2.46 cm、HILTI/MCD 旧值不得作为本版本验收 |
| 输入在库/外部输入 | `NOT_CLAIMED` | 参考图、GT、A 图、标定部分在仓库外历史目录 |
| 评测协议实现 | `FAIL_PRE_REVIEW` | reviewer R01–R07/R12 未闭合，未到可 PASS 状态 |
| 评测器正式验证 | `NOT_RUN` | 待协调者稳定后统一授权 |
| **建图精度主指标** | **BLOCKED** | 无可信共同坐标/ROI/变换来源 ⇒ 不能判 PASS/FAIL |
| 后端因果分离（本回放内） | `OBSERVED_ARTIFACTS`（候选级） | 单回放、无消融；“纠偏门杀真回环”为**已证候选**、非必然性 |
| 反事实后端实验 | `NOT_RUN_NOT_AUTHORIZED` | 属另授权后端离线实验，未授权 |

**“文档交付” ≠ “目标已实现”**：本文只说明 5 cm 目标为何当前不可判、可判到什么程度、下一步最小动作是什么。

---

## 1. 必答 8 项

### Q1 撤回哪些历史结论？证据缺口是什么？

按 `audit/historical_claims.md` 与 `reports/independent_review.json`，撤回/降级如下（原证据路径一并列出）：

| 编号 | 旧结论 | 裁定 | 证据缺口 |
|---|---|---|---|
| S6 | 上游“~25 ms / ~48 ms”“丢帧 0/0%” | **撤回**（replay_1 日志 `Time cost` 出现 0 次，数字无来源） | 上游无任何计算侧测量 |
| S4 | “MCD 固定配置 `lidar_max_range` = 300.0 m” | **撤回**（replay_2 归档配置实为 **30.0 m**，`replay_2_fork/config/lio.yaml:8`；300.0 属另一条历史 run） | 无运行期参数 dump ⇒ 量程“实测生效”仍无直接证据 |
| S7/C7 | “293/293 = 全程 100% 覆盖” | **撤回**（293/293 是估计窗口内关联率；60.80/66.20 s 只是**首末跨度比**） | 无独立冻结的按秒 coverage/消费者规则 |
| C1 | “MCD > 1 m/s 导致米级误差（根因）” | **降级为待验证假设**（无低速对照/分层回归；多因素混杂） | 需速度分层受控 A/B |
| C2 | “未世界重力对齐导致官方**必然**发散” | **撤回**（“未对齐”不准确；单次运行 + 参考系未测定 + 无消融） | 上游侧初始化记录缺失、无消融 |
| C3 | “当前比官方好 50~90 倍” | **撤回 → BLOCKED**（配置/产物类型/参考系三重不可比） | 需同栈同配置同参考系受控 A/B |
| C4 | “小窗口 ATE 证明局部几何优秀” | **撤回**（窗口自做自由 SE3 对齐；replay_1 已发散到 191.99 m 其首窗仍 0.118 m） | 无对齐的局部指标/外部局部真值 |
| C5 | “4 cm 门限小于步长，应放宽” | **结论部分撤回**（门控新息 ≠ 物理步长；放宽是改参数） | 真实拒收次数需未节流日志/计数器 |
| C6 | “`map←odom` 冻结 = `map←body` 冻结” | **撤回**（`map←body` 路径 30.39 m，仍在运动） | — |
| C8 | “删大残差降低 RMSE = 改善” | **撤回该口径资格**（截断必须并报原始值/排除率/固定分母） | 属评测口径，非算法改善 |
| 旧 4.3 | “支持域不对称（参考裁 12×12×8，估计 300 m）造成 21.3 m 虚假 max” | **撤回因果**：真正生效量程是取 30.0 m；且支持域需由**参考覆盖**预定义，不能由估计包围盒或残差筛选 | 预定义 ROI（DatasetQualification 负责） |
| 旧 §4 问题1 | “官方未做重力对齐” | **不准确**：上游把重力建模为世界系二维状态 `S2(-mean_acc/|mean_acc|·G)`、初始姿态单位阵；fork 改为重力对齐世界系 + 固定 g。二者是**参数化不同**，不等价也不证优劣 | 上游侧消融/自身初始化量 |

### Q2 已证明问题 vs 假设

**`[观测]` 已证明（本回放内）**
1. `replay_2` 的 `map.pcd`（2,315,569 点）= 524 个关键帧 `patches/*.pcd` 按 `poses.txt` 顺序拼接；重建最大坐标差 5.23e-4 m，落在 6 位打印量化界内 → **地图是里程计位姿拼接，不是体素融合全局图**（`backend_evidence/independent_metrics.json`）。
2. `accepted=0`：全程 77 检测事件、216 候选、55 被评估、**0 接受**；`[PGO][loop]` 行数 = 0 → 位姿图退化为纯里程计，`pgo_keyframes.tum` 与 `lio_odom.tum` 逐帧差 < 1 cm。
3. 唯一“仅因 `correction` 门被拒”的候选即真回环：p2pl 2.6 cm、overlap 0.434、非退化，需修正 **10.180 m > 2.80 m** ⇒ **本回放内该门唯一净效果是杀掉这个真回环**（候选级已证，非必然性）。
4. 重访区双走廊：KF0–10 vs KF514–523 单向 NN 中位 **6.196 m**（对照相邻 KF 约 0.06–0.09 m）→ 米级错位被原样烧进地图。
5. `[观测]` 上游与 fork 的**世界系定义不同**（源码可读）；`seeded` 参考图相对 replay_2 运行时世界系相差约 **12.78°**（含约 2.55° 倾斜，非纯 yaw）。
   ⚠ **两项限制**：① 该角是**相对旋转角而非纯 yaw**；② 不同复算的**方向/基不同**（EvidenceAudit `R0A·R0Bᵀ` 给 12.7798°，FrontendParity 的 `R0.T@R1` 用舍入输入给 12.761/12.505°，见 reviewer **R20**）⇒ **不得**声称“两次独立复算逐位一致”。
   ⚠ **上游侧“≈175.5394° 系差”不得引用**：该值实为 **seed `R0` 自身的转角**（EvidenceAudit v3 确认仅此含义），不等于“上游世界↔参考系旋转”（需 GT body(t0)↔IMU 实体/时刻/杆臂链，均缺）⇒ **真实跨系旋转缺链不可确认**；C2C 比较只能作**敏感性**，**不证明 `seed=fork`**。reviewer **R19** 已撤回其“确证 seed=fork / 已得量级 / ICP 无法跨越”的绝对断言；**静窗 `static=0` 只证明未达静态判据，不单独证明平台运动**（R21）。
6. 25/25 gtest 与 20 包构建有机器可读产物支持（`.gtest.xml`：6+6+10+3；注意旧文档把 `test_localizer_gate` 写成 9/9，以 XML 的 10 为准）。

**`[H]` 假设（未隔离）**：`correction` 门是**决定性瓶颈**（S1–S4 支持，但单序列单回放、O1–O4 反证并存，见 §4）；前端漂移率 3.52%/路径的根因；12.78°/175.54° 与世界系错位与历史地图指标的定量关系。

**`[BLOCKED]`**：本版本建图精度是否达标；坐标链的独立恢复；上游侧参考系差异的**残差构成**。

### Q3 官方 / current 的公平比较条件是否真正满足？——**否**

契约要求同数据、同配置、同参考系、同产物口径。逐项：

| 条件 | replay_1（上游） | replay_2（fork） | 是否可比 |
|---|---|---|---|
| 原始观测 | ROS1 原始双包 | ROS2 转换包 | 基本满足（需 DatasetQualification 背书转换等价） |
| 标定 | 外参相同（t[0.02,0,0.037]+Rx180°） | 同 | 外参相同，**但 IMU 加速度尺度处理不同**（上游 `acc*=9.81/|mean_acc|` vs fork `imu_acc_scale`） |
| 配置 | ROS1 无 `lidar_max_range`、无 `gravity_align`、`point_filter_num` 走默认 | ROS2 + 30.0 m 量程 + `gravity_align: true` + `point_quality_thresh: 0.9` | **不满足** |
| 时间区间 | 1845 帧，t=0.30 s 起 | 1797 位姿，前 5.19 s/51 帧被 IMU 初始化吃掉 | **不满足** |
| 参考系 | 地图↔参考图差 ≈175.54° | 差 12.7798° | **不满足（被判 BLOCKED 的闭式原因）** |
| 产物口径 | `scans.pcd` 逐帧全分辨率转存（8,923,192 点，含法向） | PGO `save_maps` 的 524 KF 拼接图（字段 x y z intensity） | **不满足（对象不同）** |

⇒ 契约明文：“不能公平比较则明确 BLOCKED，不比较倍数”。**151.19 m 与 1.614 m 不是可比分子/分母。**

### Q4 前端 / 后端 / 坐标 / 评测的贡献是否分离？

| 环节 | 本回放可主张 | 分级 |
|---|---|---|
| 前端/关键帧 | 0 丢帧；局部步长 vs GT 均值 0.5482 vs 0.5474 m（max 差 7.9 cm）；累积漂移 **3.52%/路径**（对本回放是相对量、对齐无关口径） | `[观测]`（漂移**根因**属前端范围，`[BLOCKED]`） |
| 后端/回环 | 真回环被 `correction` 门否决 ⇒ accepted=0 ⇒ 位姿=里程计 | `[观测]`；“决定性瓶颈”属 `[H]` |
| 坐标 | 世界系定义差异 + 参考图播种链同源 GT；独立坐标链恢复 `[BLOCKED]` | `[观测]`+`[BLOCKED]` |
| 评测 | 旧 `eval_map.py` 全图 ICP 后直判 5 cm 不得继承；新 `map_accuracy_eval.py` v2 预审 FAIL（R01–R07/R12）；本轮结果 `status=DIAGNOSTIC_ONLY_ROI` | `FAIL_PRE_REVIEW` |

**未分离项**：18.98 m 的构成、前端漂移根因、世界系错位与地图指标的定量归属——均需另授权实验。

### Q5 唯一优先下一动作，及排除其他动作的理由

**唯一优先动作：消除评测与对照有效性障碍（坐标/ROI/消费者规则/输入来源），不叠加任何新算法或检索器。**
理由：契约与主审一致要求“优先消除评测和对照有效性障碍，不叠加新算法/HBA，不放宽 4 cm 门控”。在现有参考系（175.54° / 12.78°）与 `DIAGNOSTIC_ONLY_ROI` 未闭合前，任何精度数字都不是验收数字；此时改算法既无法判定收益，也违反本轮授权。

**排除**：(a) 新增检索器/后端——无差距的方向性证据、且不可判收；(b) 放宽门控——改参数即改口径，且缺隔离实验；(c) 直接再跑回放——历史 3/3 已耗、本轮授权 0，且无公平比较条件；(d) 用调参表替代——契约禁止。

### Q6 各指标：达标 / 未达标 / BLOCKED（证据层级分开）

| 指标 | 旧值（历史口径） | 新口径重算 | 判定 |
|---|---|---|---|
| 主指标 地图点到面 RMSE ≤0.05 m | replay_1 151.192 m；replay_2 1.6135 m | v2（identity，diagnostic ROI）：replay_1 **1.3077 m**、replay_2 **2.2188 m** | **BLOCKED**（无权威 ROI/坐标链/变换来源；`pass=null`） |
| 双向 C2C | replay_1 189.93/5.93 m；replay_2 3.8445/3.2082 m | v2：replay_1 2.2420/5.1177；replay_2 3.8938/4.0256 | 辅助，无新通过线 |
| 轨迹 ATE（自由 SE3） | replay_1 191.992 m；replay_2 3.1048 m | 同（帧无关，不受坐标错位影响） | 辅助，**不是**主指标 |
| 5 cm 内比例 | replay_2 10.27% | v2 replay_2 7.04% | 辅助，无新通过线 |
| 延迟 | replay_2 22.14/38.43/54.04 ms（节点自报单帧） | 同 | 非端到端；上游 `未记录` |
| 丢帧 | replay_2 0/1848 | 同 | replay_1 `未记录` |
| 覆盖率 | “293/293=100%” | 首末跨度比 91.8%；样本关联率 44.0%/48.1% | 按秒 coverage **BLOCKED** |
| 反事实后端 | — | 离线边界（variant A/B） | `[条件]`，reviewer 未闭合，**不得当验收** |

### Q7 回放预算：历史 3/3 已耗；本轮授权 0、实际 0

`freeze/replay_budget.json`：`authorized_full_replays=0, consumed_full_replays=0`；历史 `total=3, consumed=3, remaining=0`。`eval/runs/` 另有 60 个历史 run 目录（真实在线运行次数远多于“3”，该“3”仅为本轮预算计数）。**未启动 ReplayExecutor。**

### Q8 证据与复算的准确命令

- 后端离线重算（只读）：`python3 audit/backend_evidence/recompute_all.py`、`whole_map_consistency.py`、`offline_loop_se3_check.py`、`offline_counterfactual_posegraph.py`
- 坐标系探针（只读）：`python3 audit/frontend_evidence/07_frame_probe.py`、`audit/history_evidence/yaw_frame_probe.py`、`frame_probe_uncertainty.py`
- localizer 重算：`python3 localizer_evidence/scripts/{analyze_replay3,analyze_cause,analyze_coverage,analyze_footprint_kd}.py`
- 新评测器（**尚未统一授权运行**）：`evaluation/map_accuracy_eval.py`、`gt_time_assoc_eval.py`、`cross_check_map_eval.py` 及相关 `test_*` 模块
- 哈希校验：`sha256sum -c artifacts/04b73f5_evidence_audit_20260929_5V6fKb/freeze/hashes.sha256`
- 逐回放命令行见 `freeze/commands.txt`；本节命令仅登记，**本轮未执行构建/测试/smoke/回放**。

---

## 2. 后端诊断摘要（单回放内，未经终审）

- 位姿链：`poses.txt`(524) 四元数逐位等于 `pgo_keyframes.tum`；后者与 `lio_odom.tum` xyz max 4.98e-4 m、**无一帧差 >1 cm**。
- 回环总账：`events=77 proposed=216 temporal=161 evaluated=55 accepted=0`；门计数（seed 级）见 `audit/backend_diagnosis.md` §3。
- 真回环门值：fine p2pl 2.6 cm、overlap 0.434、eig ratio 0.1428、revisit_rel_t 1.067 m、dyaw −4.01°，**仅 `correction` 10.180 > 2.80 m 失败**。
- 对齐无关交叉验证：kf6↔kf522 里程计 11.133 m / GT 1.164 m（漂移 9.969 m = 路径 3.52%）；转角漂移 3.84°（与配准 `dyaw=−4.01°` 三方一致）。
- `H=Σnnᵀ` 为**纯平移 3×3 单位法向散布**（实测分量和 3920.0 = n_corr），**不覆盖旋转退化**；fine 6-DoF ICP 的 Hessian 未进入该门。
- 离线反事实（**BackendDiagnosis 自行实现的 SE(3) GN/ICP 级联求解器，非仓库算法、非回放**）：`offline_counterfactual_posegraph.json` 给 variant A/B 重访区 NN 与位姿改变量。**范围偏差已登记**：该自写实现**未获授权**（主审已制止），产物**保留但排除于正式验收**，不计完整回放；是否可达待协议与 plan 审查。见 `final_delivery/offline_experiment_ledger.md`。

**可主张（核心，日志/重建事实）**：① `accepted=0` 全程（`events/proposed/evaluated=77/216/55`，`[PGO][loop]`=0）；② `map.pcd` = 524 张关键帧 body_cloud 按 `poses.txt` 顺序拼接，**经验上小残差一致**（多数落在打印/存储量化界内；**但至少 1 个 patch（`115.pcd`）超界约 7%** ⇒ **不得**泛称“全部量化界内”，review 更正）；③ `accepted=0` 使 PGO 关键帧位姿 ≈ LIO 里程计位姿（max 4.98e-4 m）；④ 55 个被评估候选中**仅 1 个**（kf522←6）的唯一失败门是 `correction`（10.180 m > 允许 2.80 m）。

**条件性判断**：该候选在仓库自身配准口径下是几何自洽强候选；条件 = 官方标定把 `body` 定义为 VN200 IMU 帧，但 **LIO 运行时 state 帧等价未测量**。**条件化自一致性 ≠ 外部精度。**

**未确立 / BLOCKED**：完整 6-DoF 验证；“主瓶颈已确证”；任何改善幅度；任何验收级精度。**阶段 3“任务完成”= 审计完成，不是几何根因证明。**

**审查状态**：BackendDiagnosis 已给出第二次稳定的 `audit/backend_evidence/final_stable_manifest.md`（范围/排除闭合，数据/实体链未闭合）。GT 插值方法实为 **NLERP**（早期文本误称 slerp；已登记，旧 `.json` 输出**不重跑不改**，阅读按 NLERP 解释）；自写求解器 `log6` 左雅可比逆方向错误 + 6 维 log 范数混米/弧度标 m。结论以该清单为准。

---

## 3. 坐标/评测链上的硬障碍（为什么主指标 BLOCKED）

1. **参考图非中性**：`seeded = R0 ∘ T_seed(survey)`，`R0` 由**前 20 条 IMU** 算出，运行走**静窗/回退窗 [734,1934)**，两条 acc_mean 不同 ⇒ 约 12.78° 相对旋转（含约 2.55° 倾斜；方向/基不同复算见 R20，非逐位一致）。
2. **上游侧世界系定义不同**：`R0` 是 fork 的滑动窗口重力对齐系；上游 `r_wi=I`、重力为状态。同一张 seeded 图对两次运行**不是同一世界系**（源码事实）。上游侧差异的**量级未定**（175.5394° 的解释已被 review 否决、作者修订中）。
3. **GT 与参考图同源**：MCD GT 由同序列 lidar+IMU 配准到该 TLS 底图得到；`T_seed=inv(T_body_survey(gt_t0))` 依赖 GT，**独立坐标链不存在**（`frame_consistency.md` §7）。
4. **ROI**：现用 ROI 是**参考图 AABB**，`diagnostic_only=true`，不能承载 PASS；`map_accuracy_eval.py` 遇此即 `status=DIAGNOSTIC_ONLY_ROI`、`pass=null`。
5. **变换来源与独立性**：两回放 v2 结果 `alignment.applied="identity"`、`transform_record=null` ⇒ 无“独立来源的 `T_target_source`”，漂移点不可区分“无参考/漂出支持域”。**独立性判据更正**：A/B 先验图与定位序列的**空间 bbox 相交不等于标定数据泄漏**（重访本就空间重叠）；独立性应判**样本/运行/时间索引与来源是否 disjoint**，而非硬性空间不交；拟合区/评价区仍须记录，空间重叠风险明确但**不能自动否决 A-only 标定**；**B 全轨迹参与拟合必须拒绝**。（review R03 相关；EvalContract 若以 bbox overlap 作为唯一独立性判据须修正，并测试“独立 A/B 同空间可接受”。）

⇒ 在 1–5 闭合前，任何 est→ref 的绝对地图指标**不可作为 04b73f5 验收**。

---

## 4. 给下游的引用要求

1. v2 结果（`evaluation/results/replay_{1,2}_*_v2.json`）在 EvalContract 定稿、统一验证通过前**只作诊断**。
2. 轨迹 ATE 可引用（帧无关），但须注明“GT 与参考图同源、是 map-derived trajectory”。
3. 任何 GT 辅助的 yaw/ICP 修正仅作诊断，不入验收。
4. “未记录”一律写“未记录”，不补估值；“达标/未达标/无有效结论”分开写；`pass=null` 不等于 FAIL。
