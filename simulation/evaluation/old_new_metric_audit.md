# 旧新评测差异审计（04b73f5 基线，零新增回放）

- 基线 commit：`04b73f553918b1dcc751feac69c0a2834f194432`
- 旧评测器：`/home/lee/t21_wp2/eval/{eval_map.py,eval_trajectory.py,eval_speed.py,tools/metric_audit_gtinterp.py}`（**未改动、未覆盖输出**）
- 新评测器：本目录 `map_accuracy_eval.py` / `gt_time_assoc_eval.py` / `cross_check_map_eval.py`（v2.2.0，hash 见 `evaluator_hashes.json`）
- 新增回放：**0**。新数字全部来自已有产物重算或合成已知契约用例。
- 结论先行：**本基线不产出任何精度 PASS/FAIL**。预定义 ROI、独立来源坐标链、历史 TF 消费者规则均缺失 → BLOCKED。

---

## 1. 旧主指标是如何被掩盖的（有据可查）

数据来源：`artifacts/04b73f5_diagnosis_20260929_JWUGeL/replay_{1_upstream,2_fork}/eval/map.json`（旧 `eval_map.py` 输出，原样保留）。

| 项 | replay_1_upstream（上游） | replay_2_fork（本仓） |
|---|---|---|
| `mode` | accuracy_vs_reference_map | accuracy_vs_reference_map |
| `accuracy_measurable` | **true** | **true** |
| `verdict.pass` | false | false |
| est→ref 点到面 RMSE | **151.19 m** | **1.6135 m** |
| 中位数 / p99 / max | 5.89 / 508.09 / 556.45 m | 0.585 / 6.79 / 21.31 m |
| `frac_within_5cm` | 1.4 % | 10.3 % |
| ICP 平移 / 旋转 | 4.283 m / 8.37° | 0.616 m / 2.69° |
| ICP final fitness | **0.0071** | 0.076 |

旧报告的问题不在数值算错，而在**语义**：

1. **无条件全域 ICP**：主值一律在“ICP 之后”的坐标系里计算，且 ICP 一旦移动了估计，该配准就拟合在评价数据上；旧输出没有任何“注册是否失败/是否可用于认证”的字段，只给了一个 `pass`。
2. **没有覆盖记录**：两张表都没有“估计有多少落在参考支持域内”“有多少完全没有对应”。事后用新评测器重算 replay_2 得到：参考支持覆盖率 **3.2 %**、漂出支持域 **42.7 %**、无法判定（无参考 vs 漂出）**54.0 %**。这些量旧报告一个都没有。
3. **没有固定分母/截断台账**：旧输出给了 p95/p99/max，却没有“分母是多少、若截断排除了多少”的记录，无法防止“删大残差降 RMSE”。
4. **采样退化未防**：旧实现对 est/ref 用同一体素栅格 `voxel_down_sample`，两团云可能被压到同一格心、把真实残差压成 ~0；新实现两套栅格原点相差半个体素，并按 **3D 点重合**（C2C ≤ 1e-6 m）单独报退化比例。
5. **法向不设规范**：旧实现用 `orient_normals_consistent_tangent_plane`，点面残差对法向符号/朝向有依赖；新实现用 |d·n| 且法向在“参考最近点位置”处估计（与严格交叉核验同一估计量）。
6. **时间量名不副实**：旧 `metric_audit_gtinterp.py` 的 `association_coverage` 是“插值有效期占比”（样本比例），不是时间覆盖；且 `ate_se3_aligned` 会吸收全局框架误差。replay_1/2 的 `tau_0` 样本比例 0.993 / 0.997，但原始 ATE 265.46 / 115.97 m。新实现把**样本关联率**与**按秒区间并集覆盖**分成两列，并以完整 GT 窗为分母。

### 关于“参考系/坐标链”的表述边界
`EvidenceAudit` 的 frame_consistency 证据仍在修订中（v2）。本审计**只做一件可确证的事**：新旧两个评测器面对**同一批产物**时，对新评测器而言该比较**不构成精度测量**（支持覆盖 3.2 %，状态 BLOCKED/不可测），而旧评测器在同一情形下仍然输出了 `accuracy_measurable=true` 与一个 `pass` 判决。审计**不主张**错位的具体成因、不外推到点到面或算法改善、不引用任何“误差减半/纯偏航”类结论。

---

## 2. 新评测器在同一产物上的结果（离线重算，`--stat-voxel 0.25`）

产物：`evaluation/results/replay_{1_upstream,2_fork}_map_accuracy_v2.json`；ROI 为 `results/roi_diagnostic_mcd_tuhh_night_09.json`（**diagnostic_only=true**，仅由参考点云外接框导出，非观测条件化，永不可 PASS）。

| 项 | replay_1_upstream | replay_2_fork |
|---|---|---|
| `status` | NOT_MEASURABLE_LOW_COVERAGE | DIAGNOSTIC_ONLY_ROI |
| `pass` | null | null |
| 诊断态（若 ROI 合规） | — | BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM |
| 固定分母 | 272 529 | 231 432 |
| 采样估计点总数 / 落在冻结区外 | 1 815 118 / **85.0 %** | 299 149 / 22.6 % |
| 参考支持覆盖率 | 未计算（前项已否） | **0.0324** |
| 漂出支持域 | — | 98 889（42.7 %） |
| 无法判定（无参考 vs 漂出） | — | 125 036（54.0 %） |
| 点到面 raw RMSE | 不计 | 2.1849 m（分母 231 432，**仅诊断**） |
| 点到点 RMSE | — | 3.8819 m |
| 3D 点重合比例 | — | 0.0 |
| `unknown_states` | — | UNKNOWN_REGIONAL_GAP, UNKNOWN_SUPPORT_CLASSIFICATION |

严格交叉核验（同估计量、独立算术、同一分母 231432）与主值相差 0.249 m，而折间标准误达 0.56 m，故判定 `inconclusive_tolerance_too_large`，**不称“算术已验证”**。原因可复算：该数据 **96.8 % 的样本没有对应**，点到面残差被少数超大残差主导，两种法向/残差实现在“无对应”区域的差异被放大。结论：在支持域缺失的数据上，该指标本身病态，交叉核验只在有对应处才有分辨力——这与“帧对齐/支持域”问题必须一并解决。

**这不是“新方法更差”**：体素（0.25 vs 0.05）、采样规则、分母与是否先做 ICP 都不同，两栏不可作同量比较；本节只说明**可测性判定**的差别。

---

## 3. 逐项差异（旧 → 新）

| # | 维度 | 旧 | 新 v2.2 |
|---|---|---|---|
| 1 | 可测性 | 恒 `accuracy_measurable=true` | 空重叠/极低覆盖/未对齐/未来源变换/退化 → NOT_MEASURABLE / BLOCKED，`pass=null` |
| 2 | ICP | 无条件全域 ICP 后取主值 | 默认关闭；`--icp conditional` 仅诊断，若移动了估计则 `pass=null` |
| 3 | 分母 | 无定义（体素降采后的 est 点数） | 冻结网格 + 冻结区域内，先于任何截断，`membership_hash` 可复现 |
| 4 | 区域 | 无 ROI 概念 | 预定义 ROI（`roi_schema.json`），必须 `independent_of_estimate/error=true`，否则 BLOCKED |
| 5 | 支持域 | 无 | supported / drifted-out / no-reference-locally 三分，最近邻在**全参考云**中搜索 |
| 6 | 截断 | 只报分位数 | raw 恒为主值；截断另报 excluded_n/fraction + 固定分母，且**不得据截断 PASS** |
| 7 | 法向 | open3d 一致朝向 | PCA；符号规范化；|d·n|；查询位置=参考最近点 |
| 8 | 采样参数 | 未记录 | voxel/两个栅格原点/seed/normal k,radius/support radius/权重 全部入结果 |
| 9 | 退化 | 无 | 3D 点重合比例（>0.99 硬拒；>0.05 判据性 unknown） |
| 10 | 区域/法向统计 | 无 | ROI 2×2×2 八分区 + 按参考法向主轴分类，含 p99/max |
| 11 | 薄结构 | 无 | 参考/估计平面厚度 + 双墙疑似比例 |
| 12 | 时间 | `--t-offset` 可任意；`association_coverage`（样本比例）被当覆盖 | 有来源 offset（GT 拟合的仅诊断）；gap 限插值；**样本关联率与按秒覆盖分列**；完整 GT 窗为分母 |
| 13 | 框架链 | 未定义 | 必须声明四元（source/target frame、sensor/gt entity）；缺链 BLOCKED；杆臂+SE3 实际施加；无链对照值标 `raw_unaligned` |
| 14 | 变换独立性 | 无 | 按**样本/运行/时间索引 provenance** 判互斥（空间重叠不构成泄漏），缺证 → NOT_PASSABLE |
| 15 | 交叉核验 | 无 | `--mode strict`（同估计量、独立算术）与 `--mode sensitivity`（异估计量诊断）分离，容差用实测折间标准误 |

---

## 4. 复算命令

完整验证计划（含期望与失败含义）：`verification_plan.json`（V1 合成+协议+parity 全模块；V2 离线重算；V3 严格交叉核验；V4 CLI 冒烟）。
复算命令：`bash recompute_commands.sh`（等价命令亦写入 `verification_plan.json` 的 V2/V3）。运行不涉及任何在线算法、bag 或前端；只读旧产物与参考图，输出写入 `evaluation/results/`。

---

## 5. 仍然 BLOCKED（不是本任务可闭合项）

1. **预定义观测条件化 ROI**：历史输入中没有；需要数据集侧给出（owner: DatasetQualification）。当前只用 diagnostic_only ROI，故永不 PASS。
2. **独立来源坐标链**：`T_target_source` + 杆臂 provenance 在历史运行中无记录 → BLOCKED。
3. **TF 消费者规则**（零阶保持有效期 / 关联容差 / 缺口上限）历史运行未记录 → 在线可用率 BLOCKED。
4. 主指标精度结论：在上述 1–2 解除前，本基线**不得**出现精度 PASS/FAIL。
