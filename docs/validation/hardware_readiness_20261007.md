# 验证记录：T2.1 深度审查报告整改

**日期**：2026-10-07
**基线提交**：`ba7389c`（`bernardleex526/T2.1` main）
**对应审查报告**：`T2.1_深度审查报告.md`（本地）

---

## 1. 改动清单

### 1.1 新增：四足步态运动补偿（对应缺口 2，P1）

| 文件 | 说明 |
|:---|:---|
| `src/sensing/fastlio2/src/map_builder/gait_filter.h` | 级联 RBJ 陷波器（纯头文件，不依赖 ROS/PCL） |
| `src/sensing/fastlio2/test/test_gait_filter.cpp` | 21 个信号处理契约用例 |
| `src/sensing/fastlio2/test/test_gait_integration.cpp` | 5 个 IMUProcessor 接线用例 |

### 1.2 修改

| 文件 | 改动 |
|:---|:---|
| `src/map_builder/imu_processor.{h,cpp}` | 新增 `configureGaitFilter()` / `ingestImu()` / `resetGaitFilter()` / `gaitFilterActive()` |
| `src/map_builder/commons.h` | 新增 `gait_*` 配置项（默认全关）；修正 `max_bias_accel` 的失效默认值 |
| `src/lio_node.cpp` | 读取扁平 `gait_*` 键；开启但无频率时打印 `RCLCPP_ERROR` |
| `CMakeLists.txt` | 注册 2 个新测试；**修复 Sophus include 缺失**（见 §4.1） |
| `config/lio_orin_nx.yaml` | 新增 `gait_*` 键与启用说明；补充预检指引 |
| `config/TIME_SYNC_NOTES.md` | 新增 §3.1–§3.3：逐点时间字段的实测方法与厂商事实 |
| `README.md` | 新增第八节"部署工具"与第九节"相关文档索引" |

### 1.3 新增：部署工具（对应缺口 1.2/1.3、缺口 3、缺口 4，P0/P2）

| 文件 | 对应审查项 |
|:---|:---|
| `tools/probe_io.py` | 共用输入层（`.npz`/`.csv`/ROS 2 bag，ROS 可选） |
| `tools/rslidar_pcl2_probe.py` | 缺口 1.2：逐点时间字段与量纲 |
| `tools/imu_allan_variance.py` | 缺口 1.3：`na`/`ng`/`nba`/`nbg` |
| `tools/imu_gait_spectrum.py` | 缺口 2：步态基频与谐波 |
| `tools/odom_static_drift.py` | 缺口 1/3：静止漂移端到端检验 |
| `tools/preflight_config.py` | 缺口 1/4：部署配置自检 |

### 1.4 新增：测试与文档

| 文件 | 说明 |
|:---|:---|
| `tests/test_probe_tools.py` | 56 个用例（含已知参数合成数据的定值检验） |
| `tests/validate_docs.py` | 文档链接/命令/代码围栏校验 |
| `tests/make_layout_fixtures.py` + `tests/fixtures/` | 8 个 PointCloud2 布局夹具 + 1 个可过预检的示例配置 |
| `docs/hardware_deployment.md` | 硬件部署指南（缺口 1/3） |
| `docs/calibration_procedure.md` | 标定流程（缺口 1） |
| `docs/tuning_guide.md` | 调参与排查清单（缺口 4） |
| `docs/test_scenarios.md` | 测试场景与统计口径（缺口 3） |
| `docs/quadruped_adaptation.md` | 四足适配说明（缺口 2） |

---

## 2. 验证结果

### 2.1 构建

```
colcon build --packages-select interface livox_ros_driver2 fastlio2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
build rc=0
```

环境：Ubuntu 22.04 (WSL2) + ROS 2 Humble + g++ 11.4 + ros-humble-sophus 1.22.9102 + ros-humble-gtsam 4.2.0。

### 2.2 C++ 单元测试：44/44 通过

| 测试 | 用例数 | 结果 |
|:---|:---:|:---:|
| `test_gait_filter` | 21 | PASSED |
| `test_gait_integration` | 5 | PASSED |
| `test_imu_init` | 7 | PASSED |
| `test_utils_preprocess` | 11 | PASSED |

### 2.3 Python 工具测试：56/56 通过

```
python3 -m pytest tests/test_probe_tools.py -q
56 passed
```

覆盖的**定值检验**（非"能跑就算过"）：

| 检验 | 方法 | 结果 |
|:---|:---|:---|
| Allan 白噪声估计 | 合成已知 `s`，检验 `N = s/√rate` | 误差 < 0.1% |
| Allan 随机游走估计 | 合成已知 `K`，检验 `σ(τ)=K√(τ/3)` | 误差 ~1% |
| Allan 模型拟合 | 合成已知 `na`/`nba` | `na` 误差 0.0%，`nba` 误差 11% |
| 不可辨识的随机游走 | `τ_c = 2887 s` > 2 h 记录 | 正确报 `None`（而非编数） |
| 平稳性门 | 行走数据 | 正确拒绝（退出码 2） |
| 平稳性门 | 纯噪声（含高噪声） | 正确接受（比例判据，非固定阈值） |
| 逐点时间探针 | 8 个布局夹具 | 8/8 判定正确（含 XYZI 默认无字段、UINT32 不可读） |
| 步态频谱 | 合成 12/20 Hz 步态 | 基频恢复正确；宽带噪声下拒绝输出 |
| 静止漂移 | 已知漂移轨迹 | 通过/超限/跳变/偏航四类判定正确 |
| 配置预检 | 18 个缺陷场景 | 18/18 判定正确 |
| 文档一致性 | 扫描全部文档中的工具命令 | 未知选项可被检出（注入验证） |

### 2.4 文档校验

```
python3 tests/validate_docs.py
scanned 23 markdown files
excluded-by-policy links: 26
BROKEN LINKS: 0 / MISSING TOOLS: 0 / UNBALANCED FENCES: 0
DOC VALIDATION PASSED
```

26 条"策略排除"链接指向 `artifacts/` 等仓库**按 README §七.2 有意不发布**的路径，非断链。

### 2.5 Lint 基线对比（证明无回归）

对**纯净**基线（`git clone` + `checkout ba7389c` + `git clean -fdx`）与改动后工作树分别构建并跑 lint：

| 项 | 基线 | 改动后 |
|:---|:---:|:---:|
| uncrustify 被标记文件数 | 21 | 24 |
| 新增被标记文件 | — | 3（**恰为本改动新增的 3 个文件**） |
| 既有文件新回归 | — | **0** |

**根因**：ROS 默认 `ament_code_style.cfg` 要求 `indent_columns = 2`，而本包全部使用 4 空格缩进。
基线中 **22 个源文件里有 21 个**已被标记，即这是**仓库既有的风格配置不匹配**，与本次改动无关。
新文件沿用本包既有的 4 空格约定，因此与既有文件同样"不合规"——单独重排新文件反而会
使其与包内其它文件风格不一致。

> 说明：`ament_uncrustify <单个文件>` 直接调用会报告 0 处改动；差异来自 ctest 走的是
> `ament_uncrustify` 无参数路径（对整个包按默认配置检查）。上表数字取自 ctest 路径，与
> `colcon test` 一致。

---

## 3. 与审查报告的偏差（有意为之）

审查报告的部分建议若照字面实施，会与**仓库自身的诚实性策略**冲突。以下为有意偏差：

| 报告建议 | 本实现 | 理由 |
|:---|:---|:---|
| 直接填入 `ext_il: [-0.015, -0.025, 0.045, ...]` | **不填**，保持缺失 | 该值是示例。仓库策略明确禁止提交未经实车标定的外参（README §七.2）；填入占位值会让估计器"看起来已配置" |
| 直接填入 `pcl2_time_field: "timestamp"` | **不填**，保持空；改为提供探针工具 | 速腾 `rslidar_sdk` **默认** `POINT_TYPE=XYZI`，根本没有该字段。猜测字段名与留空等价（`find_offset` 找不到即退化为不去畸变），却会掩盖问题 |
| 直接填入 `notch_freq_hz: [15, 20, 25]` | **不填**，保持关闭；改为提供频谱工具 | 对不存在的频率做陷波会删掉真实运动，比不滤更糟 |
| 声称"补偿后精度提升 >30%" | 不声称 | 该数字来自合成数据推断，本仓库无硬件数据支持 |
| 声称 Allan 曲线可直接得到全部四个参数 | 明确区分"可辨识/不可辨识" | 报告给出的 `ng=0.005`/`nbg=3e-6` 交叉点 `τ_c≈2887 s`，2 h 记录**物理上无法**辨识 `nbg` |
| 陷波器保留无界 `std::deque` 历史 | 只保留 2 个状态 | 双二阶滤波器不需要无界历史 |
| "先滤 z 再用原值覆盖 z" | 按轴掩码，未选中轴不进滤波器状态 | 报告写法会让偏航轴白跑一遍并推进状态（状态泄漏） |
| 忽略群延迟 | 提供并打印群延迟 | 陀螺与加速度计若延迟不同，相对时序被破坏，而预积分依赖该对齐 |

---

## 4. 顺带修复的既有缺陷

### 4.1 Sophus include 缺失（**阻断任何构建**）

`CMakeLists.txt` 使用 `${Sophus_INCLUDE_DIRS}`，但 `ros-humble-sophus` 只导出
INTERFACE IMPORTED 目标 `Sophus::Sophus`，**不定义该变量**。因此变量展开为空，
`#include <sophus/so3.hpp>` 报 "No such file or directory"，**整个包无法编译**。

修复：优先取 `Sophus::Sophus` 的 `INTERFACE_INCLUDE_DIRECTORIES`，回退到变量（兼容源码构建的
Sophus），两者皆无时 `FATAL_ERROR` 并说明原因。

> 该缺陷与本次审查无关，但会阻断验证，因此一并修复。

### 4.2 `max_bias_accel` 默认值失效

`commons.h` 声明 `max_bias_accel = 0.2`，但：
- `ieskf.cpp` 把静态量初始化为 `0.5`；
- `lio_node.cpp` 在键缺失时回退到 `0.5`，且**总是**用该值覆盖 `Config` 的字段。

即 `commons.h` 中的 `0.2` **从不生效**，实际生效值为 `0.5`。已将 `commons.h` 改为 `0.5`
并加注说明，避免读该结构体时被误导。（本仓库仅有一处 `Config` 实例，即节点内，故无其它消费者。）

### 4.3 `tools/rslidar_pcl2_probe.py` 的 `--once`

子代理在编写硬件部署指南时发现：仓库内 6 处文档（README、标定流程、四足适配、调参指南、
TIME_SYNC_NOTES）使用了 `--once`，而工具未定义该选项，实际会以
`error: unrecognized arguments: --once` 退出。

修复方式为**实现该选项**（等价于默认行为——探针本就只读一帧），而非改写 6 处文档：
`--once` 是 ROS 惯用写法（`ros2 topic echo --once`），保留它可让文档中的命令直接复制执行。
同时新增回归测试 `test_once_flag_is_accepted` 与 `test_every_documented_command_line_parses`，
后者会扫描全部文档中的工具命令并校验每个选项真实存在（已通过注入错误选项验证其有效）。

---

## 5. 复现命令

```bash
# C++ 构建与测试
mkdir -p /tmp/ws/src && cd /tmp/ws
ln -s <repo>/src/sensing/fastlio2          src/fastlio2
ln -s <repo>/src/sensing/livox_ros_driver2 src/livox_ros_driver2
ln -s <repo>/src/common/interface          src/interface
source /opt/ros/humble/setup.bash
colcon build --packages-select interface livox_ros_driver2 fastlio2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select fastlio2

# Python 工具测试与文档校验
cd <repo>
python3 -m pytest tests/test_probe_tools.py -q
python3 tests/validate_docs.py

# 部署配置预检（出厂 profile 预期为 NOT READY）
python3 tools/preflight_config.py --config src/sensing/fastlio2/config/lio_orin_nx.yaml
python3 tools/preflight_config.py --config tests/fixtures/calibrated_profile_example.yaml
```

---

## 6. 仍未完成（需要硬件）

以下项目**本仓库无法完成**，需要目标平台与实物：

| 项目 | 阻塞原因 |
|:---|:---|
| `ext_il` 实测标定 | 需实物与标定数据 |
| `pcl2_time_field` 实测值 | 需 Airy/Odin1 驱动与数据（且可能需重编 SDK） |
| `na`/`ng`/`nba`/`nbg` 实测 | 需 ≥2 h 静止数据 |
| 步态频率实测 | 需实走数据 |
| Orin NX 算力/温度 | 需板端 |
| 真实场景回环验收 | 需全站仪/RTK 与场地 |
| 时间同步方案与实测误差 | 需硬件授时链路 |

工具与流程已就位，上述每一项都有对应的命令与判据（见 `docs/calibration_procedure.md`）。
**在这些项目完成之前，不应声称 T2.1 已在目标平台验证。**
