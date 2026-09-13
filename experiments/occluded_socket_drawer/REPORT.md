# 实际运行报告：遮挡入座与抽屉归位

执行日期：2026-09-09 至 2026-09-10。仓库：`/home/inspur/project/RoboTwin`。

已完成真实物理场景、RGB-D 定位、H/S 控制、双臂共同物理循环、四组录像和独立测试。最终 seed 0 四组全部成功。冻结后 20 个独立种子的 80 次物理执行全部成功；另有 40 次 GPU 1 渲染启动失败，全部保留，合计 **120 次尝试、80 次成功、40 次失败**。不能把 80/80 的物理执行结果写成全部尝试成功率。

**数据不支持预期的机制性反转。** seed 0 中 S 在 LOW/HIGH 都更快。独立测试仅 seed 104 有 LOW 下 H 快 0.004 s、HIGH 下 S 快 0.100 s 的严格符号变化；LOW 差异等于一个物理步，且该实例 0.1 s 就从右腕获得信息，不能解释为“空闲助手观察更便宜、繁忙助手机会成本更高”的证据。没有添加假等待或删除不反转的种子。

## 1. 环境、版本与改动范围

仓库基线 commit 为 `2f84b98bd64d8d1f3d8be79462b6b216b81820ff`。开始时工作区干净，未找到适用 AGENTS.md。新增文件，没有修改原有跟踪文件，没有破坏性 Git 操作、训练、安装或升级依赖。

| 项目 | 实际使用 |
|---|---|
| Python | `/mnt/sda/conda_envs/RoboTwin/bin/python`，3.10.21；`PYTHONUTF8=1` |
| 仿真/规划 | SAPIEN 3.0.0b1；MPLib 0.2.1；CPU PhysX、FK/IK 与碰撞检查 |
| 视觉/数值 | OpenCV 4.10.0.84、NumPy 1.26.4、SciPy 1.10.1 |
| 机器人 | 原有 `aloha-agilex` 配置及 `arx5_description_isaac.urdf`；活动前臂 fl/fr，各六自由度 |
| 渲染 | NVIDIA RTX 5880 Ada 48 GB，GPU 0，驱动 580.159.03 |
| 视频/绘图 | 已安装 imageio、imageio-ffmpeg、matplotlib；没有新增依赖 |

9 月 9 日八张 GPU 都繁忙，9 月 10 日继续时空闲。先在 GPU 0 跑单实例，之后尝试两个独立 episode 进程。GPU 1 在 scene 创建前失败：本地 beta 的旧 `SapienRenderer()` 初始化先选中了 GPU 0，随后显式 `RenderSystem(device="cuda:1")` 报错 `current SAPIEN version only supports single-GPU rendering`。完整迁回 GPU 0 的分片与另一进程有短时并发，各自只有一个 scene；没有多个线程 step 同一个 scene。未停止用户进程，结束时 GPU 使用率为 0%。其他 GPU 编号尚未验证。

完整环境及原机器人 config/URDF/SRDF SHA256：[ENVIRONMENT.json](ENVIRONMENT.json)。冻结任务源码、配置、种子与预算：[FROZEN.json](FROZEN.json)。GPU 执行偏差：[EXECUTION_DEVIATIONS.json](EXECUTION_DEVIATIONS.json)。冻结后未改动基线源码或 YAML；后续仅完善离线分析、补充探针、测试和报告。

| 新文件 | 内容 |
|---|---|
| `envs/occluded_socket_drawer.py` | 复用 RoboTwin Base_Task/Robot 的程序化场景与 checkpoint 初始化 |
| `env_cfg/task_config/occluded_socket_drawer.yml` | 几何、相机、速度、视觉门限与评测配置 |
| `envs/occluded_socket/geometry.py`、`observation.py` | 坐标与固定标定；允许的 RGB-D、FK 观测 |
| `envs/occluded_socket/perception.py` | 彩色几何标记检测、深度定位和质量门限 |
| `envs/occluded_socket/motion.py`、`controller.py` | 共享运动限制、碰撞检查、独立双臂队列、H/S 与有限替代方案 |
| `envs/occluded_socket/evaluation.py`、`recording.py`、`runtime.py` | 独立 GT 评测、日志录像、共同物理循环及显式 oracle |
| `scripts/occluded_socket_demo.py` | audit / oracle / drawer / episode / paired / batch 入口 |
| `scripts/occluded_socket_probe.py` | 真实相机、可达性和持物自观察探针 |
| `scripts/summarize_occluded_socket.py`、`audit_occluded_evidence.py` | 全尝试统计、配对比较、图表、视频合成和离线证据审计 |
| `experiments/occluded_socket_drawer/` | README、进度、实际报告、冻结及环境记录、8 项 CPU 契约测试 |

## 2. 实验定义与数据边界

指令始终是“将右侧模块正确装入定位座，并把左侧抽屉关闭”。这是 **cold checkpoint decision experiment**：右臂通过夹爪接触持物，两臂处于公开固定准备姿态，先物理稳定 3 s；四组使用相同规则，这段不计决策时间。策略没有 checkpoint 之前的观察历史。t=0 后通过关节驱动、重力与碰撞推进，没有物体/机器人/抽屉位置传送。

模块底板为 76×64×16 mm；定位座开口为 104×92 mm，有真实底板与四面低墙。外壳保留侧壁/后壁，因开发中的真实夹爪碰撞移除了屋檐。槽位 XY 各均匀随机 ±12 mm，姿态水平，标记与随机插槽刚性连接。固定外壳不编码当前槽位。LOW 抽屉 q=0；HIGH q=0.22 m，关节最大行程 0.24 m，无自动归位驱动或回弹弹簧。抽屉由夹爪实际推合。实体临时架已构造。

策略使用实际左右腕和允许的 head RGB-D、固定标定、机器人关节/FK、已知工装和模块尺寸。overview 仅录像使用。RGB 检测四个彩色标记，通过光学深度与几何关系估计平移；要求四点完整、有效深度、重投影 ≤3 px、几何误差 ≤6 mm，以及连续两帧一致。姿态不随机，所以不宣称估计了未知六自由度位姿；质量分数不是概率。

`T_A_B` 将 B 坐标映射到 A，米制；SAPIEN Pose 使用 wxyz。腕相机采用 URDF 固定安装变换，SAPIEN 前/左/上与 OpenCV 右/下/前之间显式转换。安装目标满足：

```text
T_world_camera_cv = T_world_ee(q) @ T_ee_camera_sapien @ T_sapien_cv
T_world_ee_goal = T_world_socket_est @ T_socket_module_goal @ inverse(T_ee_module)
```

抓取变换来自公开设计标定，实际接触滑移仅写入 GT 日志。没有每步用模块真值更新标定。控制器不持有环境、目标 Actor、随机种子、LOW/HIGH 标签或 evaluator；未调用本地会读取 Actor GT 的 `grasp_actor` / `place_actor` / functional-point 辅助函数。规划器只包含机器人、公开固定工装和图像估计的抽屉，不含隐藏随机槽位/标记。oracle 仅在显式 `mode=oracle` 注入目标，视觉失败不回退。

H 让左臂帮助观察，信息有效后右臂安装、左臂直接去做抽屉；S 左臂立即做抽屉，右臂尝试持物侧移/转腕。任何角色都可利用途中自然获得的有效图像。安装依次预接近、入座、松爪、退出。两者共用速度/加速度、感知、安装控制、成功标准和碰撞处理。规划碰撞时至多尝试一次固定上方避让，再尝试下一观察候选或记录失败；没有为某策略添加假等待。

每 tick 分别更新左右目标，再唯一一次 `scene.step()`；dt=0.004 s，控制 250 Hz，相机/录像 10 Hz。关节速度上限 0.8 rad/s、加速度 1.6 rad/s²，Cartesian 上限 0.12 m/s、0.3 m/s²。开发时修正了粗采样线性插值造成的加速度尖峰，冻结版本使用 C2 三次样条和物理网格时间缩放，两种策略相同。配置是轨迹指令限制，不是对接触冲击或真实关节跟踪加速度的保证。

主时间是累计物理时间，包含观察移动、实际等待、重试、避让及安全退出。CPU 规划、渲染与感知发生在 tick 边界，wall-clock 单独记录，不模拟计算延迟。因此结果不能等同真实机器人端到端吞吐率。

右侧成功要求模块位置/姿态/速度合格、实际夹爪已打开并稳定 0.5 s；左侧要求抽屉 |q|<6 mm、|v|<0.01 m/s 并稳定 0.5 s。模块 XY 分量门限 15 mm、Z 8 mm、角度 12°，线/角速度门限分别 0.015 m/s 与 0.1 rad/s。每 tick 重查同时成功，失稳会撤销旧成功时间。整体完成还包括队列结束、安全退出及无未处理控制失败。因此 t_all 可以晚于 max(t_right,t_left)。未完成项为 null，失败在 Success@Budget 中按失败处理。

## 3. 最终四组实测：seed 0

权威版本目录为 `/home/inspur/project/RoboTwin/result/occluded_socket_drawer/release_pair`。下表均为实际累计物理秒，包含安全退出。

| 条件/策略 | 成功 | 信息就绪 | 右侧稳定完成 | 左侧稳定完成 | 整体完成 |
|---|---|---:|---:|---:|---:|
| LOW H | 是 | 3.300 | 7.836 | 0.500 | **9.352** |
| LOW S | 是 | 1.600 | 6.272 | 0.500 | **7.788** |
| HIGH H | 是 | 5.200 | 9.736 | 13.620 | **15.220** |
| HIGH S | 是 | 1.600 | 6.272 | 10.820 | **12.456** |

H 的信息来自左腕，S 来自右臂持物移动后自然露出的 head 图像。没有为了完成右腕观察路径而忽略更早信息。HIGH H 第一个观察候选受已估计抽屉阻挡，规划拒绝记录在 0 s，随后采用下一候选；该拒绝没有执行运动。H 在获得信息前抽屉位移为 0。HIGH H/S 实测两臂同时运动分别为 4.680 / 6.144 s；操作事件和关节记录证明右侧安装与左侧抽屉操作重叠。

同条件 H/S 完整初始化快照相同；LOW/HIGH 的右侧模块、槽位及右臂初始状态相同。seed 0 最早采用的视觉估计误差最大约 1.39 mm。S 的右侧和整体在两种条件下都更早，没有出现待检验的“牺牲右侧完成时间以减少整体时间”。

早期 `paired_final/`、`paired_demo_*/` 及 `visual_*` 是开发数据，部分使用旧几何或旧轨迹重采样，不能混入上述最终表。原始数据全部保留，修正原因见 [PROGRESS.md](PROGRESS.md)。

## 4. 独立测试、失败分母与反转分析

开发种子 0..4 共 20 次，全部成功；该开发批次在最终 C2 重采样修正前完成。修正后重新跑上述 seed 0 四组并通过 7 项 CPU 契约，随后在 `2026-09-10T06:08:47.728023Z` 冻结基线。独立种子预先固定为 100..119，预算每次 35 s。冻结后没有按测试结果改参数或筛选种子。

| 尝试分片 | 种子 | 尝试数 | 初始化成功 | 任务成功 | 失败 |
|---|---|---:|---:|---:|---:|
| `test_20/gpu0` | 100..109 × 四组 | 40 | 40 | 40 | 0 |
| `test_20/gpu1` | 110..119 × 四组 | 40 | 0 | 0 | 40 次渲染启动失败 |
| `test_20/gpu1_retry_on_gpu0` | 原分片全部 110..119 × 四组 | 40 | 40 | 40 | 0 |
| 全部尝试 | 保留重试与失败 | **120** | **80** | **80** | **40** |

原计划首轮为 40/80 成功；完整分片重试为 40/40。全部尝试成功率 66.67%；真正进入物理执行的 80 次为 80/80。重试由渲染初始化失败触发，不是根据任务结果挑选；代码、几何、门限、预算均相同。汇总 CSV 每行保留 `attempt_cohort` 和完整 run 路径，不会以重复 seed 覆盖失败。

以下均值只对成功物理执行统计，每组 20 个；失败仍进入左侧成功率分母：

| 组 | 全尝试成功/总数 | 失败原因 | 平均信息时间 | 平均右侧完成 | 平均左侧完成 | 平均整体完成 |
|---|---:|---|---:|---:|---:|---:|
| LOW H | 20/30 | 10 次渲染初始化失败 | 1.2050 | 5.8118 | 0.5000 | 7.3252 |
| LOW S | 20/30 | 10 次渲染初始化失败 | 0.6200 | 5.2738 | 0.5000 | 6.7872 |
| HIGH H | 20/30 | 10 次渲染初始化失败 | 1.8850 | 6.4904 | 11.8806 | 13.4812 |
| HIGH S | 20/30 | 10 次渲染初始化失败 | 0.6200 | 5.2738 | 10.8200 | 12.4560 |

Success@Budget 同样包含全部尝试；单位 %，每组分母 30：

| 预算秒 | LOW H | LOW S | HIGH H | HIGH S |
|---:|---:|---:|---:|---:|
| 5 | 0 | 0 | 0 | 0 |
| 8 | 43.33 | 66.67 | 0 | 0 |
| 10 | 66.67 | 66.67 | 0 | 0 |
| 12 | 66.67 | 66.67 | 0 | 0 |
| 15 | 66.67 | 66.67 | 53.33 | 66.67 |
| 20 / 25 / 30 / 35 | 66.67 | 66.67 | 66.67 | 66.67 |

配对完成时间条件为 **H 和 S 都成功**，每条件 20 对；正的 H−S 表示 S 更快：

| 条件 | 平均 H−S | 中位数 | 范围 | H 更快 / 相同 / S 更快 |
|---|---:|---:|---|---|
| LOW | 0.5380 s | 0.0000 s | −0.004 至 1.708 s | 1 / 12 / 7 |
| HIGH | 1.0252 s | 0.1000 s | 0.100 至 3.016 s | 0 / 0 / 20 |

40/40 个已初始化 H/S 配对的实际初始状态完全相同；另 20 对首轮 GPU 1 失败没有物理初始状态，不将它们描述为状态不匹配。

13 个种子（100、102、104、105、107、108、109、110、112、115、116、118、119）在两帧检查后 0.1 s 即由**右腕**提供有效定位；两策略均利用，说明 checkpoint 并非在整个随机范围都持续强遮挡。这不是 head 初始泄漏，也不是强制丢弃已有图像。其余 7 个种子，H 从左腕获得信息：LOW 3.2–3.3 s，HIGH 5.1–5.3 s；S 在持物移动途中由 head 于 1.4–1.7 s 获得信息。

该 7 个仍需明显观察运动的实例中，S 的 LOW 优势为 1.436–1.708 s，HIGH 优势为 2.464–3.016 s；没有反转。这是解释性分组，全部 20 个仍保留在主统计里。原假设的两个前提均未充分成立：H 在 LOW 不是更便宜的观察方案；S 没有因自观察让右侧明显更晚。HIGH 的左臂并行工作确实减少了等待，但它扩大了 S 已有的优势。

seed 104 的严格符号变化完整保留在 `summary.json` 的 `raw_sign_reversals` 中。LOW 仅一个 4 ms 控制步的差异，不能作为有意义的观察角色收益反转。没有改阈值来把这个符号变化删除，也没有宣称统计显著性或全局最优。

## 5. 视觉、并行、物理与版本证据

离线审计输出：`result/occluded_socket_drawer/test_20/evidence_audit.json`。这里使用 GT 仅做事后验证，没有反馈给策略。

- 120/120 份 run 源码哈希与冻结基线相同，120/120 份保留源码文件与各自 provenance 哈希相同；当前基线源码也未变。
- 80 次物理执行的初始标定最大矩阵差为 6.0145e−7；首次采用的视觉平移误差最大 1.4924 mm。
- 随随机插槽变化，视觉估计和实际下发的安装 EE 目标 XY 跨度均为 22.572 × 22.899 mm；80 次记录的 EE 安装目标与视觉估计坐标链完全相符。
- 对 80 份真实有效观测离线遮黑 RGB、保留原深度与标定，所有感知调用均拒绝定位，没有从深度/隐藏真值自动恢复目标。此项是**感知接口负例**，没有把它写成 80 次额外机器人遮挡运行。
- 动作起止/取消配对无错误，成功终止无未完成动作，关节日志时间递增。HIGH H 的安装/抽屉同时运动样本覆盖约 4.56–4.86 s，HIGH S 约 4.72–5.06 s（50 Hz 日志估计）；精确全 tick 的双臂运动时长另存每次 result。
- 所有 HIGH H 在信息就绪之前抽屉位移为 0。LOW/HIGH 配对的右侧初始快照差为 0。
- 80 次最后记录的模块误差：|X|≤2.041 mm、|Y|≤1.893 mm、|Z|≤0.000282 mm，角度≤0.489°；抽屉 |q|≤0.160 mm，同时稳定条件全部为真。微小 Z 误差来自理想平底刚体仿真，不代表真实装配精度。
- 除模块/夹指/插槽及抽屉/夹指预期接触外，记录到的其他前臂接触仅空左夹爪两指之间的小冲量，最大 0.002109 N·s；没有记录到外壳或桌面接触。接触日志为每 5 tick 采样且忽略低于 0.0001 N·s 的冲量，**不是**连续无碰撞证明；PhysX 全程保留实体碰撞。

8 项 CPU 契约测试通过：相机轴、抓取坐标链/目标变化、空图拒绝、GT API 边界、唯一 post-checkpoint step 调用点、拒绝轨迹不执行、物理采样率下轨迹速度/加速度、模块离座后撤销成功。日志位于 `result/occluded_socket_drawer/contract_tests.log`。它们与真实仿真成功分别报告，不相互替代。

## 6. 分阶段检查与替代策略

| 检查 | 实际结果与边界 |
|---|---|
| 现有任务 headless smoke | `stage0/smoke` 及最终 `supplementary/audit_release/smoke` 均成功创建、稳定并渲染现有 shelf 场景；不宣称完成原任务操作 |
| 右侧 oracle | 开发先验证安装可行；最终 `supplementary/oracle_release` 成功 6.036 s，仅右侧 oracle，不计视觉机制统计 |
| 单独抽屉 | 开发 `drawer_006` 成功，稳定关闭 10.908 s、安全退出 12.668 s；该记录为最终 C2 修正前开发数据 |
| 持物右腕自观察 | `supplementary/camera_validation` 实际执行并检测成功；原始左右腕/head 图像、像素坐标、深度与标定证据保留 |
| 左臂先推半程再帮助 | seed 0 HIGH，`supplementary/partial_helper` 成功；信息 10.900 s、右完成 15.440 s、左完成 16.232 s、整体 **17.508 s**，比基线 H 的 15.220 s 更慢 |
| 安装后右臂接手抽屉 | seed 0 HIGH，`supplementary/right_handoff` 实际尝试；右安装退出后 11.252 s 发起接手，左臂取消推合并于 14.624 s 完成让位，右侧抽屉 hover IK 失败；35 s 超时、整体完成 null |

相机补充探针的 checkpoint 右腕因重投影/几何门限拒绝、head 缺一个角点；左腕帮助视点通过，右臂持物侧移视点也通过。有效右腕图中各标记面积约 55–260 px，保持实际夹持，模块相对抓取标定的平移偏差约 6.2 mm。探针按“checkpoint→左观察→右观察”顺序运行，12.08 s 的右腕有效时间包含此前左观察，**不能**当作独立 S 的观察成本。S 基线优先采用途中更早的 head 信息。

右臂接手失败是本版给定接近姿态/路径的可达性失败，不证明所有右手接手方式都不可行。补充策略仅检查 seed 0，没有独立种子优势结论。临时架存在，但通用暂放/重抓恢复未实现；当前采用并验证的是用户要求优先考虑的持物观察。没有禁止更好的替代路径来维持反转。

## 7. 实际命令与完整复现

以下列出本轮真正执行过的主要命令，工作目录为仓库根目录。每个 run 的 `provenance.json` 保存其实际 Python 路径、完整 argv、依赖版本和源文件快照。历史输出已存在，不能原地重复写入。

```bash
cd /home/inspur/project/RoboTwin
export PYTHONUTF8=1
PY=/mnt/sda/conda_envs/RoboTwin/bin/python

$PY scripts/occluded_socket_demo.py paired --seed 0 --gpu 0 --record --output result/occluded_socket_drawer/release_pair
$PY scripts/occluded_socket_demo.py batch --seeds 0:5 --gpu 0 --output result/occluded_socket_drawer/development_5
$PY scripts/occluded_socket_demo.py batch --seeds 100:110 --gpu 0 --output result/occluded_socket_drawer/test_20/gpu0
$PY scripts/occluded_socket_demo.py batch --seeds 110:120 --gpu 1 --output result/occluded_socket_drawer/test_20/gpu1
# 上一条产生 40 次渲染启动失败；以下为完整分片原样迁移重试。
$PY scripts/occluded_socket_demo.py batch --seeds 110:120 --gpu 0 --output result/occluded_socket_drawer/test_20/gpu1_retry_on_gpu0

$PY scripts/occluded_socket_demo.py episode --policy H --load-condition HIGH --seed 0 --gpu 0 --record --variant partial_helper --output result/occluded_socket_drawer/supplementary/partial_helper
$PY scripts/occluded_socket_demo.py episode --policy H --load-condition HIGH --seed 0 --gpu 0 --record --variant right_handoff --output result/occluded_socket_drawer/supplementary/right_handoff
$PY scripts/occluded_socket_demo.py oracle --right-only --gpu 0 --record --output result/occluded_socket_drawer/supplementary/oracle_release
$PY scripts/occluded_socket_probe.py --gpu 0 --output result/occluded_socket_drawer/supplementary/camera_validation
$PY scripts/occluded_socket_demo.py audit --gpu 0 --output result/occluded_socket_drawer/supplementary/audit_release

$PY scripts/summarize_occluded_socket.py result/occluded_socket_drawer/release_pair --compose
$PY scripts/audit_occluded_evidence.py result/occluded_socket_drawer/release_pair
$PY scripts/summarize_occluded_socket.py result/occluded_socket_drawer/test_20
$PY scripts/audit_occluded_evidence.py result/occluded_socket_drawer/test_20
$PY experiments/occluded_socket_drawer/test_contracts.py
```

上述列表按用途排列，非时间顺序；实际先开发再冻结测试。`development_5` 的源码以各自保留快照为准。复现当前冻结版本请使用新目录，已验证设备为 GPU 0：

```bash
cd /home/inspur/project/RoboTwin
export PYTHONUTF8=1
PY=/mnt/sda/conda_envs/RoboTwin/bin/python
RUN=result/occluded_socket_drawer/reproduce_$(date +%Y%m%d_%H%M%S)

$PY experiments/occluded_socket_drawer/test_contracts.py
$PY scripts/occluded_socket_demo.py audit --gpu 0 --output "$RUN/audit"
$PY scripts/occluded_socket_demo.py oracle --right-only --gpu 0 --headless --record --output "$RUN/oracle"
$PY scripts/occluded_socket_demo.py drawer --load-condition HIGH --gpu 0 --headless --record --output "$RUN/drawer"
$PY scripts/occluded_socket_probe.py --gpu 0 --output "$RUN/cameras"
$PY scripts/occluded_socket_demo.py episode --policy S --load-condition HIGH --seed 0 --gpu 0 --headless --record --output "$RUN/single_S"
$PY scripts/occluded_socket_demo.py paired --seed 0 --gpu 0 --headless --record --output "$RUN/paired"
$PY scripts/summarize_occluded_socket.py "$RUN/paired" --compose
$PY scripts/audit_occluded_evidence.py "$RUN/paired"
$PY scripts/occluded_socket_demo.py batch --seeds 100:120 --gpu 0 --headless --output "$RUN/test"
$PY scripts/summarize_occluded_socket.py "$RUN/test"
$PY scripts/audit_occluded_evidence.py "$RUN/test"
for variant in partial_helper right_handoff; do
    $PY scripts/occluded_socket_demo.py episode --policy H --load-condition HIGH --seed 0 --gpu 0 --headless --record --variant "$variant" --output "$RUN/$variant"
done
```

`--budget` 可改预算，`--no-record` 关闭录像，`--no-headless` 在有显示服务时启用 viewer；本次只验证 headless。运行入口用 `result.json` 表达任务成功/失败，退出码不是任务成功的充分判据。不要将 `supplementary`、oracle 或开发记录和冻结独立测试混在一个研究均值里。

## 8. 视频、图像、日志与限制

所有产物根目录：`/home/inspur/project/RoboTwin/result/occluded_socket_drawer/`（被仓库原有规则忽略，文件真实保存在本机）。

| 产物 | 根目录下的路径 |
|---|---|
| 四组对照视频 | `release_pair/summary/four_way.mp4` |
| 四段完整原视频 | `release_pair/s0_LOW_H/execution.mp4`、`s0_LOW_S/execution.mp4`、`s0_HIGH_H/execution.mp4`、`s0_HIGH_S/execution.mp4`；后面三者同属 `release_pair/` |
| 四组表/图 | `release_pair/summary/results.csv`、`results.png`、`results.pdf`、`summary.json` |
| 全部 120 次测试表/图 | `test_20/summary/results.csv`、`results.png`、`results.pdf`、`summary.json` |
| 初始、助手和持物自观察原图 | `supplementary/camera_validation/checkpoint_head.png`、`checkpoint_right.png`、`left_0_left.png`、`right_0_right.png`、`right_1_right.png` |
| 相机探针数据 | `supplementary/camera_validation/detections_and_calibration.json` |
| 证据审计 | `release_pair/evidence_audit.json`、`test_20/evidence_audit.json` |
| GPU 失败原始日志 | `test_20/gpu1/s110_LOW_H/error.log` 等全部 40 个 run |
| 替代方案录像/日志 | `supplementary/partial_helper/`、`supplementary/right_handoff/` |

每个初始化成功 run 含 `config.json`、`provenance.json`、`source/`、`initialization_gt.json`、`initial_state_gt.json`、`final_state_gt.json`、`camera_calibration.json`、`controller.jsonl`、`perception.jsonl`、`joints.jsonl`、`evaluator_gt.jsonl`、`contacts_gt.jsonl`、`result.json`、checkpoint/有效图像及有效观测 NPZ。未开始 scene 的失败保留 result、错误、配置、源码和 provenance，没有伪造初始图像/状态。

视频为真实渲染，含 overview/head/双腕与策略、条件、物理时间、定位和任务状态。合成从 t=0 对齐，全部 10 fps、1×，先结束的保持最后帧；不分别变速。编码为 H.264，原片保留。非整 0.1 s 的终止画面放在下一录像帧，显示起点偏差小于一帧，加上该帧自身播放时间，媒体时长可比物理时间多不到 0.2 s。合成保留全部原始终止帧，时长 15.4 s；精确完成时间以 JSON 为准。`video_alignment.json` 保存实际 FFmpeg 参数，`video_validation.json` 保存实际解码校验。

本版限制：宽间隙、水平平移随机化、理想 RGB-D、固定夹持标定与程序化标记；13/20 测试实例初始右腕已能定位；视觉目标一旦通过门限便用于安装，没有持续力控纠偏；离散规划与采样接触日志不是连续空间碰撞证明；未实现通用暂放重抓恢复、P2 自动 H/S 选择、抽屉行程扫描、真实机器人测试、噪声鲁棒性实验或统计显著性分析；右臂接手仅验证一个失败路线。结果只说明这两种具体 H/S 控制方案和这些测试实例的相对收益。

P0 已完成，P1 已完成独立测试与有限替代检查；P2 未执行。工程验收证据充分支持“能运行、视觉驱动安装、真实接触操作、双臂并行、四组证据完整”，研究数据不足以支持预期的一般性收益反转。
