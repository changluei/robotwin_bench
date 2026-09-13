# 观察几何校准：实际运行报告

日期：2026-09-10。根目录：`/home/inspur/project/RoboTwin`。

**本轮没有找到足以进入 HIGH 实验的帮助观察局部收益。** 原 v1 的直接执行和小幅持物自观察都更快；新增实体遮挡板后，一度由较大的自观察路线得到 H 优势，但补测可行的小幅侧移/转腕后该优势消失。没有通过禁用这条路线保留结论。训练、P2、20 种子规模评测和新配置 HIGH 均未运行。

先完成并交付了 [v1 审计报告](V1_AUDIT.md)，随后才运行 v2。本文补充完整 LOW 校准、失败原型及四次新种子核验；原 v1 的代码、报告、录像、全部结果和负面结论未覆盖。

## 1. v1 审计结论

被动记录复跑开发种子 0–4 和已有代表种子 104：seed 0 四组，其余只跑 LOW H/S，共 14 次，全部成功。seed 0 的实际初始状态 hash、信息时间及各完成时间与原 release_pair 逐项一致。t=0 与首次接受信息时的 head/左右腕 RGB-D、相机矩阵、qpos/qvel、FK 和累计运动量均已保存。

H/S 共用感知门限，获得有效信息后同 tick 取消剩余观察并开始安装。没有发现已经看清却强制走完观察流程。seed 0 各观察轨迹均提前取消，没有到达预定观察终点，故实际“视点到达时间”是 null，不能填规划时长。

H 的 LOW 3.3 s 与 HIGH 5.2 s 差异来自**观察路径受到打开抽屉阻挡**：LOW 直接走 observe_0；HIGH 在 0 s 拒绝该路径（直接及避让后均与图像估计抽屉前板冲突），0.004 s 改走 observe_1，并采用上方避让。获得信息前左末端实际路径从 0.1963 m 增至 0.3455 m，关节累计路径从 3.8567 rad 增至 5.5629 rad。

两组机器人初始 qpos/qvel 完全相同，初始快照唯一差异为抽屉 qpos；H 没有提前推动抽屉，没有右臂预操作或同步等待。信息到右侧预接近结束均为 1.212 s。因此额外 1.9 s 不能全归于左臂工作机会成本：v1 同时改变了工作量和观察通道。

| v1 seed 0 | 信息时间/来源 | 入座准备完成 | 右侧稳定完成 | 右侧退出结束 | 左臂开始抽屉 |
|---|---|---:|---:|---:|---:|
| LOW H | 3.300 / left | 4.512 | 7.836 | 9.352 | 无工作 |
| LOW S | 1.600 / head | 2.948 | 6.272 | 7.788 | 无工作 |
| HIGH H | 5.200 / left | 6.412 | 9.736 | 11.252 | 5.200 |
| HIGH S | 1.600 / head | 2.948 | 6.272 | 7.788 | 0.000 |

“入座准备完成”统一使用共享 install_preapproach 轨迹完成的物理边界，下一动作即入座，包含看清后的接近/恢复运动。它不是首次检测时间，也不是额外静止等待。右侧成功仍由独立评测器验证释放、位姿、速度和持续稳定；退出另外计时。

种子 1、4、104 在 0.1 s 即由右腕定位，H 此前右臂末端位移为 0，S 仅移动约 0.0284 mm、旋转 0.000481 rad。seed 104 的 t=0 原图已显示四角点和槽口；0.1 s 主要是第二帧一致性门限。旧 13/20 结论不是由完成了有意义的观察运动造成的。

另发现初始化局限：v1 仅检查抓取相对平移 <25 mm，没有检查转角和近零速度。seed 0 的持物姿态偏差约 3.81°，瞬时角速度约 1.64 rad/s，夹持接触仍有求解器抖动。3 s 初始化稳定过程不等于完全静止夹持；最终入座后的稳定成功条件仍实际通过。原报告保留，这一补充写入本轮审计。

## 2. 比较方法和新基线

继续使用原 RoboTwin 环境、aloha-agilex、GPU 0、SAPIEN 3.0.0b1。未安装依赖、未改核心物理循环。候选只使用公开工装、机器人 FK/当前相机安装位置和固定参数；不读随机种子、隐藏目标 Actor/关键点、GT 分割或 evaluator 误差。实体碰撞始终开启，新遮挡板同时加入公开规划碰撞模型。

H 与 S 共用原感知、关节/Cartesian 速度及加速度、安装轨迹、碰撞处理与成功条件。持续接收 head 和双腕图像，有有效信息就取消剩余观察。完整时间包含恢复入座姿态和实际暂放/重抓，CPU 规划/渲染 wall time 单列，不混入物理时间。录像均真实渲染、10 fps、1×。

LOW 中实际尝试了多个左腕观察位置、就地相机转向/转腕、右臂小幅平移/转腕、中等和大幅移动、较远相机视点及暂放重抓。失败候选保留。搜索并未覆盖连续视点空间或所有 IK/回程路径；“最好”仅指本轮已找到并执行的完整方案。

- **direct_live**：不规划专门观察动作。先采集 t=0/0.1 s 两个实际初始帧，再向公开固定工装的名义预接近位置正常运动；途中允许三路相机提供有效目标。真正入座必须基于有效视觉估计，无法定位时保持失败，不盲插、不回退 GT。接受有效目标后沿用 v1 的目标提交规则，不是持续力控视觉伺服。
- **direct_initial_only**：采用同样的两帧初始窗口及粗预接近，只接受该初始窗口的有效估计，之后不更新目标。后续图像照常采集/记录，策略按这个明确的诊断条件不采用；没有用同一张图片伪造两帧。
- **putdown**：真实向固定架放置、松爪、空手观察、重抓、抬起再安装；若释放前已看清，取消暂放直接安装；若已经释放，则完成取回，避免将未持物状态当作安装就绪。抓取恢复使用已知架面和公开抓取标定，不查询模块真值。
- **putdown_cycle_validation**：单独强制完成一次架上释放/重抓，只用于验证接触能力，不能把它的耗时作为合理 S 的观察成本。正常比较用前一自适应版本。

“initial_only”是刻意限制信息更新的诊断；“cycle_validation”是明确强制动作的能力验证。它们与正常 H/S 和自适应暂放分别标记，未用于宣称 H 优势。

## 3. v1 LOW：专门观察没有局部时间收益

seed 0 代表性实测如下，时间单位 s，全部包含真实执行。

| 方案 | 信息时间/来源 | 入座准备 | 右侧稳定 | 全部含退出 | 结果 |
|---|---|---:|---:|---:|---|
| H 原候选 | 3.300 / left | 4.512 | 7.836 | 9.352 | 成功 |
| H 已找到最佳 helper_high | 2.000 / left | 3.212 | 6.536 | **8.048** | 成功 |
| H 原相机位置附近转向 | 2.300 / left | 3.516 | 6.840 | 8.356 | 成功 |
| S 小幅后移 self_small_back | **0.300 / head** | 1.568 | 4.892 | 6.408 | 成功 |
| S 小幅侧移+转腕 self_small_yaw | 0.500 / head | 1.732 | 5.056 | 6.568 | 成功 |
| S 侧移 self_small_x | 0.600 / head | 1.916 | 5.240 | 6.756 | 成功 |
| S 较大侧移 self_large_x | 0.600 / head | 1.792 | 5.116 | 6.632 | 成功 |
| S 原较远视点 | 1.600 / head | 2.948 | 6.272 | 7.788 | 成功 |
| 直接执行 direct_live | 1.000 / head | **1.356** | **4.680** | **6.196** | 成功 |
| 仅初始估计 | null | null | null | null | 35 s 感知超时 |
| 自适应暂放候选 | 0.700 / head | 2.112 | 5.436 | 6.952 | 释放前已看清，取消暂放 |
| 强制暂放重抓能力验证 | 0.700 / head | 13.336 | 16.660 | 18.184 | 成功，仅诊断 |

小幅后移只需约 11 mm 实际末端运动就获取信息；直接执行的信息较晚，却已完成大部分正常预接近，因此整体更快。不能以 0.3 s 对 1.0 s 的检测时间直接判定前者更好。

强制暂放验证确有模块/实体架接触，记录时间范围 3.204–9.504 s；随后实际重抓抬起，11.736 s 完成持物恢复，再安装成功。没有通过设置最终状态或固定连接完成取放。

开发种子 1 和 2 进一步区分信息来源：

| 种子 | H 优化 | S 小幅后移 | 直接执行 | 仅初始估计 |
|---|---:|---:|---:|---|
| 1 | 6.340 | 6.340 | 6.340 | 成功，6.340 |
| 2 | 8.096 | 6.444 | 6.320 | 感知超时，null |

种子 1 的四个方案都直接利用 0.1 s 的右腕初始信息；种子 0/2 的初始信息不足，正常操作中的后续视觉有价值。这里保留了全部可见/不可见实例，没有筛掉容易实例。

## 4. v2 实体几何原型与失败

三个配置均独立保存，未覆盖前一个：

| 配置 | 物理改动 | 实际结果及处理 |
|---|---|---|
| A：v2_probe | 模块 90×100 mm、开口 118×128 mm，维持每侧 14 mm 间隙；前壁 x=[.05,.31]、y=[−.196,−.184]、z=[.75,1.02] | 初始右夹指与前壁接触，3 次都失败；不用于观察收益比较 |
| B：v2_clearance | 前壁前移 4 cm，顶部提高至 1.08 m，保留加宽模块 | 9 次都失败；加宽使抓取姿态偏差增至约 27.6°，且完整前壁与右肘插入运动发生接触，存在操作/标定混杂 |
| C：v2_baffle | 恢复原模块/槽位/标记/抓取尺寸；只加实体窄挡板 x=[.01,.18]、y=[−.236,−.224]、z=[.75,1.08] | 单独 oracle 首先成功 6.036 s；随后进行 LOW 视觉路线比较 |

A/B 的程序初始化返回且保存了快照，但初始物理条件不合格，不能据此说场景通过物理验收。原始失败和接触冲量保留；没有关闭碰撞、按策略改速度或改变视觉门限解决它们。

C 的物理动机是挡住 head 到槽口的实际斜视射线，同时留出右肘经过挡板右缘的空间，左侧观察及上方插入通道保持开放。head 的实际初始图像中，目标槽口中心被实体板遮挡，并非只隐藏彩色标记。所有腕部视线与可行动作仍允许。相机安装、机器人初始关节状态、槽位随机范围、物理 dt 和阈值均与 v1 相同。

## 5. v2 C 的局部优势被更好的 S 路线消除

| seed 0 方案 | 信息时间/来源 | 入座准备 | 右侧稳定 | 全部含退出 | 结果 |
|---|---|---:|---:|---:|---|
| H helper_camera_here | 2.300 / left | 3.516 | 6.840 | **8.356** | 成功 |
| H helper_near | 2.900 / left | 4.112 | 7.436 | 8.952 | 成功 |
| S 原较大视点 | 5.400 / right | 11.856 | 15.184 | 16.700 | 成功 |
| S 自适应暂放、观察、重抓 | 10.800 / right | 22.364 | 25.688 | 27.212 | 成功，20.792 s 恢复持物 |
| S 小幅侧移+转腕 | **0.500 / right** | **1.732** | **5.056** | **6.568** | 成功，消除 H 优势 |
| S 纯侧移 6/8/10/12 cm、较大侧移及其他若干视点 | null | null | null | null | 候选无有效定位，全部保留 |
| 直接执行 | null | null | null | null | 正常预接近后仍未定位，35 s 超时 |

如果只用前两个 S 方案，会误判 H 有显著局部收益。补测 self_small_yaw 后，这个结论不成立。其公开候选目标为右臂侧移 3 cm、世界 z 轴转动 0.3 rad，但**实际只执行了约 8.74 mm 平移、0.108 rad 旋转就看清并取消剩余观察**；不能把全部候选运动计入它的代价。H 此前左末端约移动 90.0 mm、累计转动 1.173 rad。

实体挡板确实使直接执行失败，也把 S 的信息来源从 head 改为 right；但它没有消除可行的便宜持物自观察。S 的新有效估计误差为约 0.821 mm，H 约 1.001 mm，实际安装均稳定成功。因此不能通过提高共享视觉门限来把已能准确安装的 S 路线排除。

已找到的最好单候选路线为 H=helper_camera_here、S=self_small_yaw。候选空间和恢复路径优化未穷尽，不声称全局最优。由于便宜的 S 消除了 H 局部优势，**没有运行新几何 HIGH，也没有为寻求反转继续调整挡板**。

## 6. 冻结后只做四次新种子 LOW 核验

100..119 已参与先前分析，未复用为本轮独立测试。完成开发搜索后保存 [FROZEN_V2_LOW.json](FROZEN_V2_LOW.json)，固定 C 配置、上述两条路线、35 s 预算及新种子 300、301，再运行四次 LOW。冻结后源码未变，未按新种子结果调参。此处 n=2，只能作为有限核验，不是大规模统计证据。

| 新种子/方案 | 信息时间/来源 | 入座准备 | 右侧稳定 | 整体完成 | 成功 |
|---|---|---:|---:|---:|---|
| 300 H | 0.100 / right | 1.384 | 4.708 | 6.220 | 是 |
| 300 S | 0.100 / right | 1.384 | 4.708 | 6.220 | 是 |
| 301 H | 2.300 / left | 3.516 | 6.840 | 8.356 | 是 |
| 301 S | 0.500 / right | 1.736 | 5.060 | 6.572 | 是 |

两个 H/S 配对的实际初始状态 hash 均完全匹配。种子 300 初始右腕可见，没有删除或重采样；它表明当前遮挡仍不覆盖全部随机实例。四次首次有效估计误差均小于 0.986 mm，没有记录到非预期机器人接触。S 在 seed 301 更快 1.784 s，seed 300 相同。

## 7. 统计口径、完整失败与检查

统计分别提供：程序初始化错误、初始化完成后的任务成功率、任务完成占所有尝试的比例，以及运行结果记录是否完整。`summary/attempts.csv` 用实际 config 内容 SHA256、seed、condition、policy_id、attempt_id 和绝对路径标识对应关系；历史声明 ID 另存 declared_config_id。重跑不覆盖，也不按 seed 合并删除。

| 本轮类别 | 尝试 | 程序初始化错误 | 任务成功 | 任务失败 |
|---|---:|---:|---:|---:|
| v1 原样审计 | 14 | 0 | 14 | 0 |
| v1 LOW 路线校准/确认，含强制能力验证 1 次 | 29 | 0 | 23 | 6 |
| v2 A/B 失败原型 | 12 | 0 | 0 | 12 |
| v2 C LOW 开发候选 | 17 | 0 | 5 | 12 |
| v2 C oracle，单独列出 | 1 | 0 | 1 | 0 |
| 冻结后新种子 LOW | 4 | 0 | 4 | 0 |
| 全部运行清单 | **77** | **0** | **47** | **30** |

77/77 次运行都有结果记录；47/77 次完成各自任务。这是异质开发/诊断的完整清单，不能当作一个策略的成功率。A/B 有明确物理初始化问题，已单列，不能混入有效几何的策略收益。逐 config/policy 的初始化条件任务成功率和全部尝试完成比例见 summary.json。

真正的新种子核验：每种方案初始化完成 2/2、任务成功 2/2、任务完成占全部尝试 2/2；合计 4/4。旧 v1 的 40 次 GPU 1 启动失败仍完整保留：旧物理任务成功 80/80、初始化失败 40/120、任务完成占全部尝试 80/120。本轮全用 GPU 0，没有把旧启动失败藏入或排除出旧分母。

检查结果：

- 22 个 v1 保存源文件、567 个旧产物文件逐个 SHA256 校验无变化；新种子运行前后冻结源码也一致。
- 原有 8 项 CPU 契约与新增 3 项控制契约通过；新增检查覆盖初始限定/后续信息差异、仍持物时及时取消暂放、完成重抓抬起前禁止安装。
- 77 次离线动作顺序审计无错误，所有失败的整体完成时间为 null。
- 47 份实际首次有效图像（包括获取信息后仍操作失败的诊断）遮黑 RGB 后均拒绝定位；有视觉目标的实际安装变换与观测坐标链完全一致。此项是离线感知负例，不声称另跑了 47 次机器人任务。
- 接触日志每 5 tick 采样、过滤小于 0.0001 N·s 的冲量，不是连续无碰撞证明。A/B 的真实异常接触明确保留；有效 C 对比和四次新种子核验没有记录到非预期机器人接触。

## 8. 交付路径与实际命令

新产物根目录：`/home/inspur/project/RoboTwin/result/occluded_socket_calibration/`。

| 产物 | 根目录内路径 |
|---|---|
| v1 t=0/首次定位三路原图与相机/关节状态 | `v1_audit_001/<run>/audit_frames/initial/`、`first_information/` |
| v1 相机拼图 | `v1_audit_001/summary/seed0_actual_cameras.png`、`seed104_actual_cameras.png` |
| 全部尝试 CSV / JSON | `summary/attempts.csv`、`summary/summary.json` |
| 独立 GT 证据审计 | `physics_evidence_audit.json` |
| v2 开发种子对照视频/原图拼图 | `comparison_v2_seed0/comparison.mp4`、`actual_cameras.png` |
| v2 新种子 301 对照视频/原图拼图 | `comparison_v2_seed301/comparison.mp4`、`actual_cameras.png` |
| 最终 H 原视频 | `v2_local_rotation_001/s0_LOW_helper_camera_here/execution.mp4` |
| 最终 S 原视频 | `v2_self_refinement_001/s0_LOW_self_small_yaw/execution.mp4` |
| 真正暂放/重抓原视频 | `v2_baffle_search_001/s0_LOW_putdown/execution.mp4` |
| 新种子配对日志/原视频 | `fresh_low_s300/`、`fresh_low_s301/` |
| 失败原型全部记录 | `v2_geometry_pilot_001/`、`v2_clearance_search_001/` |

每个成功初始化的 run 含 config、原 v1 provenance/source、初始/最终 GT 快照、controller/perception/joints/evaluator_gt/contacts_gt 日志，以及本轮 audit_summary、audit_milestones、实际相机数据；本轮运行源码快照另存 instrumentation_source/provenance。最初的少数 v1 被动审计运行早于新增 instrumentation_source 记录功能，原 v1 源码快照与状态/时间对应仍完整。

以下主要命令均已实际执行，历史输出存在，复现时改成新目录：

```bash
cd /home/inspur/project/RoboTwin
export PYTHONUTF8=1
PY=/mnt/sda/conda_envs/RoboTwin/bin/python

$PY scripts/calibrate_occluded_socket.py audit-suite --gpu 0 --output result/occluded_socket_calibration/v1_audit_001
$PY scripts/calibrate_occluded_socket.py route-suite --scene v1 --seed 0 --gpu 0 --record --routes self_small_x,direct_live,putdown_cycle_validation --output result/occluded_socket_calibration/low_pilot_001
$PY scripts/calibrate_occluded_socket.py oracle --scene v2_baffle --seed 0 --gpu 0 --record --output result/occluded_socket_calibration/v2_baffle_oracle_001
$PY scripts/calibrate_occluded_socket.py route-suite --scene v2_baffle --seed 0 --gpu 0 --record --routes helper_camera_here,helper_wrist_turn,self_camera_here --output result/occluded_socket_calibration/v2_local_rotation_001
$PY scripts/calibrate_occluded_socket.py route-suite --scene v2_baffle --seed 0 --gpu 0 --record --routes self_small_yaw,self_mid_x08,self_mid_x10,self_mid_x12,self_large_x --output result/occluded_socket_calibration/v2_self_refinement_001
$PY scripts/calibrate_occluded_socket.py route-suite --scene v2_baffle --seed 300 --gpu 0 --record --routes helper_camera_here,self_small_yaw --output result/occluded_socket_calibration/fresh_low_s300
$PY scripts/calibrate_occluded_socket.py route-suite --scene v2_baffle --seed 301 --gpu 0 --record --routes helper_camera_here,self_small_yaw --output result/occluded_socket_calibration/fresh_low_s301
$PY scripts/summarize_occluded_calibration.py result/occluded_socket_calibration
$PY scripts/audit_occluded_calibration_physics.py result/occluded_socket_calibration
$PY experiments/occluded_socket_calibration/test_contracts.py
```

其余实际候选/确认运行的完整 argv 在各 run provenance 与 console 日志中。一次完整的精简复现见 [README.md](README.md)。

## 9. 判断与未解决限制

v1 应保留为直接操作即可自然获取信息的简单对照。v2 C 可保留为 head 遮挡对照，但便宜的持物右腕路径和新的初始可见实例仍存在，不支持继续用当前几何检验 HIGH 下的机会成本反转。没有以“出现反转”作为验收断言，也没有为了保留初步 H 优势禁止小幅转腕。

本轮实现的是固定架、公开标定下的真实暂放重抓，不是通用任意位置恢复。连续视点、所有 IK 解及回程路径未穷尽；接触求解器抖动、理想 RGB-D、宽松装配间隙和仅平移随机化仍限制结论。v1 的右臂接手抽屉路线失败与混合策略范围限制仍保留，本轮未在新几何 HIGH 上复验。P2、训练、大规模新测试和真实机器人实验均未执行。

可见性结论针对实际图像及本轮共享的四点 RGB-D 检测器，不证明更强的部分标记/轮廓算法无法利用剩余像素恢复目标。不能把检测器拒绝定位直接等同于物理世界完全没有信息。
