# Occluded socket + drawer: checkpoint decision experiment

任务始终为“将右侧模块正确装入定位座，并把左侧抽屉关闭”。这是传统 RGB-D 视觉、有限状态机、机器人运动学和真实接触控制的研究 demo，不训练模型，不调用在线大模型。研究假设不是测试断言；成功或失败、是否反转均由保存的执行数据报告。

已实际执行：[完整运行报告](REPORT.md)。权威四组录像在 `result/occluded_socket_drawer/release_pair/`：seed 0 的 LOW H/S 为 9.352/7.788 s，HIGH H/S 为 15.220/12.456 s，全部成功。冻结后独立测试共保留 120 次尝试：80 次物理执行成功、40 次 GPU 1 初始化失败。13/20 种子在 0.1 s 就从右腕定位；唯一严格符号反转的 LOW 优势只有 0.004 s，不支持预期机制成立。完整失败分母、替代策略、图表、视频与复现命令均在报告中。

## 环境和入口

- 仓库基线：`2f84b98bd64d8d1f3d8be79462b6b216b81820ff`，开始时无未提交修改，无适用 AGENTS.md。
- Python：`/mnt/sda/conda_envs/RoboTwin/bin/python`，3.10.21。
- SAPIEN 3.0.0b1 / MPLib 0.2.1 / OpenCV 4.10.0 / NumPy 1.26.4 / SciPy 1.10.1。
- 复用现有 `aloha-agilex` URDF、RoboTwin `Base_Task` 和 `Robot`、原有驱动参数与质量初始化。没有重新安装环境或改动既有任务。
- 默认单实例、GPU 0 渲染、CPU PhysX/运动学/碰撞检查。本地 beta 的旧 SapienRenderer 初始化路径会先锁定 GPU 0；本次 GPU 1 尝试在 scene 创建前失败，全部保留。复现请使用已验证的 `--gpu 0`，其他设备尚未验证可用。没有为并行更改冻结版本或升级依赖。
- FFmpeg 使用已安装的 imageio-ffmpeg 所带程序。所有输出目录必须尚不存在，以避免覆盖。

```bash
cd /home/inspur/project/RoboTwin
export PYTHONUTF8=1
PY=/mnt/sda/conda_envs/RoboTwin/bin/python

$PY scripts/occluded_socket_demo.py audit --gpu 0 --output result/occluded_socket_drawer/my_audit
$PY scripts/occluded_socket_demo.py oracle --right-only --gpu 0 --record --output result/occluded_socket_drawer/my_oracle
$PY scripts/occluded_socket_demo.py drawer --load-condition HIGH --gpu 0 --record --output result/occluded_socket_drawer/my_drawer
$PY scripts/occluded_socket_demo.py episode --policy H --load-condition HIGH --seed 0 --gpu 0 --headless --record --output result/occluded_socket_drawer/my_H
$PY scripts/occluded_socket_demo.py paired --seed 0 --gpu 0 --headless --record --output result/occluded_socket_drawer/my_pair
$PY scripts/summarize_occluded_socket.py result/occluded_socket_drawer/my_pair --compose
$PY scripts/occluded_socket_demo.py batch --seeds 0:5 --gpu 0 --output result/occluded_socket_drawer/my_development
# Freeze code/config before independent tests. Keep every failure.
$PY scripts/occluded_socket_demo.py batch --seeds 100:120 --gpu 0 --output result/occluded_socket_drawer/my_test
$PY scripts/summarize_occluded_socket.py result/occluded_socket_drawer/my_test
$PY experiments/occluded_socket_drawer/test_contracts.py
$PY scripts/audit_occluded_evidence.py result/occluded_socket_drawer/my_test
```

`--budget` changes the physical time budget. `--no-headless` opens a viewer when a display is available. `--no-record` retains images/JSON logs without video. `--right-only` isolates stage-1 visual/oracle installation. `--variant partial_helper` tries half the drawer work before helping; `--variant right_handoff` attempts to transfer unfinished drawer work to the right arm after installation. These alternatives are logged separately from baseline results.

## 数据边界

| Component | Responsibility / allowed state |
|---|---|
| `envs/occluded_socket_drawer.py` | Procedural physical scene, private random insert translation, checkpoint initialization |
| `observation.py` | Real RGB + scalar optical depth, camera intrinsics, robot FK/mount calibration; no segmentation/actor target pose |
| `perception.py` | RGB HSV components, metric-size candidates, four-point geometry and depth consistency |
| `motion.py` | Robot FK/IK, joint/Cartesian trajectories, sampled collision checking against robot, known fixed fixtures and image-estimated drawer |
| `controller.py` | Independent H/S queues; sees no environment, seed, load label, target actor or evaluator |
| `evaluation.py` | Private ground truth, release/stability/simultaneous success checks; never feeds target/error to policy |
| `runtime.py` | Single shared physics loop, explicit oracle injection, immutable run records |

原有 `grasp_actor` / `place_actor` / functional-point 辅助函数会读取 Actor GT，本 demo 的视觉控制不调用它们。隐藏槽位、随机标记不进入规划世界；移动抽屉障碍位置来自图像。碰撞引擎保留所有真实几何。MPLib 排除仅涉及未使用后臂、底盘轮子内部的碰撞结果，所有涉及活动前臂的碰撞继续检查。

## 视觉与坐标

`T_A_B` 将 B 中的点变换到 A，单位米，矩阵运算统一。SAPIEN camera 的轴为前/左/上；OpenCV 为右/下/前，具体变换保存在 `geometry.py`。SAPIEN pose 四元数使用 wxyz；本模块主要使用 4×4 矩阵。深度为 `-Position[...,2]`，不向策略开放 Position 的三维真值通道。

四个彩色方块与内部随机插槽刚性连接，基于 RGB 检测；由光学深度还原到相机系，再用腕部 FK 和 URDF 固定安装标定变换到世界系。初版只随机平移，已知水平姿态，所以估计平移并检查完整四点几何，不假装恢复未随机化的旋转。

```
T_world_camera_cv = T_world_ee(q) @ T_ee_camera_sapien @ T_sapien_cv
T_world_ee_goal = T_world_socket_estimated @ T_socket_module_goal @ inverse(T_ee_module)
```

标记检测要求完整四点、足够像素、合理实际尺寸、深度有效、几何/重投影误差合格及连续两帧一致。阈值见 YAML；这些是代理质量门限，不是概率。抓取变换来自模块与把手设计标定，初始化后测量值仅写入 GT 记录，从不刷新控制标定。模块靠夹爪摩擦接触持有，无固定连接约束。

参考：[SAPIEN 相机约定](https://sapien-sim.github.io/docs/user_guide/rendering/camera.html)、[OpenCV 投影坐标约定](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)。实际 beta API 以本地源码与运行结果为准。

## 场景、策略和时间

场景使用程序化刚体。定位座有底板和四面低墙，内部是真实空腔；外壳有侧壁，未关闭碰撞。模块为宽底板加把手，另有实体临时架。抽屉只有一个无弹簧的滑动关节，由夹爪接触推动，无后台驱动。LOW=已关，HIGH=打开 22 cm；其余随机槽位和右侧初始状态一致。开发中尺寸/布局的变更与原因见 PROGRESS.md。

这是**冷启动中途 checkpoint**：两臂被初始化到公开固定姿态，模块建立夹持接触后用物理稳定 3 s；这段不计入决策时间，四组规则完全相同。策略此前没有观察历史，从 t=0 起才获得图像。之后禁止设置物体 pose、机器人 qpos 或抽屉 qpos；相机 pose 更新只跟随机器人安装点。

- H：右臂持物，左臂移动到固定候选视点；信息足够后，右臂入座，左臂去推抽屉，两队列独立执行。
- S：左臂立即做抽屉，右臂尝试持物侧移/转腕观察，避免强制暂放。开发探针验证了持物右腕观察；实际运行允许更早获得的 head 或另一腕图像直接触发安装。
- 所有任务角色持续获得左右腕和明确允许的 head RGB-D；录像 overview 不进入策略。
- 速度/加速度、视觉门限、安装步骤、碰撞处理、成功标准对 H/S 相同。候选最多三次；规划碰撞时最多一次固定上方绕行，仍失败则明确失败，不回退 oracle。

两臂每 tick 更新各自的关节目标，随后只执行一次 `scene.step()`。轨迹队列不按动作列表对齐，不要求先回原点；成功时包括安全退出。主指标是实际推进的物理步数 × dt（dt=0.004 s），包含所有执行、观测移动、等待、重试和绕行。CPU 规划、渲染和检测的 wall-clock 与物理时钟分开；它们发生在物理 tick 边界，不模拟计算延迟。这是机制实验的明确时间约定，不能将物理结果当作机器人真实 wall-clock 吞吐率。

冻结版本采用 C2 三次样条并在物理采样网格上缩放轨迹，统一检查关节速度/加速度上限。8 项 CPU 契约测试通过，包括拒绝路径不执行和模块离座后撤销成功；真实仿真证据另由 `scripts/audit_occluded_evidence.py` 审计。

## 结果与证据

每个 run 保存完整 config、源文件快照及 SHA256、commit、命令、环境版本、实际初始状态摘要、独立 `controller/perception/joints` 与 `evaluator_gt/contacts_gt` JSONL、结果 JSON、关键帧与可选视频。失败的整体完成时间为 null，不参与成功条件下的均值，但在 Success@Budget 分母中保留。H/S 配对首先检查实际初始状态 hash，不只比较 seed。

视频来自真实仿真；四窗显示 overview/head/左右腕及物理时间、策略、状态。四组视频从共同 t=0 对齐、10 fps、1×速度；先结束的画面保持最后帧，不分别调速。统计图仅使用 result.json 的真实执行点。最终已执行命令与结果见 REPORT.md，开发与 oracle 数据单独列出。

## 限制

这是宽松装配间隙、平移随机化、程序化标记和理想仿真 RGB-D 的实验，没有相机噪声或精密接触力控制。碰撞规划按离散轨迹采样，实体接触由 PhysX 处理；它不证明连续空间的全局最优性。抓取校准有实际接触滑移误差，独立评测器记录最终偏差。

head 的自然可见性可能让额外腕部观察提早结束，必须报告这种任务退化。两套具体方案及少数替代检查不能支持全局最优或一般性收益反转结论。临时架实体已提供；当前验证的是更便宜的持物观察路径，尚未实现通用暂放后重抓恢复。P2 自动选 H/S 不属于本版。
