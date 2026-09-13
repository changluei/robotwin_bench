# 持物模块底部安装键：几何审计与停止结论

**结论：模块自身的背面遮挡成立，左臂帮助观察可行，但本轮没有发现 H 的局部收益。停止该候选，不进入 LOW/HIGH、训练、P2 或扩大测试。**

同版本、同初始物理状态的开发实例 400 中，合理的 S 将底键展示给 head，右侧稳定完成为 **8.212 s**；H 使用主动左腕为 **8.380 s**。两者都完成真实接触入座。这个单实例结果不证明 S 全局最优或跨实例占优；它没有给出继续构造 H 机会成本实验的依据。搜索和失败全部保留。

研究定义已按补充要求写入 [RESEARCH_SCOPE.md](/home/inspur/project/RoboTwin/experiments/held_module_key/RESEARCH_SCOPE.md)：最终问题是 **preempt an operating arm for sensing**。如果将来其他候选通过局部检查，两种条件中两臂都必须有可执行、有价值的 manipulation work，主要改变暂停左侧工作的边际机会成本，不预设 LOW/HIGH 的正确策略。本轮没有运行独立的左侧工作负载，不能把左臂回原位时间解释为机会成本，也不构成最终 benchmark。

**实现和物理结构。** 新代码位于 [envs/held_module_key](/home/inspur/project/RoboTwin/envs/held_module_key)，单路径入口为 [held_module_key_demo.py](/home/inspur/project/RoboTwin/scripts/held_module_key_demo.py)，配置为 [held_module_key.yml](/home/inspur/project/RoboTwin/env_cfg/task_config/held_module_key.yml)。GPU 0，CPU PhysX / MPLib，4 ms 物理步长，三路 640×480 RGB-D 和视频均为 10 Hz。初始化用真实夹指接触稳定 3 s，之后只有关节驱动和物理步进，没有物体焊接、位姿回填或运动后 teleport。

模块本体 76×64×16 mm，把手保持相同。底部主键 32×20×12 mm，带不对称侧舌；键在模块坐标中的 x/y 偏移和 yaw 可变。固定座具有对应实体 L 形孔，每侧约 2.5 mm 间隙。底键与孔的实体碰撞保留；四块颜色只是底键表面的非碰撞涂色，不能改变安装约束。固定座采用公开的夹具标定坐标，实际初始 head/右腕图像也可看到定位座。控制器不能读取底键 actor pose、模块 ID、实例 seed 或隐藏键配置。

实例 400 的底键偏移和角度仅写在 GT 审计文件；策略输入不包含这些值。此轮所有物理试验均为开发实例 400，未将其称为独立测试，也未运行新的种子队列。键的随机变化已实现，跨变化实例的稳健性尚未验证。

**实际相机安装链。** [URDF 链审计](/home/inspur/project/RoboTwin/result/held_module_key/final400_H/mounting_chain.json) 对应实际加载的 aloha-agilex URDF：

| 链段 | 实际关节 | 类型/关系 |
|---|---|---|
| fr_link5 → fr_link6 | fr_joint6 | 可转动 |
| fr_link6 → right_camera | right_camera_joint | 固定，平移 [0.07, 0.032, 0.065] m，rpy [0, 0.4, 0] |
| fr_link6 → fr_link7 | fr_joint7 | 可伸缩，沿 +y |
| fr_link6 → fr_link8 | fr_joint8 | 可伸缩，沿 −y |
| 左侧相应链段 | fl_joint6/7/8、left_camera_joint | 相同结构 |

相机固定在掌部第六连杆上，但相机到接触指面的链路包含伸缩关节。因此不能把相机到物体的相对位姿当作机械固定约束。腕部相机的实际渲染位姿与 URDF FK 推导的观测位姿最大矩阵元素差约 6.1×10⁻⁷，见 [相机校验](/home/inspur/project/RoboTwin/result/held_module_key/dev400_translate_d/rendered_camera_vs_fk.json)。

冷启动后实际抓取相对设计标定约偏转 15.8°；这个 GT 偏差没有注入控制器。感知直接由 RGB-D 估计完整的 `T_ee_key`，安装目标为公开固定座变换乘以该视觉变换的逆。相机、物体、机器人关节和接触分文件记录。观测导出的相机矩阵使用 OpenCV 坐标；`evaluator_gt.jsonl` 中 `T_camera_module` 使用实际挂载相机的 SAPIEN 坐标（前/左/上）。

| 保持同一次抓取的几何试验 | 相对位移最大变化 | 相对旋转最大变化 | 夹指开度最大范围 |
|---|---:|---:|---:|
| 右臂平移 7 cm 后返回 | 0.0665 mm | 0.00533° | 0.1203 mm |
| H 低位观察、左臂返回 | 0.00742 mm | 0.00055° | 0.00746 mm |
| S 展示给 head、右臂返回 | 0.3186 mm | 0.01062° | 0.3212 mm |

这些运行有双夹指接触记录，支持“稳定抓取时近似固定”的判断。求解器仍报告非零瞬时物体角速度，几何试验峰值约 1.85–2.48 rad/s；原始日志保留。本报告按实际位姿漂移描述稳定性，不宣称零振动或绝对刚性。改变抓取后相对位姿可发生约 7.5 mm / 15.8° 的变化，不能沿用此前抓取的固定关系假设。

**真实相机证据。** 所有图像来自模拟器渲染；overview 只录制，不进入策略。每个首次有效估计都保存当时 head、左右腕原图、RGB-D、内外参及关节状态。以下放大图只放大实际像素：

![实际底键像素](/home/inspur/project/RoboTwin/experiments/held_module_key/key_pixels_zoom.png)

- [初始和 H/S 首次定位三相机图集](/home/inspur/project/RoboTwin/experiments/held_module_key/actual_camera_grid.jpg)
- [H 首次定位原图及相机/关节记录](/home/inspur/project/RoboTwin/result/held_module_key/final400_H/frames/first_information)
- [S 首次定位原图及相机/关节记录](/home/inspur/project/RoboTwin/result/held_module_key/final400_S/frames/first_information)
- [H/S 同物理时钟并排视频](/home/inspur/project/RoboTwin/experiments/held_module_key/H_S_actual_video.mp4)

首次关键帧的 GT-only 位姿误差：H 为 1.59 mm / 0.42°，S 为 1.23 mm / 0.40°。所有相机都运行同一四颜色 RGB-D 刚体拟合、几何检验和两帧一致性判断；运动目标的一致性在机器人 EE 坐标中检查。合法信息足够后立即取消专门观察，不等到预先安排的视点终点。

**五类路径的结果与覆盖边界。**

| 路径 | 实际结果 | 完整恢复代价/证据 |
|---|---|---|
| 保持抓取、整体移动右臂 | 实际平移 7 cm，三路均无有效底键估计；自身相机与模块的相对关系几乎不变 | 往返 2.060 s；[视频](/home/inspur/project/RoboTwin/result/held_module_key/dev400_translate_d/execution.mp4) |
| 左臂主动换位 | 高位和掠视候选失败，低位候选在 4.300 s 有效；预定 5.016 s 视点到达被提前取消 | 左臂恢复 9.196 s；[观察与恢复视频](/home/inspur/project/RoboTwin/result/held_module_key/dev400_helper_c/execution.mp4) |
| 暂放/改变抓取后自观察 | 实体支架暂放和真实重抓已执行；尚未获得有效自观察。前侧视线被支架挡住；改从开口端的指定相机目标又出现 IK 失败 | 最新完整接触重抓/返回 11.760 s，**不称为成功观察方案**；[视频](/home/inspur/project/RoboTwin/result/held_module_key/dev400_putdown_d/execution.mp4)、[另一实际视点视频](/home/inspur/project/RoboTwin/result/held_module_key/dev400_putdown_c/execution.mp4) |
| 展示给 head | 扩展合法关节姿态后找到有效路径，2.600 s 由 head 定位；计划终点约 3.584 s，被提前取消 | 返回原持物姿态 5.708 s；[视频](/home/inspur/project/RoboTwin/result/held_module_key/dev400_head_e/execution.mp4) |
| 展示给未调走的左腕 | 左腕始终保留在初始工作位、正常接收图像。60 个目标视点的初始列表未找到 IK 路径；后续 6000 个有限关节样本中，1 个通过粗可见性筛选，路径因左相机与 fr_link4 碰撞被拒绝 | 尚无实际成功展示轨迹，不能报告完成代价或全局不可行；[初始三相机图](/home/inspur/project/RoboTwin/result/held_module_key/dev400_passive_left_b/frames/initial)、[被拒绝路径](/home/inspur/project/RoboTwin/result/held_module_key/dev400_passive_left_b/candidate_plans.json) |

给 H 和 S 均做了公共目标的路径选择。H 检查较低相机位置及滚转，S 检查模块平移、转腕、相机展示位置和有限关节姿态；速度、加速度和真实碰撞规则共用。候选列表只用机器人 FK、公开夹具与模块外形，无隐藏键真值。有限关节搜索的粗视场/朝向筛选以及直线关节路径检查不覆盖所有合法路径；RRT 绕行、其他 IK 分支、视场边缘及其他重抓方式仍未穷尽。因此只报告已找到的最好可用方案。

**完整安装的时间分解。** 最终 H/S 的配置、全部源文件快照和初始状态 SHA 完全相同，见 [配对校验](/home/inspur/project/RoboTwin/experiments/held_module_key/paired_validation.json)。H 的左臂恢复与右臂操作实际并行，按 20 ms 实测关节位置差统计，两臂同时运动 3.84 s；S 左臂未被调动。

| 物理事件/代价 | H：主动左腕 | S：展示给 head |
|---|---:|---:|
| 观察启动 | 0.000 s | 0.000 s |
| 首次有效信息 | 4.300 s | 2.600 s |
| 获得信息 → 到达预备位/启动插入 | 1.196 s | 2.732 s |
| 到达预备位/启动插入 | 5.496 s | 5.332 s |
| 右侧稳定完成 | **8.380 s** | **8.212 s** |
| 右腕退出完成 | 9.900 s | 9.736 s |
| 左臂恢复原位 | 9.196 s | 未被调动 |

这里没有把第一次检测当作完整观察代价。S 的恢复/准备段确实更长，抵消了其大部分 1.7 s 信息优势；完整右侧任务仍早 0.168 s。没有将如此小的单实例差异解释为统计显著优势。

安装就绪的限制需单独保留：表中 5.496 / 5.332 s 是物理时钟下的预备位动作结束及实际插入启动，现场 FK 已保存。该时刻 H 的位置误差为 [−1.526, −1.557, +0.293] mm、角误差 0.318°，S 为 [−1.750, −1.359, +0.349] mm、0.261°。**额外加入的“3D 位置误差 <2 mm 且低关节速度”的严格静止就绪标志两组均为 null。** 不把它改写成已满足严格就绪；这个指标未完全校准，原值和 [插入启动审计](/home/inspur/project/RoboTwin/experiments/held_module_key/attempts.json) 均保留。右侧稳定完成则按实体座接触、键位置/方向、线/角速度阈值连续 0.5 s 判定，已真实满足；最终 H/S 平面误差均约 1.55 mm、角误差分别 0.670°/0.606°。

![完整物理时间](/home/inspur/project/RoboTwin/experiments/held_module_key/physical_timing.png)

**失败、修改和统计。** 初始化、几何探测和安装尝试没有混为一个任务成功率：

| 类别 | 全部尝试 | 初始化失败 | 初始化成功 | 预定动作流程执行到终点 | 有效观察且恢复/稳定安装 |
|---|---:|---:|---:|---:|---:|
| 几何探测 | 18 | 3 | 15 | 10 | 2 |
| 完整安装（含同实例重跑） | 4 | 0 | 4 | 4 | 4 |

完整安装任务成功率为 4/4（按初始化成功），全部安装尝试完成比例也为 4/4；这只有一个开发实例、两次 H/S 配对，**不是四个独立测试实例**。全部研究尝试 22 次，初始化失败 3/22，预定动作流程到终点 14/22，终态结果记录 22/22。几何探测中未获得信息、路径拒绝、初始崩溃均保留；流程走完不等于观察成功。不同模式的有效终点不合并宣传为 benchmark 成功率。逐尝试的 instance/config/policy/source 标识和重跑关系在 [CSV](/home/inspur/project/RoboTwin/experiments/held_module_key/attempts.csv)、[完整 JSON](/home/inspur/project/RoboTwin/experiments/held_module_key/attempts.json)。

主要负面记录：

- 首先将持物高度提高到 1.04 m，产生 fr_link3/fr_link5 自碰撞及物体弹出。三次诊断保留；改回已验证的 0.94 m 高度，并在初始化前检查碰撞。
- 高支架的暂放预备位被拒绝；新候选自己的支架降低到 0.90 m，落在桌面的实体支脚保留，预备高度缩短为 0.06 m。此修改为可达性和自观察通道服务，未改变 v1/v2。
- `putdown_b` 的直接空手路径撞落模块，随后没有真实重抓成功。它还暴露了诊断恢复分支等到 30 s 的无效等待；该等待已删除。**38.064 s 的命令完成不能当作有效观察代价或 S 对比数据。** 后续侧向退让在物理中保持了支撑和重抓，但观察仍未成功。没有用失败重抓或旧等待制造 H 优势。
- 固定左腕展示、其他重抓方式和混合/接手策略没有完成验证；不能把这些尚未实现的替代项标为不可能。已有合法 head 方案使本轮无需继续扩大搜索。

**保存和检查。** v1/v2 原代码、配置、视频、全部结果及负面结论没有修改；[哈希复核](/home/inspur/project/RoboTwin/experiments/held_module_key/preservation_check.json) 检查原有 9,331 个文件，变化数为 0。新增候选每次运行都保存独立源文件快照和结果，不覆盖重跑。移动物体坐标变换及感知/GT 边界的两项测试通过；四次完整安装是实际 GPU 0 渲染和物理执行验证，验收没有 H 优势断言。[停止状态和源码哈希](/home/inspur/project/RoboTwin/experiments/held_module_key/STOPPED.json) 已保存。
