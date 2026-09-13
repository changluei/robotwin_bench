# 实际进度与结果（2026-09-14）

**未完成用户要求的双臂主动观察 demo。完成 Phase 1 和 Phase 2 的 CPU
物理几何验证；Phase 3 被真实渲染环境阻塞。没有视觉 baseline benchmark，
也没有证据表明存在或不存在 strategy ranking reversal。**

## 已验证的场景

复用 Aloha-agilex 双臂 asset、关节驱动配置和原有 FK/IK/限速轨迹算法。
新增独立的宽底板 rack、实际空槽、托盘、前端 handle 和远端单 peg。
插入沿 world-y，横向修正沿 world-x。Rack 宽 210 mm，tray 宽 90 mm，
没有精密导轨。Peg 宽 10 mm，slot 宽 20 mm，提供每侧 5 mm 的横向间隙。
初始 hole 横向偏差 ±14 mm，rack/tray 各有 ±2 mm 的平移扰动。
当前没有角度和初始关节扰动，没有 B，也未添加用于视觉研究的 hood。

左臂实际执行：悬停 → 下探 → 夹爪闭合 → 抬升 → 部分插入 → 原位横向修正
→ 最终插入 → 持续夹持稳定验证。右臂保持固定初始姿态，尚不执行观察或 B。
Tray 通过碰撞和手指摩擦移动；episode 内没有物体瞬移、机械臂 qpos 瞬移、
固定附着约束或关闭碰撞。

## 同一最终版本的 Phase 2 结果

以下全部使用 GT 驱动，只能用于几何调试；**不是 self/helper 成绩**。
时间包含抓取、插入和真实稳定验证，dt=0.004 s。

| Seed | 物理步 | 完成时间/s | 末端横向真值误差/mm | 插入成功 |
|---|---:|---:|---:|---|
| 0 | 2004 | 8.016 | -2.348 | 是 |
| 1 | 2024 | 8.096 | -2.279 | 是 |
| 2 | 2205 | 8.820 | -1.230 | 是 |

权威原始记录位于
`result/active_observation_tray/phase2_holdfix_seed{0,1,2}/result.json`，
完整动作、关节、接触和误差时间序列在各自的 `trajectory_gt.jsonl`。
每次运行保存源码副本及 SHA256；时间严格等于实际 `scene.step()` 次数乘 dt。
以上三例为开发 seed，不代表独立泛化评估或精密装配可靠性。

插入成功要求 peg 中心横向误差小于 5 mm、纵向进入目标区、竖向误差小于
9 mm，并连续 125 步保持插入状态且 tray 线速度低于 0.02 m/s。
最长验证 2 秒，所有实际步数计时；不要求释放托盘。
当前仍有小幅角速度抖动，没有声称完成精密静止或 release-and-settle 验证。

## 开发失败及修改

全部带结果文件的尝试见 [CSV](phase2_attempts.csv) / [JSON](phase2_attempts.json)。
没有删除失败后重新计算一个无失败分母。

1. 最早的 `phase2_dev0` 在创建运行目录前失败：现有 Robot 导入 cuRobo
   时初始化 CUDA，报 `No CUDA GPUs are available`，随后无法导入 CuroboPlanner。
   因此该次不在 8 个 `result.json` 的汇总中。解决方式仅针对 CPU debug：
   从同一 URDF/配置直接创建 articulation 和驱动，不修改旧 Robot/core。
2. `phase2_dev1`：MPLib 对临时 URDF 的绝对 mesh 路径处理失败，0 个物理步。
   修复为碰撞规划读取原 asset URDF；CPU PhysX 读取去 visual 的临时副本。
3. `phase2_dev2`：连接颈宽 20 mm，遇 20 mm slot 时存在真实卡碰，最终 peg
   中心仍在目标前 2.03 mm。将连接颈缩至 8 mm，peg 仍宽 10 mm，slot 未变。
   后续 `phase2_dev3` 成功。没有改变成功阈值。
4. `phase2_seed2`：peg 已进入，但原固定 0.5 秒验证窗口中出现短暂速度抖动，
   如实记录失败。改为最多推进 2 秒、要求连续 0.5 秒满足相同条件，
   任一不满足即清零连续步数；未放宽速度或几何阈值。最后对三个 seed 全部重跑。
   Seed 2 因真实稳定过程增加到 2205 步，没有凭公式添加惩罚。

## 视觉与 baseline 状态

| 请求项 | 当前状态 |
|---|---|
| 正常腕部图像 / normal view | 未能渲染，不能判断是否已足够 |
| self-observation 独立成功 | 尚未实现/验证 |
| immediate-helper | 尚未实现/验证 |
| helper-after-current-B-step | 尚未实现/验证 |
| pre-observation | 尚未实现/验证，不排除其击败 helper |
| helper-motion-without-helper-image | 尚未实现/验证，仍是后续必需门槛 |
| A/B 同步物理时间线 | 尚未加入 B |
| 真值隔离的统一视觉控制器 | 尚未实现；当前入口明确是 debug oracle |
| 策略 CSV/JSON / 排名反转 | 无有效 episode，不能给出结果 |

目前只能证明场景允许左臂真实抓取和局部修正。Self motion 的成本、helper
travel 的成本、B 资源竞争和视觉增量都还无法测量。因而现在没有依据调整
observer pose 来追求反转，也不能把这三次 oracle 的计时当作研究结论。

## 环境阻塞与下一步

- `/mnt/sda/conda_envs/RoboTwin/bin/python` 中 SAPIEN 3.0.0b1 / MPLib 0.2.1 /
  OpenCV 4.10.0 可导入。
- 当前内核 `6.8.0-138-generic`。`/sys/module/nvidia`、
  `/proc/driver/nvidia/version`、`/dev/dri` 不存在。
- `nvidia-smi` 在沙箱内和显式沙箱外执行均不能连接驱动。
- 最小 `sapien.render.RenderSystem()` 在沙箱内及沙箱外均报
  `vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent`。
- 已安装软件 Vulkan ICD，但无法满足当前 SAPIEN 的设备扩展要求。
  保存的探针结果：`result/active_observation_tray/render_probe/environment_probe.json`。

需要恢复 GPU 驱动与设备访问，并先通过 README 中的 `probe`，才能继续用户
要求的 Phase 3。这里没有擅自重装驱动、修改内核或重启主机。随后仍须依次
完成真实视觉、自观察、辅助观察、B 和配对评测；这不是已完成任务的交付。

## 修改范围与检查

只新增 `envs/active_observation_tray/`、`scripts/active_observation_tray_demo.py`
及 `experiments/active_observation_tray/`，运行记录写入 `result/active_observation_tray/`。
RoboTwin core、既有实验和原 asset 均未修改。语法编译检查通过。
当前 phase-2 motion adapter 未建立新 rack 的规划障碍集合，实体接触由
PhysX 处理并记录；没有声称已完成双臂碰撞规划验证。
