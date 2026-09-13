# Tray active observation — 当前为 Phase 2，尚未完成 demo

截至 2026-09-14，已完成仓库检查和 CPU PhysX 的最小几何 oracle 调试。
**没有完成真实视觉闭环、Task B、四种策略或 ranking reversal 验证。**
当前机器 NVIDIA 驱动不可用，SAPIEN 创建渲染设备报
`vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent`；沙箱外复查结果相同。
因此按照用户指定顺序停在 Phase 3 的真实相机验证入口，不能以 oracle 代替视觉。

详见 [仓库检查](INSPECTION.md)、[实际结果与限制](REPORT.md)。

## 当前可运行命令

```sh
cd /home/inspur/project/RoboTwin
PY=/mnt/sda/conda_envs/RoboTwin/bin/python

# 真实渲染检查；当前环境预期非零退出，并保存明确错误。
$PY scripts/active_observation_tray_demo.py probe --output result/active_observation_tray/my_render_probe

# Phase 2：无渲染的真实机器人、接触和轨迹调试。明确允许 GT，仅用于几何。
$PY scripts/active_observation_tray_demo.py oracle --physics-only --seed 0 --output result/active_observation_tray/my_geometry_0

# GPU 环境恢复后，同一几何可保存所有腕部/头部/overview 的实际图像。
# 仍是 oracle 调试，不是 self/helper 策略。
$PY scripts/active_observation_tray_demo.py oracle --seed 0 --output result/active_observation_tray/my_rendered_geometry_0

# 汇总现有记录，包含失败；不运行仿真，不构造策略结果。
$PY experiments/active_observation_tray/summarize_phase2.py result/active_observation_tray --output experiments/active_observation_tray
```

每个仿真输出目录必须尚不存在。退出码 0 表示本条验证成功，1 表示失败。
`oracle` 的 `eligible_for_strategy_benchmark` 始终为 false。

## 已新增文件和关键代码

- `envs/active_observation_tray/configs.py`：几何、初始平移扰动和统一运动上限。
- `scene.py`：桌面、宽支撑面、真实开槽、带 handle/peg 的动态 tray；拥有真值。
- `motion.py`：复用原有限速 FK/IK/轨迹生成，移除旧实验的无关规划障碍。
- `debug_oracle.py`：真实抓取与插入动作、动作边界、物理步计时、源码快照。
- `evaluation.py`：独立计算真值误差和插入判定。
- `scripts/active_observation_tray_demo.py`：渲染探针和 Phase 2 命令。
- 本目录：检查、报告和由真实结果生成的 CSV/JSON。

运行时核心路径是 `kin.drive(...) → kin.compensate() → scene.step()`。
只有初始机械臂配置调用一次 `set_qpos`；之后没有物体 pose 设置、焊接约束、
运动瞬移或按策略修改 physics。时间等于实际物理步数 × 0.004 秒。
抓取通过两个手指的物理接触维持，tray 会有真实偏转与滑动。

CPU 调试的 URDF 临时副本只移除 visual 元素以避免初始化渲染器；保留碰撞、
质量/关节结构和 camera mounting links。由于现有 `Robot` 的 cuRobo 模块导入
会初始化 CUDA，CPU 模式用同一 URDF/配置直接建立驱动；有渲染模式继续使用
原 `Robot` 类。该差异不被隐藏，也不能据此宣称已验证视觉版的一致性。

## 后续必须完成的内容

1. 恢复当前内核可用的 GPU 驱动/设备访问，先通过 `probe`。
2. 使用真实 RGB-D 验证 normal、self 和 helper 视角；根据实际可见性决定
   是否需要 rack 局部遮挡结构。目前场景只验证支撑与 slot，尚未调视觉遮挡。
3. 实现传统视觉与统一插入控制器，移除 debug oracle 的目标输入；历史观测
   根据 FK 补偿托盘跟随左腕运动，记录接触滑移误差。
4. 依次跑通 self/helper，再加入右臂 1/3/5 块搬运与共享物理时间线。
5. 才提供 `no-extra-observation`、`self-observation`、`immediate-helper`、
   `helper-after-current-B-step`、`pre-observation` 和
   `helper-motion-without-helper-image` 的执行入口及配对 benchmark。

目前这些策略**没有可运行命令**；没有添加会误导用户的空策略或假 benchmark。
不能从当前 oracle 的 3 个成功实例推断辅助观察价值。
