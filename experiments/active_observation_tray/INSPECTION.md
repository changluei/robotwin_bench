# 实现检查记录 — 2026-09-15

## 数据边界

`vision.py` 定义策略唯一可见的 `RGBFrame`：name、timestamp、RGB、intrinsic 和
`T_world_camera`。采集只请求 SAPIEN `Color` attachment。控制器源码没有访问
`hole_xyz`、actor `get_pose()`、Position attachment、segmentation 或 seed。

slot 使用四个红色标记；tray 使用四个颜色各异的小标记。后者避免同色平面 PnP
的对应歧义，并通过最小/最大连通域面积排除托盘表面、机械臂和 Task-B 方块。
helper 画面中四点均为真实 RT 渲染像素，不是投影生成的假检测。

`evaluation.py` 是独立的 privileged 边界，只在控制动作完成后读取 tray、hole 和
block pose。`test_contracts.py` 通过 AST/源码检查禁止视觉策略读取上述字段。

## 物理和调度

- 左臂：抓取 → 抬升 → 部分插入 → RGB 横向修正 → 固定距离最终插入 → 稳定检查。
- 右臂：逐块 hover/pick/close/lift/place/open/retreat；1/3/5 是同一队列的前缀。
- 每个 tick 先写左、右驱动和重力补偿，再执行恰好一次共享 `scene.step()`。
- helper 暂停 B，保存右臂关节状态，移动观察、采 RGB、返回，再恢复未完成阶段。
- tray 和 blocks 仅靠 PhysX 接触及夹爪摩擦运动；初始化后不设置物体 pose。

本机默认 raster shader 在 camera readback 上死锁，所以场景在创建 RenderSystem
前选择 `rt` shader。光线反弹设置只影响渲染质量，不进入观测张量。

## 已发现的真实限制

左腕相机相对夹爪固定，tray 相对夹爪也由抓取固定，因此移动左臂不能改变
“相机—tray”遮挡关系。真实 self frame 中托盘覆盖目标区域，四个 slot 标记不可见。
该策略保留为有效失败样本；helper view 则能同时看到 slot 与 tray 标记。

延迟 helper 可能在返回路径上扰动已经放好的 B 方块。这不是计时公式惩罚，而是
实际接触后最终位置误差，结果文件会按最终状态记录。
