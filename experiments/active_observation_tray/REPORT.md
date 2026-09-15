# 当前结果 — 2026-09-15

实验代码、六策略 CLI、纯 RGB 估计器、共享双臂调度、1/3/5 Task-B 队列以及
CSV/JSON 汇总已经实现。以下均来自真实 SAPIEN RT Color 图像和 PhysX 动作。

| 验证 | 结果 | 关键测量 |
|---|---|---|
| render probe | 通过 | Color shape 64×64×4 |
| immediate helper，seed 0，B=1 | 成功 | RGB 修正后 Task A/B 均成功 |
| pre observation，seed 0，B=1 | 成功 | 早期 RGB slot + grasp/FK peg prior |
| immediate helper，seed 0，B=5 | 成功 | A 稳定完成；5 块最终误差 3.1–6.4 mm；69.004 s |
| no extra，seed 0，B=1 | 失败 | 正常腕部画面无可用 slot，零修正后真实碰壁 |
| self observation，seed 0，B=1 | 无效观察 | 所持 tray 固定遮挡左腕相机；保存了真实失败帧 |
| delayed helper，seed 0，B=1 | 失败 | A 已插入但返回路径扰动已放置 B，最终误差 38.8 mm |
| helper motion/no image，seed 0，B=1 | 失败 | 同样运动但无 helper 像素，零修正 |

开发中修复的主要问题：

1. H200 上默认 Vulkan raster readback 挂起，改用仓库可工作的 RT shader；只读
   Color attachment。
2. 同色 tray 标记产生平面 PnP 歧义，改为绿/橙/品红/黄四点并扩大基线。
3. 大面积青色托盘和黄色机械臂曾被误识别，加入 marker 面积上限。
4. tray 连接颈曾比 peg 先碰孔壁，缩短其碰撞体，使 peg 成为首个导向接触。
5. 最终评估不再用 GT 提前结束 episode；现在固定等 Task B 完成并额外沉降 2 秒，
   然后只读取一次真值，因此高负载和不同策略使用同一停止规则。

当前结果只是功能 smoke test，不能宣称策略总体排名或 ranking reversal。权威统计
需要按 README 的 paired batch 对 20 个或更多 held-out seeds 运行；所有失败会保留
在分母中。

最终版本的单 seed 六策略 smoke 汇总保存在 [CSV](smoke_seed0_b1.csv) 和
[JSON](smoke_seed0_b1.json)。
