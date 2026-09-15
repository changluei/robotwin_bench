# Active-observation tray experiment

这是一个可运行的双臂主动观察实验。左臂抓住托盘，将远端 peg 插入有随机横向
偏移的 slot；右臂同时搬运 1/3/5 个方块（Task B），也可以暂时中断搬运去提供
helper view。策略比较的核心是：观察能提高 Task A 成功率，但自观察、辅助观察、
等待时机和中断 Task B 都有真实的物理时间与干扰成本。

## 观测约束

测试策略是 **RGB only**：相机适配器只调用 `get_picture('Color')`，输出 RGB、
内参、外参和时间戳。没有读取 Position 图、距离图、分割图、actor pose、随机
seed 或 hole/tray 真值。红色 slot 标记和托盘上的绿/橙/品红/黄色标记都是普通
渲染几何；位姿由 HSV 连通域和 calibrated monocular PnP 得到。

`set_ray_tracing_path_depth(...)` 是 SAPIEN 的光线路径反弹次数设置，仅用于避开
本机默认 raster readback 死锁；它不是传感器的深度数据。

真值只在 episode 结束后的 `evaluation.py` 中计算成功率、末端误差和 Task-B
方块误差，不会反馈给控制器。`oracle` 命令仍保留为明确标注的几何调试入口，
不能计入策略 benchmark。

## 六种策略

| policy | 行为 |
|---|---|
| `no-extra-observation` | 只用正常腕部画面；看不到目标时执行零修正 |
| `self-observation` | 左臂带托盘移动后使用自己的腕部相机 |
| `immediate-helper` | 右臂立即暂停 B，移动到 helper viewpoint 并采一帧 RGB |
| `helper-after-current-B-step` | 等右臂完成当前方块后再提供 helper RGB |
| `pre-observation` | 抓取前先由 helper 观察 slot，之后用公开 grasp/FK 传播 peg |
| `helper-motion-without-helper-image` | 执行同样 helper 运动但丢弃图像，用于隔离运动成本 |

所有机械臂目标在一次 `scene.step()` 前共同写入；没有为两臂分别推进仿真时间。
episode 内不瞬移物体、不附着 tray，也不按策略修改动力学参数。

## 运行

```bash
cd /home/cuidi/workspace/code/robotwin_bench
PY=/mnt/data/users/cuidi/envs/robotwin_bench/bin/python

# 只验证真实 Color readback
$PY scripts/active_observation_tray_demo.py probe \
  --output /tmp/active_tray_probe

# 单 episode
$PY scripts/active_observation_tray_demo.py episode \
  --policy immediate-helper --b-load 5 --seed 0 \
  --video --output /tmp/active_tray_episode

# 配对实验；每个 seed × load 都运行全部六种策略
$PY scripts/active_observation_tray_demo.py batch \
  --seeds 100:120 --b-loads 1,3,5 \
  --output result/active_observation_tray/rgb_benchmark
```

输出目录必须预先不存在。单次结果在 `result.json`，并保存实际采集的 PNG；加
`--video` 会生成带仿真时间和动作标签的 `overview.mp4`。该 overview 只用于结果
展示，绝不会输入控制器。batch
持续写入完整 `results.json` 和扁平的 `results.csv`，即使某条策略失败也会保留。

## 已验证范围

- RT Color readback：通过，64×64×4；CPU 和 CUDA readback 均不再卡死。
- `immediate-helper, seed=0, b_load=1`：Task A/B 成功。
- `pre-observation, seed=0, b_load=1`：Task A/B 成功。
- `immediate-helper, seed=0, b_load=5`：Task A 成功，5 个 B 方块全部成功；
  最终方块中心误差为 3.1–6.4 mm，仿真时间 69.004 s。
- 当前 Aloha 左腕相机被所持托盘固定遮挡，已实际保存 self 图像并记录为无效观察；
  这是该 embodiment 的实验结果，不用合成检测或真值把它改成成功。

最终代码的 seed-0/B=1 六策略结果见 [CSV](smoke_seed0_b1.csv) 和
[JSON](smoke_seed0_b1.json)。这些是 smoke tests，不是 20-seed 统计结论；正式
排名应运行上面的 batch 命令。

一条 `immediate-helper, seed=0, B=1` 的加速 overview 示例保存在
[overview.mp4](video_immediate_helper_seed0/overview.mp4)，对应 helper 原始 RGB
帧为 [helper_step_2873.png](video_immediate_helper_seed0/helper_step_2873.png)。
