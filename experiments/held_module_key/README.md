# held_module_key — 已停止的独立局部几何候选

结论见 [REPORT.md](/home/inspur/project/RoboTwin/experiments/held_module_key/REPORT.md)。模块背面遮挡与左臂观察可行；现有合理 head 方案完整右侧任务更便宜，未发现 H 局部收益，因此不进入 LOW/HIGH、训练、P2 或扩大评测。未来定义见 [RESEARCH_SCOPE.md](/home/inspur/project/RoboTwin/experiments/held_module_key/RESEARCH_SCOPE.md)。

所有尝试在 `/home/inspur/project/RoboTwin/result/held_module_key/`。最终同版本比较为 `final400_H`、`final400_S`；均为开发实例 400，不能当独立测试。原固定槽位 v1/v2 没有改动。

以下仅用于手动复现；输出必须使用新目录。当前停止状态不要求继续运行。

```sh
cd /home/inspur/project/RoboTwin
PYTHONUTF8=1 /mnt/sda/conda_envs/RoboTwin/bin/python scripts/held_module_key_demo.py --route helper --pick 1 --candidates-from result/held_module_key/dev400_helper_b/candidate_plans.json --install --gpu 0 --output result/held_module_key/reproduce_H_new
PYTHONUTF8=1 /mnt/sda/conda_envs/RoboTwin/bin/python scripts/held_module_key_demo.py --route head --candidates-from result/held_module_key/dev400_head_e/candidate_plans.json --install --gpu 0 --output result/held_module_key/reproduce_S_new
PYTHONUTF8=1 /mnt/sda/conda_envs/RoboTwin/bin/python -m unittest experiments.held_module_key.test_contracts -v
```

`--install` 之前的几何探测为默认模式。CLI 没有批量评测、HIGH、训练或 P2 功能。`--cycle` 只标记暂放重抓诊断，不代表必须暂放的 S；未传该标志时，已获得合法信息可在释放前取消暂放。所有路径均保留 head 和两腕输入。

单次目录包括 `provenance.json` / `source/` / `config.json`、`initialization_gt.json`、`mounting_chain.json`、三相机 PNG/NPZ 与关节状态、实际 MP4、控制/感知/关节日志以及隔离的接触和位姿 GT 日志。物理安装就绪的严格 2 mm 静止指标未通过，报告保留 null，并单独列出实测预备位误差与真实稳定完成时刻。

`summarize.py` 和 `compose.py` 只汇总已有文件、拼接实际图像/视频，不运行模拟器。`attempts.json/csv` 保留所有失败及重跑；`STOPPED.json` 固化停止原因和源码哈希。
