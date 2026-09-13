# occluded_socket_drawer 观察校准

先读 [v1 原场景审计](V1_AUDIT.md)，再读 [本轮实际报告](REPORT.md)。原 v1 完全保留；新代码位于 `envs/occluded_socket_calibration/`，新产物位于 `result/occluded_socket_calibration/`。没有训练或 P2。

核心结果：H 的原 1.9 s 跨条件差异来自打开抽屉阻挡观察路径；已有有效信息时 H/S 都及时取消观察。v1 中直接执行 seed 0 为 6.196 s，快于优化 H 的 8.048 s。实体遮挡板 v2 一度显示 H 优势，但可行的小幅持物转腕将 S 降至 6.568 s，快于 H 的 8.356 s。冻结后新种子 300/301 的四次 LOW 核验全部成功；300 初始可见，两者同为 6.220 s，301 的 H/S 为 8.356/6.572 s。未进入新几何 HIGH。

使用现有 `/mnt/sda/conda_envs/RoboTwin/bin/python` 和 GPU 0。`audit-suite` 专用于原 v1 的 14 次诊断复跑；`route` 是单次，`route-suite` 是明确给出的候选列表；`oracle` 显式使用 GT 验证物理安装。每次输出目录必须不存在。HIGH 在新 v2 入口中被阶段条件阻止，因为 LOW 未发现可信 H 优势。

```bash
cd /home/inspur/project/RoboTwin
export PYTHONUTF8=1
PY=/mnt/sda/conda_envs/RoboTwin/bin/python
RUN=result/occluded_socket_calibration/reproduce_$(date +%Y%m%d_%H%M%S)

$PY experiments/occluded_socket_calibration/test_contracts.py
$PY scripts/calibrate_occluded_socket.py audit-suite --gpu 0 --output "$RUN/v1_audit"
$PY scripts/calibrate_occluded_socket.py route-suite --scene v1 --seed 0 --gpu 0 --record --routes helper_high,self_small_back,direct_live,direct_initial_only,putdown --output "$RUN/v1_low"
$PY scripts/calibrate_occluded_socket.py oracle --scene v2_baffle --seed 0 --gpu 0 --record --output "$RUN/oracle"
$PY scripts/calibrate_occluded_socket.py route-suite --scene v2_baffle --seed 0 --gpu 0 --record --routes helper_camera_here,self_small_yaw,direct_live,putdown --output "$RUN/v2_low"
$PY scripts/compose_occluded_calibration.py --left "$RUN/v2_low/s0_LOW_helper_camera_here" --right "$RUN/v2_low/s0_LOW_self_small_yaw" --output "$RUN/comparison"
$PY scripts/summarize_occluded_calibration.py "$RUN"
$PY scripts/audit_occluded_calibration_physics.py "$RUN"
```

原型 A/B 不适合收益比较，仍可用 `--scene v2` / `--scene v2_clearance` 复查其真实失败。有效几何为 `--scene v2_baffle`。全部几何、候选、失败原因和校准过程见报告及 PROGRESS.md。

两次直接执行诊断共享 t=0/.1 s 的实际两帧初始窗口。direct_live 在粗预接近过程中继续接受新有效信息；direct_initial_only 仅采用初始估计，后续图像保留但不供该诊断更新目标。两者真正入座均需要视觉目标，无 oracle 回退。已放下模块时，putdown 必须先真实重抓恢复，再进入共享安装流程。

新种子 300/301 已用于本轮有限核验，今后修改几何/控制器后，不得再把它们当未见测试种子。[FROZEN_V2_LOW.json](FROZEN_V2_LOW.json) 保留冻结时间、代码/配置哈希和所选方案。统计按 config/instance/policy/attempt 区分，oracle 与强制能力验证单列。
