# Repository inspection — 2026-09-14

## Scope and reusable code

- `envs/_base_task.py`: task lifecycle (`setup_demo`, `load_actors`, `play_once`,
  `check_success`), SAPIEN scene creation, default 250 Hz physics, observation
  collection and action dispatch. `move` aligns action lists; its blocking
  execution is unsuitable for interruption at arbitrary B progress.
- `envs/pick_dual_bottles.py`, `place_dual_shoes.py`, `stack_blocks_two.py`:
  paired arm actions, grasp/lift/place patterns, procedural blocks.
- `envs/robot/robot.py`: existing Aloha URDF, arm joint groups, drive gains,
  gripper joints, wrist links, FK and planning interface. Reuse this embodiment.
- `envs/camera/camera.py`: both wrists remain enabled; `update_wrist_camera`
  follows robot links, `get_rgb` reads Color, depth is scalar optical depth
  from negative camera Position z. Calibration includes intrinsics/extrinsics.
- `envs/utils/actor_utils.py`: `get_functional_point` and `get_contact_point`
  multiply local geometry by exact actor pose. Consequently `grasp_actor` and
  `place_actor` are oracle helpers, unsuitable for the tested visual controller.
- `envs/occluded_socket/motion.py`: robot-only FK/IK, quintic joint motion,
  C2 Cartesian interpolation, common velocity/acceleration limits, drive update
  without a hidden physics step. Reuse with a local collision-world adapter.
- `envs/occluded_socket/observation.py`, `geometry.py`: RGB + scalar depth
  allow-list, wrist extrinsics from FK and the fixed URDF mounting transform.
  Reuse for the visual phase, after rendering works.
- Existing independent experiments under `envs/held_module_key` and
  `envs/occluded_socket` establish a local package + scripts + experiments/results
  convention. They are historical experiments, not evidence for this task.

No applicable AGENTS.md was found. Existing tracked files were clean at start.
No core changes are needed. New code belongs in `envs/active_observation_tray`,
with its own CLI and result directories.

## Scene and information boundary

Use procedural collision boxes for a wide shelf and an actual open slot,
plus a dynamic tray with a front handle and one distal peg. The rack provides
broad floor support without close lateral rails. Table coordinates use +world-y
for insertion and world-x for lateral correction (the requested task-local y).
Slot and peg marks will be on the internal mating geometry itself.

Phase 2 is explicitly oracle-only. It must execute pick, partial insertion,
lateral correction and final insertion through physical joint drives, with
no object teleport or attachment after initialization. Oracle results are not
baseline results and cannot demonstrate a sensing trade-off.

The future visual controller must receive only robot state, calibrated frames,
known nominal geometry and image-derived estimates. The evaluator owns actor
poses, true peg/slot error and contact logs. Self/helper acquisition must feed
one common insertion controller. Historical estimates must be propagated with
robot motion when self-observation moves the held tray.

## Phase gates

1. Inspect repository and verify environment.
2. Physically validate minimal geometry with explicitly labelled oracle input.
3. Validate real wrist visibility and RGB-D estimates; save frames, detections
   and evaluation-only GT projections. Do not replace rendering with synthetic
   point projections or fabricated detections.
4. Validate independent self-observation trajectory.
5. Validate helper trajectory with the same controller.
6. Add B and independent per-arm queues, driven before ONE shared scene.step.
7. Paired seeds, image ablation, pre-observation, CSV/JSON and measured ranking.

## Environment finding

Installed Python: `/mnt/sda/conda_envs/RoboTwin/bin/python`; SAPIEN 3.0.0b1,
MPLib 0.2.1, OpenCV 4.10.0. Both sandbox and escalated `nvidia-smi` fail.
`/sys/module/nvidia`, `/proc/driver/nvidia/version`, `/dev/dri` are absent.
Minimal SAPIEN RenderSystem creation also fails both inside and outside the
sandbox: `vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent`.
The installed Vulkan software ICD does not provide the extensions required by
this SAPIEN build. CPU PhysX can be attempted for Phase 2 only; it cannot provide
the required real wrist image evidence. Do not proceed past the visual gate
without restoring the rendering environment.
