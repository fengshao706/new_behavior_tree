# Posture Demo Sequence

这个脚本现在只发送裁判系统信息，不再发送：

- `/track`
- `TF`
- `/odom`
- `/rm_ecat_hw/dbus`

因此它的作用变成：

- 给 `rm_behavior_tree` 提供稳定的 `/rm_referee/*` 输入
- 让你把 TF、目标跟踪、遥控器输入交给别的节点或手工工具
- 单独控制裁判字段的切换节奏

## 运行

```bash
source /home/ROS_NOETIC_WS/rm_ws/devel/setup.bash
python3 /home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/posture_demo_driver.py
```

如果只想看时间表：

```bash
python3 /home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/posture_demo_driver.py --print-plan
```

## 脚本持续发送什么

脚本只会持续发送这些 `/rm_referee/*` 话题：

- `/rm_referee/game_status`
- `/rm_referee/game_robot_status`
- `/rm_referee/game_robot_hp`
- `/rm_referee/event_data`
- `/rm_referee/bullet_allowance_data`
- `/rm_referee/rfid_status_data`
- `/rm_referee/power_heat_data`

裁判系统固定基线：

- `power_heat_data.stamp = now`
- `game_progress = 4 (IN_BATTLE)`
- `robot_id = 7`
- `remain_hp = 400`
- `bullet_allowance_num_17_mm = 240`

阶段切换只改裁判字段：

1. `event_data.central_point_state`
2. `game_status.game_progress`
3. `game_robot_status.remain_hp`
4. `bullet_allowance_data.bullet_allowance_num_17_mm`

## 时间表

默认每阶段 `8s`。

1. `0s - 8s`
   - `game_progress = 4`
   - `central_point_state = 0`
   - `remain_hp = 400`
   - `bullets = 240`

2. `8s - 16s`
   - `game_progress = 4`
   - `central_point_state = 1`
   - `remain_hp = 400`
   - `bullets = 240`

3. `16s - 24s`
   - `game_progress = 4`
   - `central_point_state = 2`
   - `remain_hp = 400`
   - `bullets = 240`

4. `24s - 32s`
   - `game_progress = 4`
   - `central_point_state = 3`
   - `remain_hp = 400`
   - `bullets = 240`

5. `32s - 40s`
   - `game_progress = 4`
   - `central_point_state = 0`
   - `remain_hp = 400`
   - `bullets = 240`

## 限制

因为这个脚本不再发送 TF 和 `/track`，所以它本身不能单独演示完整姿态切换。

如果你还想看到：

- `posture=3 -> 2 -> 1 -> 2 -> 3`

那还需要你额外提供：

- 自车位置 / TF
- `/track`
- 如有需要，再提供 `/rm_ecat_hw/dbus`

## 展示时看什么

脚本会打印两类输出：

- 当前阶段与当前发送的裁判字段
- `/sentry_cmd` 实际姿态切换

如果 `rm_behavior_tree` 正在发布 `/behavior_tree/log`，脚本也会同步打印：

- `GetSentryCmd`
