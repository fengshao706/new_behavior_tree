# Behavior Tree Node Reference

> 本文档对 `include/` 下所有行为树节点进行分类整理，涵盖条件节点、动作节点及基础设施模块。

---

## 目录

1. [Condition Nodes（条件节点）](#1-condition-nodes条件节点)
2. [Common Action Nodes（通用动作节点）](#2-common-action-nodes通用动作节点)
3. [Chassis Action Nodes（底盘动作节点）](#3-chassis-action-nodes底盘动作节点)
4. [Gimbal Action Nodes（云台动作节点）](#4-gimbal-action-nodes云台动作节点)
5. [Shooter Action Nodes（射击动作节点）](#5-shooter-action-nodes射击动作节点)
6. [Manual Action Nodes（手动操作节点）](#6-manual-action-nodes手动操作节点)
7. [Infrastructure（基础设施）](#7-infrastructure基础设施)

---

## 1. Condition Nodes（条件节点）

**命名空间:** `condition_node` | **基类:** `BT::ConditionNode`
**文件:** `include/behavior_tree/condition_node.h`

### 裁判系统与比赛状态

| 节点 | 功能 | 端口 |
|------|------|------|
| `IsRefereeOnline` | 裁判系统在线时返回 SUCCESS | 无 |
| `IsGameInBattle` | 比赛进行中（IN_BATTLE）返回 SUCCESS | 无 |
| `IsTimeRangeCondition` | 当前游戏时间在 `[min_time, max_time]` 内返回 SUCCESS | `min_time`, `max_time`, `game_total_time` |

### 机器人自身状态

| 节点 | 功能 | 端口 |
|------|------|------|
| `IsSentryHpUrgent` | 哨兵血量低于 `trigger_blood_return_hp` 阈值返回 SUCCESS（默认 30） | `trigger_blood_return_hp` |
| `IsSentryHpReturnMax` | 哨兵血量为满时返回 SUCCESS | 无 |
| `IsBulletsRemain` | 17mm 弹丸剩余量在 (0, 2000) 区间内返回 SUCCESS | 无 |
| `IsRemoteControlTurnOn` | DBUS 数据时间戳在 1s 内（遥控器开启）返回 SUCCESS | 无 |
| `IsNeedInverseGimbal` | 后视检测到有效目标时返回 SUCCESS，触发云台反转 | `default_aim_rank` |

### 目标检测与跟踪

| 节点 | 功能 | 端口 |
|------|------|------|
| `IsTargetEffective` | 跟踪目标 ID 非零时返回 SUCCESS | 无 |
| `IsTrackLoss` | 距离上次跟踪时间超过 `lost_track_tolerant_sec` 返回 SUCCESS | `last_track_time`, `lost_track_tolerant_sec` |
| `CheckTargetType` | 当前跟踪目标的 ID 匹配 `track_id` 时返回 SUCCESS | `track_id` |
| `CheckGimbalMode` | 当前云台模式与 `expected_gimbal_mode` 匹配时返回 SUCCESS | `gimbal_mode`, `expected_gimbal_mode` |
| `IsTargetNotInvincible` | **TODO** — 目标是否非无敌状态（未实现，恒返回 SUCCESS） | 无 |

### 友军与防御建筑

| 节点 | 功能 | 端口 |
|------|------|------|
| `IsEngineerAlive` | 友方工程机器人血量 > 0 返回 SUCCESS | 无 |
| `IsOutpostAlive` | 友方前哨站血量 > 0 返回 SUCCESS | 无 |
| `IsHasEngineerMarked` | 雷达标记了工程机器人时返回 SUCCESS | 无 |
| `IsOwnOutpostHpBeyondTheValue` | 友方前哨站血量超过 `outpost_hp_threshold` 返回 SUCCESS（默认 800） | `outpost_hp_threshold` |
| `IsDefenseBuffBelowTheThreshold` | **TODO** — 防御 buff 低于阈值（未实现，恒返回 SUCCESS） | `defense_buff_threshold` |

### 区域与位置

| 节点 | 功能 | 端口 |
|------|------|------|
| `IsInOwnHalfArea` | **TODO** — 是否在我方半场（未实现，恒返回 SUCCESS） | 无 |
| `IsNeedAvoidDrone` | **TODO** — 是否需要规避无人机（未实现，恒返回 SUCCESS） | 无 |
| `IsNeedDefenseBase` | **TODO** — 是否需要防守基地（未实现，恒返回 SUCCESS） | 无 |
| `IsClientMapUpdate` | 客户端地图已更新时返回 SUCCESS | 无 |

> 共计 **22** 个条件节点

---

## 2. Common Action Nodes（通用动作节点）

**命名空间:** 全局 | **基类:** `BT::SyncActionNode`
**文件:** `include/behavior_tree/common/action_node.h`

| 节点 | 功能 | 端口 |
|------|------|------|
| `StartMainControllers` | 启动主控制器并校准，睡眠 1s | 无 |
| `StopMainControllers` | 停止主控制器，睡眠 0.5s | 无 |
| `VisionCalibrate` | 设置敌方颜色以及前后摄像头检测目标类型 | 无 |
| `RemoteControlTurnOff` | 将所有指令发送器置零并发送（底盘/云台/射击） | 无 |
| `OutputRightSwitchState` | 读取 DBUS 右开关状态并输出：`manual`(MID)、`auto`(UP)、`idle`(DOWN)；进入 manual 时取消 MBF 导航 | `state` |
| `SetIdle` | 启动主控制器（停止云台控制器）、校准、关闭陀螺仪、设置底盘为 RAW 模式 | 无 |
| `Test1` | 调试节点，每 0.5s 打印 "Test1" | 无 |
| `Test2` | 调试节点，每 0.5s 打印 "Test2" | 无 |

> 共计 **8** 个通用动作节点

---

## 3. Chassis Action Nodes（底盘动作节点）

**命名空间:** `chassis` | **基类:** `BT::SyncActionNode` / `BT::StatefulActionNode`
**文件:** `include/behavior_tree/chassis_action_node.h`

### 导航巡逻（StatefulActionNode，持续运行）

| 节点 | 功能 |
|------|------|
| `PatrolAbnormalBackHomeGoal` | 导航回到己方中心哨兵巡逻区 |
| `PatrolAttackEnemyPositiveArea` | 导航到敌方正区域进攻 |
| `PatrolOwnOutpostArea` | 巡逻己方前哨站区域 |
| `PatrolEnemyOutpostArea` | 巡逻敌方前哨站区域 |
| `PatrolSentryPatrolArea` | 巡逻中心哨兵巡逻区 |
| `GotoReturnBloodArea` | 前往补给区回血 |
| `PatrolHoleUpArea` | 前往埋伏点 |
| `GotoEnemyBaseArea` | 前往敌方基地 |
| `GotoAttackEnemyEngineer` | 前往敌方工程机器人预期位置 |
| `GotoConductPoint` | 导航到操作手指引点（通过 MiniMapTools） |
| `ChaseEnemy` | 持续追击跟踪目标（**TODO: 优先级判断未完成**） |

### 状态控制（SyncActionNode，单次执行）

| 节点 | 功能 | 端口 |
|------|------|------|
| `ChassisSlowGyro` | 设置底盘 RAW 模式并以 `slow_gyro_vel_scale` 速度慢速旋转 | `slow_gyro_vel_scale` |
| `AbnormalStillStopAllMotion` | 紧急停止：将所有速度/云台/底盘指令置零并发送 | 无 |
| `SetChassisMode` | 读取 `chassis_mode_id` 并将对应 `ChassisMode` 枚举写入黑板 | `chassis_mode_id` |
| `SetIsEnableFight` | 设置战斗使能标志 | `is_enable_fight` |
| `SetGyroInCombat` | 基于 `standby_velocity` 参数使用正弦模式设置战斗陀螺速度 | `standby_velocity` |
| `ReviveIfDead` | 检测死亡(HP=0)，停止控制器，复活后延时 0.8s 重置校准；设置虚弱/复活标志 | `self_is_weak`, `self_weak_until`, `need_supply`, `has_revived`, `confirm_respawn` |
| `GetKeyboardCommand` | 从客户端地图数据中获取键盘指令 | `keyboard_command` |

> 共计 **18** 个底盘动作节点

---

## 4. Gimbal Action Nodes（云台动作节点）

**命名空间:** `gimbal` | **基类:** `BT::SyncActionNode` / `BT::StatefulActionNode`
**文件:** `include/behavior_tree/gimbal_action_node.h`

| 节点 | 类型 | 功能 | 端口 |
|------|------|------|------|
| `SetGimbalMode` | Sync | 读取 `gimbal_mode_id` 并将对应 `GimbalMode` 枚举写入黑板 | `gimbal_mode_id` |
| `YawSlowRound` | Stateful | 控制云台 yaw 慢速扫射 + pitch 在 `[pitch_min, pitch_max]` 间摆动，持续运行 | `yaw_vel`, `scan_range_circles`, `pitch_inside_vel`, `pitch_outside_vel`, `pitch_min`, `pitch_max`, `breach_thresholds` |
| `InverseGimbal` | Stateful | 将后视检测结果转换到地图坐标系并指向目标（云台反转），持续运行 | 无 |
| `TrackEnemy` | Stateful | 设置云台跟踪模式并更新跟踪信息，单次执行返回 SUCCESS | 无 |

> 共计 **4** 个云台动作节点

---

## 5. Shooter Action Nodes（射击动作节点）

**命名空间:** `shooter` | **基类:** `BT::SyncActionNode` / `BT::StatefulActionNode`
**文件:** `include/behavior_tree/shooter_action_node.h`

| 节点 | 类型 | 功能 |
|------|------|------|
| `SetShooterMode` | Sync | 将 `shooter_mode_id_input` 拷贝到 `shooter_mode_id_output`（端口透传） |
| `ShooterStop` | Stateful | 设置射击为 STOP 模式并发送指令，持续运行 |
| `ShooterReady` | Stateful | 设置射击为 READY 模式，halt 时切换为 STOP，持续运行 |
| `ShooterPush` | Stateful | 设置射击为 PUSH（开火）模式，halt 时切换为 STOP，持续运行 |

> 共计 **4** 个射击动作节点

---

## 6. Manual Action Nodes（手动操作节点）

**命名空间:** `manual` | **基类:** `BT::StatefulActionNode`
**文件:** `include/behavior_tree/manual_action_node.h`

| 节点/类 | 类型 | 功能 |
|---------|------|------|
| `RemoteControlTurnOff` | BT StatefulActionNode | 将速度/云台/射击指令全部置零，持续运行 |
| `SimpleAction` | 辅助工具类（非 BT 节点） | 提供 `sendChassisCmd()` / `sendGimbalCmd()` / `sendShooterCmd()` 方法，处理基于 DBUS 摇杆的手动控制、陀螺减速、跟踪云台模式及射击触发逻辑 |

> 共计 **1** 个 BT 节点 + 1 个辅助类

---

## 7. Infrastructure（基础设施）

### perception_layer.h

| 类 | 命名空间 | 功能 |
|---|---------|------|
| `Subscriber` | `perception` | ROS 主题订阅中心，订阅约 20+ 个主题（DBUS、裁判系统、跟踪、相机检测、雷达、规划器等），所有数据访问带互斥锁保护 |
| `TfAccessor` | `perception` | TF2 坐标变换查询封装，提供 `FrameId` 枚举（MAP, BASE_LINK, BACK_CAMERA, ODOM, YAW, PITCH 等） |
| `Publisher` | `perception` | 管理 ROS 发布器：地图哨兵数据、瞄准优先级、哨兵状态/指令、指引点、攻击目标、Marker 等 |

### common/tools.h

| 类 | 命名空间 | 功能 |
|---|---------|------|
| `EnableGyroServiceCaller` | `tools` | 陀螺仪使能/禁用 ROS 服务调用 |
| `SetLimitVelServiceCaller` | `tools` | 速度限制 ROS 服务调用 |
| `CmdTools` | `tools` | 指令发送管理器，管理底盘/云台/射击/速度指令发送器，以及动态调参/PID/TF 等 |
| `MiniMapTools` | `tools` | 小地图-世界坐标转换，导航路径变换与指引点获取 |
| `NavigationTools` | `tools` | MBF 导航客户端，支持顺序/随机巡逻、追击、导航状态管理 |
| `ControllerTools` | `tools` | ros_control 控制器生命周期管理（启动/停止/校准） |
| `GimbalTools` | `tools` | 云台控制逻辑：yaw/pitch 扫描、pitch 摆动方向、地图坐标指向、跟踪模式等 |
| 自由函数 | `tools` | `getZonesPosition()` 获取区域导航点，`isPointInPolygon()` 点-多边形测试，`determinePolygonInWhich()` 确定点所在区域 |

### common/types.h

| 类型 | 命名空间 | 说明 |
|------|---------|------|
| `ChassisMode` | `types` | 19 种底盘模式枚举（慢陀螺、急停、回老家、进攻、巡逻、追击等） |
| `GimbalMode` | `types` | 9 种云台模式枚举（慢扫、反转、跟踪、搜敌等） |
| `ShooterMode` | `types` | 3 种射击模式（停止、就绪、开火） |
| `InvincibleMode` | `types` | 5 种无敌模式（可受伤、检测中、补给区、刚复活等） |
| `RobotType` | `types` | 机器人类型（英雄、工程、步兵、哨兵、前哨站、基地等） |
| `SentryIntention` | `types` | 哨兵意图（进攻、防守、移动） |
| `ENEMY_INFO` 结构体 | `types` | 敌方信息：上次治疗/复活时间、无敌模式、瞄准优先级、血量 |
| `CHASE_JUDGE` 结构体 | `types` | 追击配置：追击限制区域、前哨站血量阈值 |

### 其他基础设施

| 文件 | 类/函数 | 功能 |
|------|---------|------|
| `common/sentry_param_loader.h` | `SentryParamLoader` | 从 ROS 参数服务器加载全部配置并写入 BT 黑板（底盘/云台/导航/区域/颜色等） |
| `register_node.h` | `register_node()` | 将所有 BT 节点注册到 `BT::BehaviorTreeFactory`，配置 Groot2 监控并注入所有依赖 |

---

## 统计汇总

| 类别 | 数量 |
|------|------|
| Condition Nodes（条件节点） | 22 |
| Common Action Nodes（通用动作节点） | 8 |
| Chassis Action Nodes（底盘动作节点） | 18 |
| Gimbal Action Nodes（云台动作节点） | 4 |
| Shooter Action Nodes（射击动作节点） | 4 |
| Manual Action Nodes（手动操作节点） | 1 + 1 辅助类 |
| Infrastructure（基础设施类/结构体） | 14+ |
| **BT 节点总数** | **57** |
