# new_behavior_tree 项目评估报告

> 评估时间：2026-06-09
> 评估范围：完整项目代码审查
> 基于 XML 行为树配置：`config/untitled_1.xml`

---

## 一、项目概述

基于 **BehaviorTree.CPP 4.x** 框架的 RoboMaster 哨兵机器人决策系统。项目通过行为树组织哨兵的自动/手动/空闲逻辑，通过 ROS 与底层硬件、裁判系统、感知模块、导航模块交互。

### 技术栈

| 组件 | 技术 |
|------|------|
| 框架 | BehaviorTree.CPP 4.x |
| ROS | ROS 1 (Noetic) |
| 控制 | ros_control + rm_common |
| 导航 | move_base_flex (MBF) |
| 感知 | 前后目检测、雷达、裁判系统 |
| 调试 | Groot2 在线监控 (端口 5555) |

### 节点规模

| 类别 | 数量 |
|------|------|
| 条件节点 (Condition) | 22 |
| 通用动作节点 (Common Action) | 8 |
| 底盘动作节点 (Chassis Action) | 18 |
| 云台动作节点 (Gimbal Action) | 4 |
| 射击动作节点 (Shooter Action) | 4 |
| 手动控制 (Manual) | 1 + 1 辅助类 |
| **BT 节点总数** | **57** |

---

## 二、架构评估

### 2.1 整体架构

```
main() 50Hz 循环
  |
  +-- tree.tickExactlyOnce()              <- 行为树驱动
  +-- controller_tools.ControllerUpdate()  <- 控制器更新
  |
  +-- SentryParamLoader             <- 启动时加载所有参数到黑板
  +-- Subscriber (~20个话题)        <- 感知数据订阅，互斥锁保护
  +-- TfAccessor                    <- TF 坐标变换封装
  +-- Publisher (~8个话题)          <- 数据发布
  +-- Tools Layer                   <- 业务逻辑封装层
       +-- CmdTools                 <- 指令发送器管理
       +-- NavigationTools          <- MBF 导航客户端
       +-- ControllerTools          <- ros_control 控制器管理
       +-- GimbalTools              <- 云台控制逻辑
       +-- PlannerTools             <- 全局规划器参数 + 陀螺服务
       +-- MiniMapTools             <- 小地图坐标变换
```

### 2.2 架构优点

- **清晰的层次分离**：行为树节点 (BT Node) -> 工具层 (Tools) -> 感知层 (Perception)，职责明确
- **ReactiveSequence 驱动**：50Hz 循环 + ReactiveSequence 保证每个 tick 重新评估条件，响应及时
- **互斥锁保护完整**：`Subscriber` 中所有跨线程数据访问都有 `std::mutex` 或 `std::atomic` 保护
- **工具层设计合理**：`CmdTools` 统一管理所有指令发送器，避免重复创建 ROS 客户端

### 2.3 架构隐患

| # | 问题 | 说明 |
|---|------|------|
| 1 | **PlannerTools 多继承** | `ServiceCallerBase<SetLimitVel>` + `ServiceCallerBase<EnableGyro>`，两个 base 各自管理独立后台线程 + ROS ServiceClient，构造时必须确保都正确传参 |
| 2 | **Subscriber 回调内直接操作 cmd_tools_** | DBUS / Track 等回调中调用 `cmd_tools_` 更新，回调线程 (AsyncSpinner 4) 与主循环存在隐式数据竞争 |
| 3 | **main.cpp 构建顺序有隐式依赖** | `planner_tools(bt_nh)` 需要服务端就绪，构造时服务不一定已启动 |

---

## 三、当前 mainTree 逻辑评估（基于 XML）

### 3.1 行为树结构

```xml
<BehaviorTree ID="mainTree">
  <ReactiveSequence>
    +-- <ReactiveFallback>
    |    +-- IsRemoteControlTurnOn      <- 遥控器在线？
    |    +-- RemoteControlTurnOff       <- 不在则全置零
    +-- <IfThenElse>
    |    +-- IsRefereeOnline            <- 裁判系统在线？
    |    +-- VisionCalibrate            <- 视觉校准
    |    +-- AlwaysSuccess              <- 离线则跳过
    +-- OutputRightSwitchState         <- 读拨杆状态 -> {state}
    +-- <Switch3 state={state}>
         +-- [auto]   Test1            <- 占位
         +-- [manual] Parallel{Chassis, Gimbal, Shooter}
         +-- [idle]   SetIdle
         +-- [default] SetIdle
  </ReactiveSequence>
</BehaviorTree>
```

### 3.2 各路径逻辑审计

#### Auto 模式（`Test1`）

| 项 | 状态 |
|------|------|
| cmd_chassis 更新 | **未更新** |
| cmd_vel 更新 | **未更新** |
| 云台指令 | **未更新** |
| 射击指令 | **未更新** |
| 控制器状态 | **未管理** |

**风险**：最后一个 manual/idle 指令会持续残留，机器人可能继续运动。

#### Manual 模式（`Parallel{ManualSendChassisCmd, GimbalCmd, ShooterCmd}`）

| 项 | 状态 |
|------|------|
| cmd_chassis 更新 | 每 tick 更新 |
| cmd_vel 更新 | 每 tick 更新 |
| 云台指令 | 每 tick 更新 |
| 射击指令 | 每 tick 更新 |
| 保护逻辑 | wheel>0.01 切 RAW，否则 FOLLOW |

#### Idle 模式（`SetIdle`）

| 项 | 状态 |
|------|------|
| cmd_chassis 更新 | 每 tick 更新 (mode=RAW, frame=base_link) |
| cmd_vel 更新 | **未显式归零** |
| 云台控制器 | 已停止 |
| 陀螺 | setGyroSpeed(0) |

#### RemoteControlTurnOff（遥控断开保护）

| 项 | 状态 |
|------|------|
| cmd_chassis | 置零发送 |
| cmd_vel | setZero() 置零发送 |
| 云台 | setZero() 发送 |
| 射击 | setZero() 发送 |
| 控制器 | 停止主控制器 + 校准控制器 |

### 3.3 状态切换路径安全评估

| 切换路径 | 风险 |
|----------|------|
| manual -> auto | **中** -- auto 无指令输出，残留 manual 最后指令 |
| auto -> manual | **低** -- OutputRightSwitchState 会 cancel MBF goal + reset patrol |
| manual -> idle | **低** -- SetIdle 覆盖 chassis 指令，cmd_vel 未归零 |
| idle -> manual | **低** -- ManualSendChassisCmd 立刻接管 |
| 遥控断开 | **低** -- RemoteControlTurnOff 全置零 |
| 裁判系统离线 | **低** -- VisionCalibrate 被 IfThenElse 跳过 |

---

## 四、代码质量问题

### 4.1 严重问题

| # | 文件:行 | 问题 |
|---|---------|------|
| 1 | `gimbal_action_node.h:109-113` | **`InverseGimbal::onStart()` 缺少 return** -- 未定义行为，可能随机 crash |
| 2 | `condition_node.h:266-288` | `IsBulletsRemain` 中 `bullet_allowance_num_17_mm < 2000 == true` 无语法错误但冗余 |

### 4.2 中等问题

| # | 文件:行 | 问题 | 说明 |
|---|---------|------|------|
| 1 | `register_node.h` | `register_node` 函数签名 12 个参数 | 可读性和可维护性差 |
| 2 | `sentry_param_loader.h:307` | `load_robot_color` 用 `ROS_ASSERT` 校验颜色 | 参数错误直接 abort |
| 3 | `chassis_action_node.h:491` | `ChaseEnemy` TODO | 追击优先级判断未实现 |
| 4 | `perception_layer.cpp:7` | 构造函数使用全局 `NodeHandle` 而非传入的 `bt_nh` | 话题名可能不在预期命名空间下 |

### 4.3 轻微问题

| # | 文件:行 | 问题 |
|---|---------|------|
| 1 | `condition_node.h` | 5 个节点为桩实现（恒 SUCCESS）：`IsNeedAvoidDrone`, `IsNeedDefenseBase`, `IsDefenseBuffBelowTheThreshold`, `IsInOwnHalfArea`, `IsTargetNotInvincible` |
| 2 | `gimbal_action_node.h:147` | `TrackEnemy::onRunning()` 返回 `SUCCESS` 而非 `RUNNING`，语义不当但运行正常 |
| 3 | `sentry_param_loader.h:17` | 作者自注 `//TODO : 该文件有较大漏洞，需重新编写` |
| 4 | `perception_layer.h:235-238` | 互斥锁声明在同一行，可读性差 |
| 5 | `action_node.h:84` | 多余空行 |

### 4.4 跨线程数据竞争风险

`Subscriber` 回调函数中直接操作 `cmd_tools_`（ROS AsyncSpinner 4 线程 vs 主循环 50Hz）：

| 回调 | 操作对象 | 保护 |
|------|---------|------|
| `dbusCallback` | `chassis_command_sender_->updateRefereeStatus()` | 无 |
| `trackCallback` | `shooter_command_sender_->updateTrackData()` | 无 |
| `gameRobotStatusCallback` | `chassis_command_sender_->updateGameRobotStatus()` | 无 |
| `powerHeatDataCallback` | `chassis_command_sender_->updatePowerHeatData()` | 无 |
| `capacityDataCallback` | `chassis_command_sender_->updateCapacityData()` | 无 |

---

## 五、命令发布链路评估

### 5.1 cmd_chassis 完整链路

```
BT Node tick()
  -> vel_2d_command_sender_->setXXX()    设置速度
  -> chassis_command_sender_->sendChassisCommand(time, is_gyro)
    -> power_limit_->setLimitPower()     功率限制
    -> msg_.accel = 斜坡加速度
    -> pub_.publish(msg_)                -> /cmd_chassis
  -> vel_2d_command_sender_->sendCommand(time)
    -> pub_.publish(msg_)                -> /cmd_vel
```

底盘控制器 (`chassis_base.cpp`) 同时消费 `/cmd_chassis` 和 `/cmd_vel`：
- `/cmd_chassis` -> mode, accel, power_limit, command_source_frame
- `/cmd_vel` -> 线速度/角速度目标值

两个 topic 独立发布，如果漏发其一，控制器读到**旧值**。

### 5.2 各路径发布覆盖

| 路径 | cmd_chassis | cmd_vel | 云台指令 | 射击指令 |
|------|:-----------:|:-------:|:--------:|:--------:|
| auto (Test1) | x | x | x | x |
| manual (Parallel) | v | v | v | v |
| idle (SetIdle) | v | x | x | x |
| RemoteControlTurnOff | v | v | v | v |

---

## 六、TODO / 已知空缺

| 项目 | 说明 | 优先级 |
|------|------|:------:|
| **Auto 模式行为树** | 当前为 `AlwaysSuccess`/`Test1` 占位 | **高** |
| `ChaseEnemy` 优先级判断 | 追击决策逻辑未实现 | 中 |
| `IsTargetNotInvincible` | 无敌检测算法需重写 | 中 |
| `IsNeedAvoidDrone` | 无人机规避逻辑未实现 | 低 |
| `IsNeedDefenseBase` | 基地防守逻辑未实现 | 低 |
| `IsDefenseBuffBelowTheThreshold` | 防御 buff 检测未实现 | 低 |
| `IsInOwnHalfArea` | 半场判断未实现 | 低 |
| `SentryParamLoader` 代码重构 | 作者自评"有较大漏洞，需重新编写" | 中 |
| `setNavigationTools` TODO | 需要优化实现方式 | 低 |
| `YawSlowRound/InverseGimbal/TrackEnemy` | `onHalted()` 为空 | 低 |

---

## 七、风险评估总结

### 当前可运行的场景

| 场景 | 结论 |
|------|------|
| 遥控手动操作 | 安全，全部命令每 tick 更新 |
| 遥控断开 | 安全，全置零 |
| Idle 待机 | 基本安全，cmd_chassis 更新但 cmd_vel 未归零 |
| 裁判系统离线 | VisionCalibrate 被跳过，不影响运动 |

### 需关注的风险

1. **Auto 模式无指令输出** -- 最后一个 manual/idle 指令残留，是最大的不安全因素
2. **SetIdle 未归零 cmd_vel** -- 可能导致 idle 模式下底盘仍按旧速度运动
3. **回调线程数据竞争** -- ROS 回调直接写 `cmd_tools_`，主循环 50Hz 读取，可能读到不一致状态
4. **InverseGimbal::onStart() 缺 return** -- 未定义行为，可能随机 crash

---

## 八、改进建议

### P0 -- 必须修复

- [ ] Auto 树先用 `SetIdle` 兜底，防止无指令输出
- [ ] `InverseGimbal::onStart()` 补上 `return BT::NodeStatus::RUNNING;`

### P1 -- 建议修复

- [ ] `SetIdle` 中补上 `vel_2d_command_sender_->setZero()` + `sendCommand()`
- [ ] 评估回调线程直接操作 `cmd_tools_` 的并发安全性，考虑加锁或延迟更新
- [ ] XML 中 `VisionCalibrate` 补上 `robot_color="{robot_color}"` 端口绑定

### P2 -- 可优化

- [ ] `register_node()` 参数过多，考虑用结构体聚合
- [ ] `onHalted()` 回调实现清理逻辑（停止云台/射击）
- [ ] 导航节点补充执行阶段超时保护
- [ ] `SentryParamLoader` 按计划重构，`ROS_ASSERT` 改为优雅报错+默认值

---

*本文档由代码审查自动生成于 2026-06-09*
