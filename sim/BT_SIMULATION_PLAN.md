# rm_behavior_tree 运行仿真方案

---

## 一、行为树整体逻辑与调用链路

### 1.1 节点关系图

```
main.cpp
├── BasicControl(nh)
│   ├── CmdTools           (底盘/云台/射击命令发送器)
│   ├── Subscriber         (所有 ROS 话题订阅 + 关键发布器)
│   ├── AutoControlInfo    (决策数据中心 + 区域逻辑)
│   ├── Manual / ChassisBehavior / GimbalBehavior / ShooterBehavior
│   └── ControllerManager / CalibrationQueue
├── ConditionNode(nh, basic_control)   (只读条件判断层)
└── SimpleAction(nh, basic_control, condition_node)  (行为执行层)

BT::BehaviorTreeFactory
└── 从 config/main_tree.xml 构建行为树
    └── MainTree (Sequence)
        ├── IsRemoteControlTurnOn        [Condition]
        ├── OutputRightSwitchState       [SyncAction → 输出 state 到黑板]
        └── Switch3(state)
            ├── case "auto"  → SubTree:Auto
            ├── case "manual"→ Manual
            └── case "idle"  → Idle
```

### 1.2 Auto 子树执行顺序（每 10ms tick 一次）

```
Auto (Sequence)
 1. ReviveIfDead          → HP==0 时停控制器并返回 FAILURE（阻断后续）
 2. GetPresentTime        → present_time = 300 - stage_remain_time
 3. GetPosInMap           → TF: map → base_link
 4. GetPolygonInWhich     → 判断机器人在哪个多边形区域
 5. GetSentryCmd          → 根据区域设置 posture_cmd (attack=1/defense=2/move=3)
 6. SetPidPlannerParam    → 决定小陀螺开/关、速度限制
 7. ChassisSequence
    ├── GetChassisDecisions  → 核心决策：输出 chassis_mode_
    └── SendChassisCommand   → 按 chassis_mode_ 调用对应 chassis_behavior
 8. GimbalSequence
    ├── GetGimbalDecisions   → 核心决策：输出 gimbal_mode_
    └── SendGimbalCommand    → 按 gimbal_mode_ 调用对应 gimbal_behavior
 9. BoosterSequence
    ├── GetShooterDecisions  → 核心决策：输出 booster_mode_
    └── SendShooterCommand   → 按 booster_mode_ 调用对应 shooter_behavior
```

### 1.3 底盘模式决策树

```
referee 在线?
├─No─► game_progress==IN_BATTLE? → AbnormalBackHome : AbnormalStill
└─Yes► game_progress==IN_BATTLE?
        ├─No─► ChassisSlowGyro
        └─Yes► isSentryHpUrgent()?
                ├─Yes → GotoHpReturnArea (优先级最高)
                └─No──► 当前在 middle_area?
                        ├─Yes: central_point_state:
                        │  0→GotoMiddleArea  1→GotoMiddleArea(或ProtectMidRobot)
                        │  2/3→AttackMidRobot  default→GotoHpReturnArea
                        └─No:  central_point_state:
                           0→GotoMiddleArea  2/3→AttackMidRobot
                           1→(TrackEnemy||!chasePathFinished)&&bullets?
                              enableChase?→Chase : UnChase  : ProtectMidRobot
```

### 1.4 云台模式决策树

```
referee在线?
├─No─► AbnormalStill
└─Yes► game_progress==IN_BATTLE?
        ├─No─► YawSlowRound
        └─Yes► !isTrackLost && target 非补给站无敌?
                ├─Yes → TrackEnemy
                └─No──► chassis==AttackMidRobot && 进入未超attack_remain_time(10s)?
                        ├─Yes → FanSearchEnemy
                        └─No──► enableGimbalInverse?
                                ├─Yes → InverseGimbal
                                └─No──► RoundSearchEnemy
```

### 1.5 射击模式决策树

```
referee在线?
├─No─► Stop
└─Yes► game_progress==IN_BATTLE?
        ├─No─► Ready
        └─Yes► gimbal==TrackEnemy && track_id!=0 && target非NORMAL_FRESHLY_RESURRECTED?
                ├─Yes → Push
                └─No──► Ready
```

---

## 二、关键输入话题（驱动行为树的ROS消息）

| 话题 | 消息类型 | 作用 |
|------|----------|------|
| `/rm_ecat_hw/dbus` | `rm_msgs/DbusData` | 遥控器在线检测 + 模式拨杆(s_r) |
| `/rm_referee/game_status` | `rm_msgs/GameStatus` | 比赛进度(`game_progress`) + 剩余时间 |
| `/rm_referee/game_robot_status` | `rm_msgs/GameRobotStatus` | 机器人血量/最大血量/robot_id |
| `/rm_referee/game_robot_hp` | `rm_msgs/GameRobotHp` | 全场机器人血量 |
| `/rm_referee/event_data` | `rm_msgs/EventData` | `central_point_state`(中心点占领状态) |
| `/rm_referee/bullet_allowance_data` | `rm_msgs/BulletAllowance` | 17mm 可用弹量 |
| `/rm_referee/robot_buff` | `rm_msgs/Buff` | 机器人Buff状态 |
| `/rm_referee/rfid_status_data` | `rm_msgs/RfidStatus` | RFID区域状态(补给站) |
| `/rm_referee/radar_to_sentry` | `rm_msgs/RadarToSentry` | 雷达提供的敌人位置 |
| `/rm_referee/sentry_info` | `rm_msgs/SentryInfo` | 哨兵自主复活/补弹指令 |
| `/track` | `rm_msgs/TrackData` | 装甲板跟踪结果(目标id/position) |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `TF: map→base_link` | `tf2` | 机器人在地图中的位置 |

---

## 三、关键输出话题（观测行为树效果）

| 话题 | 消息类型 | 含义 |
|------|----------|------|
| `/behavior_tree/log` | `std_msgs/String` | 行为树节点状态日志(含C/G/B模式) |
| `/custom_info` | `std_msgs/String` | 射击攻击目标信息 |
| `/sentry_cmd` | `rm_msgs/SentryCmd` | posture_cmd: (0无/1攻/2防/3移动) |
| `/cmd_chassis` | `rm_msgs/ChassisCmd` | 底盘模式指令 |
| `/conduct_point_in_map` | `geometry_msgs/PoseStamped` | 规划目标点 |
| `/move_base_flex/GlobalPlanner/plan` | `nav_msgs/Path` | 全局路径规划路径 |
| `/map_sentry_data` | `rm_msgs/MapSentryData` | 路径数据(上报裁判系统客户端) |
| `/sentry_target_to_referee` | `rm_msgs/SentryAttackingTarget` | 攻击目标上报裁判 |

---

## 四、仿真方案设计

### 4.1 无硬件仿真架构

```
┌─────────────────────────────────────────────────────────────┐
│                    仿真运行环境                               │
│                                                             │
│  [match_simulator.py]   ──publish──►  rm_behavior_tree      │
│   比赛场景驱动器                        (待测节点)            │
│   ├── 模拟裁判系统消息                  ├── 行为决策          │
│   ├── 模拟遥控器在线                    ├── 底盘模式输出       │
│   ├── 模拟目标跟踪                      ├── 云台模式输出       │
│   └── 模拟TF坐标                        └── 射击模式输出      │
│                                                             │
│  [bt_monitor.py]        ◄─subscribe─  日志/状态话题          │
│   行为状态监控器                                              │
│   ├── 实时打印决策状态                                        │
│   ├── 记录模式切换事件                                        │
│   └── 生成仿真报告                                           │
│                                                             │
│  [fake_tf_publisher.py] ──publish──► TF树                   │
│   TF仿真发布器                                               │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 仿真场景脚本 (match_simulator.py)

仿真脚本模拟5分钟完整比赛，按以下阶段推进：

| 时间段 | present_time | 场景事件 | 预期底盘模式 | 预期云台模式 |
|--------|-------------|----------|-------------|-------------|
| 比赛开始前 | - | game_progress=WAIT | - | - |
| 0–10s | 0–10 | 开局，中心点未占，血量满 | GotoMiddleArea | YawSlowRound→RoundSearchEnemy |
| 10–30s | 10–30 | 进入中间区域，中心点state=0 | GotoMiddleArea | RoundSearchEnemy |
| 30–60s | 30–60 | 我方占领中心点(state=1)，无追踪 | ProtectMidRobot | RoundSearchEnemy |
| 60–90s | 60–90 | 出现跟踪目标，敌方state=2 | AttackMidRobot→Chase | TrackEnemy |
| 90–120s | 90–120 | 血量下降至250(<300)触发补血 | GotoHpReturnArea | RoundSearchEnemy |
| 120–150s | 120–150 | 回满血，敌方占领(state=2) | AttackMidRobot | FanSearchEnemy |
| 150–210s | 150–210 | 保守限制期，双方占领(state=3) | AttackMidRobot | TrackEnemy/FanSearch |
| 210–270s | 210–270 | 机器人死亡复活，referee离线测试 | AbnormalBackHome/ReviveFail | AbnormalStill |
| 270–300s | 270–300 | 逼近终局，残血多次补血 | GotoHpReturnArea | RoundSearchEnemy |

### 4.3 日志观测方案

行为树节点已通过 `bt_log.h` 向 `/behavior_tree/log` 发布结构化日志：

```
[BT] GetChassisDecisions SUCCESS C:GMA G:RSE B:R pos=(2.85,3.76)@middle_area Enemy:blue Time:15.00
     ^        ^              ^    ^         ^   ^   ^                          ^         ^
     前缀  节点名称         状态  底盘   云台  射击  当前坐标@区域              敌方颜色   比赛时间
```

- C: 底盘缩写  GMA=GotoMiddleArea, AMR=AttackMidRobot, PMR=ProtectMidRobot, GHRA=GotoHpReturnArea, C=Chase
- G: 云台缩写  YSR=YawSlowRound, RSE=RoundSearchEnemy, TE=TrackEnemy, FSE=FanSearchEnemy, IG=InverseGimbal  
- B: 射击缩写  S=Stop, R=Ready, P=Push

---

## 五、RMUC 2026 比赛规则与行为树对应关系

### 5.1 规则条文映射

| 规则条文 | 行为树对应逻辑 | 参数 |
|----------|--------------|------|
| 中心点占领得分 | `GetChassisDecisions`: central_point_state 0/1/2/3 决策 | `event_data.central_point_state` |
| 哨兵自动复活消耗金币 | `ReviveIfDead`: HP==0 → `confirm_respawn=1`→`sentry_cmd_pub_` | `wait_time_=0.8s` |
| 哨兵补弹消耗金币 | `isSentryHpUrgent()` → `GotoHpReturnArea` | `bake_home_high_hp_threshold=300` |
| 裁判系统断联保活 | `referee_is_online_==false` → `AbnormalBackHome/Still` | - |
| 弹药限制 | `isBulletsRemain()`:  `bullet_allowance_num_17_mm>0` | 影响 Chase 决策 |
| 无敌窗口不可攻击 | `enemy_info_.invincible_mode==IN_SUPPLY_BASE` → 不 TrackEnemy/Push | `getEnemyInvincibleMode()` |
| 进中心点开小陀螺 | `SetPidPlannerParam`: staying_area=="middle_area" → gyro ON | `move_gyro_vel=8.0` |
| 雷达标记敌人位置 | `GetEnemyInfo`: `radar_to_sentry_info_` → 地图标记 | `radar_to_sentry` topic |
| 速度限制 | `pid_local_planner` 速度限制服务 | `default_limit_vel=6.0` |

### 5.2 通信协议字段映射（裁判系统 → ROS）

| 裁判协议字段 | ROS消息 | 字段名 |
|-------------|---------|--------|
| `0x0001 game_status` game_progress | `rm_msgs/GameStatus` | `game_progress` (IN_BATTLE=4) |
| `0x0001` stage_remain_time | `rm_msgs/GameStatus` | `stage_remain_time` |
| `0x0003 game_robot_hp` | `rm_msgs/GameRobotHp` | `red_1`..`blue_7` |
| `0x0101 game_robot_status` remain_hp | `rm_msgs/GameRobotStatus` | `remain_hp` |
| `0x0101` robot_id | `rm_msgs/GameRobotStatus` | `robot_id` (红方<100, 蓝方≥100) |
| `0x0103 event_data` central_point | `rm_msgs/EventData` | `central_point_state` |
| `0x0108 bullet_remaining` | `rm_msgs/BulletAllowance` | `bullet_allowance_num_17_mm` |
| `0x0105 rfid_status` | `rm_msgs/RfidStatus` | `nan_overlapping_supplier_zone` |
| `0x020E sentry_info` | `rm_msgs/SentryInfo` | `sentry_autonomous_...` |
| `0x0120 robot_buff` | `rm_msgs/Buff` | 各Buff位标志 |
| `0x020C radar_to_sentry` | `rm_msgs/RadarToSentry` | `robot_ID`, `position_x/y` |
| 哨兵上报指令`0x0121` | `rm_msgs/SentryCmd` | `posture_cmd`, `confirm_respawn` |
| 地图路径数据`0x0305` | `rm_msgs/MapSentryData` | `start_position_x/y`, `delta_x/y` |
