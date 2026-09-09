//
// Created by root on 2026/3/6.
//

#ifndef NEW_BEHAVIOR_TREE_TYPES_H
#define NEW_BEHAVIOR_TREE_TYPES_H

#include "ros/ros.h"

namespace types
{
  enum class ChassisMode
  {
    ChassisSlowGyro = 0,
    AbnormalStill = 1,
    AbnormalBackHome = 2,
    GotoFirstArea = 3,
    GotoOwnOutpost = 4,
    GotoEnemyOutpost = 5,
    GotoSentryPatrolArea = 6,
    GotoHpReturnArea = 7,
    GotoHoleUpArea = 8,
    GotoAttackEngineer = 9,
    GotoConductPoint = 10,
    GotoConductPointAndStand = 11,
    PatrolAnyArea = 12,
    GotoBaseDefenseArea = 13,
    GotoEnemyBase = 14,
    ConductAbnormalGyro = 15,
    AvoidDrone = 16,
    Chase = 17,
    UnChase = 18,
    GotoHitEnemyOutpostArea = 19 ,
    GotoTrapezoidalHighland = 20 ,
    GotoOwnFortress = 21

  };

  enum class GimbalMode
  {
    YawSlowRound,
    AbnormalStill,
    LidarTowardsFront,
    RoundSearchEnemy,
    FanSearchEnemy,
    InverseGimbal,
    AimOutpost,
    AimBase,
    TrackEnemy
  };
  enum class ShooterMode
  {
    Stop,
    Ready,
    Push
  };

  enum class InvincibleMode
  {
    INJURABLE,
    CHECKING,
    IN_SUPPLY_BASE,
    NORMAL_FRESHLY_RESURRECTED,
    MONEY_FRESHLY_RESURRECTED
  };

  enum class RobotType
  {
    HERO = 1,
    ENGINEER,
    STANDARD_3,
    STANDARD_4,
    STANDARD_5,
    OUTPOST,
    SENTRY,
    BASE
  };

  enum class SentryIntention
  {
    AttackAtTheTargetPoint = 1,
    DefendAtTheTargetPoint ,
    MoveToTheTargetPoint
  };

  typedef struct
  {
    ros::Time last_heal_time;
    ros::Time last_revive_time;
    InvincibleMode invincible_mode;
    int aim_priority;
    int hp;
    int revive_hp;
  } ENEMY_INFO;

  // 无敌/生命状态（与 invincible_detection 对齐，types.h 自包含，invincible_detection 经 alias 复用）
  enum class EnemyInvincibleState
  {
    UNKNOWN,           // 无确认信息（雷达 HP 一帧都还没来，或 index 越界）
    ALIVE,             // 已确认存活（连续 confirm_samples 帧 HP>0）
    DEAD,              // 已确认阵亡（连续 confirm_samples 帧 HP<=0）
    REVIVE_INVINCIBLE, // 复活无敌期（打了不扣血）
    REGION_INVINCIBLE  // 区域无敌（敌方补给区 / 工程交换区）
  };

  // 单个机器人的可攻击性快照（= invincible_detection snapshot 返回值）
  struct EnemyInvincibleInfo
  {
    EnemyInvincibleState state{EnemyInvincibleState::UNKNOWN}; // 去抖后的生命状态
    ros::Time revive_invincible_until;                         // 复活无敌截止时刻（未复活时为零）
    int hp{0};                                                 // 最近一帧 HP（已下限钳到 0）
  };

  // 机器人名称 → 状态 映射表，即最终经黑板发布的数据类型
  struct EnemyInvincibleTable
  {
    EnemyInvincibleInfo hero;
    EnemyInvincibleInfo engineer;
    EnemyInvincibleInfo infantry_3;
    EnemyInvincibleInfo infantry_4;
    EnemyInvincibleInfo aerial; // 5号：无人机（雷达 `reserved` 字段）
    EnemyInvincibleInfo sentry; // 6号：哨兵
  };

}

#endif //NEW_BEHAVIOR_TREE_TYPES_H