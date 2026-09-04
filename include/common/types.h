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

}

#endif //NEW_BEHAVIOR_TREE_TYPES_H