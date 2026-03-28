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
    ChassisSlowGyro,
    AbnormalStill,
    AbnormalBackHome,
    GotoFirstArea,
    GotoOwnOutpost,
    GotoEnemyOutpost,
    GotoSentryPatrolArea,
    GotoHpReturnArea,
    GotoHoleUpArea,
    GotoAttackEngineer,
    GotoConductPoint,
    GotoConductPointAndStand,
    PatrolAnyArea,
    GotoBaseDefenseArea,
    GotoEnemyBase,
    ConductAbnormalGyro,
    AvoidDrone,
    Chase,
    UnChase
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

  enum class ControlState
  {
    CHASE_MODE = 1,
    GIMBAL_CONTROL_MODE ,
    DEFAULT_NAVIGATION_MODE
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

  typedef struct
  {
    std::vector<std::string> chase_restricted_zone;
    int outpost_hp_threshold;
  } CHASE_JUDGE;

}

#endif //NEW_BEHAVIOR_TREE_TYPES_H