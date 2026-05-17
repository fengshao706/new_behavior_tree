//
// Created by root on 2026/3/29.
//

#include "register_node.h"

namespace register_node
{
  void register_node(ros::NodeHandle &bt_nh, double &wait_time ,BT::Blackboard::Ptr& blackboard, tools::CmdTools& cmd_tools, perception::Subscriber& subscriber, BehaviorBase& behavior_base, manual::SimpleAction &manual_action ,BT::BehaviorTreeFactory& factory)
  {
    factory.registerBuilder<chassis::ChassisSlowGyro>("ChassisSlowGyro",
    [&behavior_base , &cmd_tools](const std::string &name , const BT::NodeConfig &config)
    {return std::make_unique<chassis::ChassisSlowGyro>(name , config ,behavior_base,cmd_tools);});

    factory.registerBuilder<chassis::AbnormalStillStopAllMotion>("AbnormalStillStopAllMotion",
      [&behavior_base , &cmd_tools](const std::string &name ,const BT::NodeConfig &config)
      {return std::make_unique<chassis::AbnormalStillStopAllMotion>(name , config , behavior_base , cmd_tools);});

    factory.registerBuilder<chassis::PatrolAbnormalBackHomeGoal>("PatrolAbnormalBackHomeGoal",
      [&behavior_base , blackboard , &cmd_tools](const std::string &name ,const BT::NodeConfig &config)
      {return std::make_unique<chassis::PatrolAbnormalBackHomeGoal>(name , config , behavior_base , *blackboard , cmd_tools);});

    factory.registerBuilder<chassis::PatrolAttackEnemyPositiveArea>("PatrolAttackEnemyPositiveArea" ,
      [&behavior_base , blackboard ,&cmd_tools](const std::string &name , const BT::NodeConfig &config)
      {return std::make_unique<chassis::PatrolAttackEnemyPositiveArea>(name , config , behavior_base , *blackboard , cmd_tools);});

      // PatrolOwnOutputArea
    factory.registerBuilder<chassis::PatrolOwnOutpostArea>("PatrolOwnOutpostArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::PatrolOwnOutpostArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // PatrolEnemyOutpostArea
    factory.registerBuilder<chassis::PatrolEnemyOutpostArea>("PatrolEnemyOutpostArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::PatrolEnemyOutpostArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // PatrolSentryPatrolArea
    factory.registerBuilder<chassis::PatrolSentryPatrolArea>("PatrolSentryPatrolArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::PatrolSentryPatrolArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // GotoReturnBloodArea
    factory.registerBuilder<chassis::GotoReturnBloodArea>("GotoReturnBloodArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GotoReturnBloodArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // PatrolHoleUpArea
    factory.registerBuilder<chassis::PatrolHoleUpArea>("PatrolHoleUpArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::PatrolHoleUpArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // GotoEnemyBaseArea
    factory.registerBuilder<chassis::GotoEnemyBaseArea>("GotoEnemyBaseArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GotoEnemyBaseArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // GotoAttackEnemyEngineer
    factory.registerBuilder<chassis::GotoAttackEnemyEngineer>("GotoAttackEnemyEngineer",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GotoAttackEnemyEngineer>(name, config, behavior_base, *blackboard, cmd_tools); });

    // PatrolAfterRevivePatrolArea
    factory.registerBuilder<chassis::PatrolAfterRevivePatrolArea>("PatrolAfterRevivePatrolArea",
      [&behavior_base, blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::PatrolAfterRevivePatrolArea>(name, config, behavior_base, *blackboard, cmd_tools); });

    // GotoConductPoint
    factory.registerBuilder<chassis::GotoConductPoint>("GotoConductPoint",
      [&behavior_base, &cmd_tools, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GotoConductPoint>(name, config, behavior_base, cmd_tools, subscriber); });

    // CreateMbfClient
    factory.registerBuilder<chassis::CreateMbfClient>("CreateMbfClient",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::CreateMbfClient>(name, config, cmd_tools); });

    // ChaseEnemy
    factory.registerBuilder<chassis::ChaseEnemy>("ChaseEnemy",
      [blackboard, &subscriber, &cmd_tools, &bt_nh, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::ChaseEnemy>(name, config, *blackboard, subscriber, cmd_tools, bt_nh, behavior_base); });

    // SetChassisMode
    factory.registerBuilder<chassis::SetChassisMode>("SetChassisMode",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::SetChassisMode>(name, config, *blackboard); });

    // ReviveIfDead
    factory.registerBuilder<chassis::ReviveIfDead>("ReviveIfDead",
      [&cmd_tools, &subscriber, wait_time, blackboard, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::ReviveIfDead>(name, config, cmd_tools, subscriber, wait_time, *blackboard, behavior_base); });

    // SetIsEnableFight
    factory.registerBuilder<chassis::SetIsEnableFight>("SetIsEnableFight",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::SetIsEnableFight>(name, config, *blackboard); });

    // GetPresentTime
    factory.registerBuilder<chassis::GetPresentTime>("GetPresentTime",
      [blackboard, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GetPresentTime>(name, config, *blackboard, subscriber); });

    // GetKeyboardCommand
    factory.registerBuilder<chassis::GetKeyboardCommand>("GetKeyboardCommand",
      [blackboard, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::GetKeyboardCommand>(name, config, *blackboard, subscriber); });

    // SetGyroInCombat
    factory.registerBuilder<chassis::SetGyroInCombat>("SetGyroInCombat",
      [blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::SetGyroInCombat>(name, config, *blackboard, cmd_tools); });

    // ==================== Condition Nodes Registration ====================

    // IsNavigationReady
    factory.registerBuilder<chassis::IsNavigationReady>("IsNavigationReady",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsNavigationReady>(name, config, cmd_tools); });

    // IsRefereeOnline
    factory.registerBuilder<chassis::IsRefereeOnline>("IsRefereeOnline",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsRefereeOnline>(name, config, subscriber); });

    // IsGameInBattle
    factory.registerBuilder<chassis::IsGameInBattle>("IsGameInBattle",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsGameInBattle>(name, config, subscriber); });

    // IsClientMapUpdate
    factory.registerBuilder<chassis::IsClientMapUpdate>("IsClientMapUpdate",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsClientMapUpdate>(name, config, subscriber); });

    // IsSentryHpUrgent
    factory.registerBuilder<chassis::IsSentryHpUrgent>("IsSentryHpUrgent",
      [&subscriber, blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsSentryHpUrgent>(name, config, subscriber, *blackboard); });

    // IsSentryHpReturnMax
    factory.registerBuilder<chassis::IsSentryHpReturnMax>("IsSentryHpReturnMax",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsSentryHpReturnMax>(name, config, subscriber); });

    // IsNeedAvoidDrone
    factory.registerBuilder<chassis::IsNeedAvoidDrone>("IsNeedAvoidDrone",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsNeedAvoidDrone>(name, config, *blackboard); });

    // IsNeedDefenseBase
    factory.registerBuilder<chassis::IsNeedDefenseBase>("IsNeedDefenseBase",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsNeedDefenseBase>(name, config, *blackboard); });

    // IsTimeRangeCondition
    factory.registerBuilder<chassis::IsTimeRangeCondition>("IsTimeRangeCondition",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsTimeRangeCondition>(name, config, *blackboard); });

    // IsDefenseBuffBelowTheThreshold
    factory.registerBuilder<chassis::IsDefenseBuffBelowTheThreshold>("IsDefenseBuffBelowTheThreshold",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsDefenseBuffBelowTheThreshold>(name, config, *blackboard); });

    // IsChasePathFinished
    factory.registerBuilder<chassis::IsChasePathFinished>("IsChasePathFinished",
      [blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsChasePathFinished>(name, config, *blackboard, cmd_tools); });

    // IsOwnOutpostHpBeyondTheValue
    factory.registerBuilder<chassis::IsOwnOutpostHpBeyondTheValue>("IsOwnOutpostHpBeyondTheValue",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsOwnOutpostHpBeyondTheValue>(name, config, *blackboard); });

    // IsInOwnHalfArea
    factory.registerBuilder<chassis::IsInOwnHalfArea>("IsInOwnHalfArea",
      [blackboard, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsInOwnHalfArea>(name, config, *blackboard, cmd_tools); });

    // IsBulletsRemain
    factory.registerBuilder<chassis::IsBulletsRemain>("IsBulletsRemain",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsBulletsRemain>(name, config, subscriber); });

    // CheckTargetType
    factory.registerBuilder<chassis::CheckTargetType>("CheckTargetType",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::CheckTargetType>(name, config, *blackboard); });

    // CheckGimbalMode
    factory.registerBuilder<chassis::CheckGimbalMode>("CheckGimbalMode",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::CheckGimbalMode>(name, config, *blackboard); });

    // IsHasRevived
    factory.registerBuilder<chassis::IsHasRevived>("IsHasRevived",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsHasRevived>(name, config, *blackboard); });

    // IsEnableFight
    factory.registerBuilder<chassis::IsEnableFight>("IsEnableFight",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsEnableFight>(name, config, *blackboard); });

    // IsEnableHoleUp
    factory.registerBuilder<chassis::IsEnableHoleUp>("IsEnableHoleUp",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsEnableHoleUp>(name, config, *blackboard); });

    // IsHasEngineerMarked
    factory.registerBuilder<chassis::IsHasEngineerMarked>("IsHasEngineerMarked",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsHasEngineerMarked>(name, config, subscriber); });

    // IsEngineerAlive
    factory.registerBuilder<chassis::IsEngineerAlive>("IsEngineerAlive",
      [blackboard, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsEngineerAlive>(name, config, *blackboard, subscriber); });

    // IsOutpostAlive
    factory.registerBuilder<chassis::IsOutpostAlive>("IsOutpostAlive",
      [blackboard, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsOutpostAlive>(name, config, *blackboard, subscriber); });

    // IsReachGoal
    factory.registerBuilder<chassis::IsReachGoal>("IsReachGoal",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<chassis::IsReachGoal>(name, config, cmd_tools); });

    // ==================== Gimbal Action Nodes Registration ====================

    // SetGimbalMode
    factory.registerBuilder<gimbal::SetGimbalMode>("SetGimbalMode",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::SetGimbalMode>(name, config, *blackboard); });

    // YawSlowRound
    factory.registerBuilder<gimbal::YawSlowRound>("YawSlowRound",
      [blackboard, &cmd_tools, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::YawSlowRound>(name, config, *blackboard, cmd_tools, behavior_base); });

    // LidarTowardsFront
    factory.registerBuilder<gimbal::LidarTowardsFront>("LidarTowardsFront",
      [&cmd_tools, &subscriber, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::LidarTowardsFront>(name, config, cmd_tools, subscriber, behavior_base); });

    // RoundSearchEnemy
    factory.registerBuilder<gimbal::RoundSearchEnemy>("RoundSearchEnemy",
      [blackboard, &cmd_tools, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::RoundSearchEnemy>(name, config, *blackboard, cmd_tools, behavior_base); });

    // InverseGimbal
    factory.registerBuilder<gimbal::InverseGimbal>("InverseGimbal",
      [&behavior_base, &subscriber, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::InverseGimbal>(name, config, behavior_base, subscriber, cmd_tools); });

    // AimOutpost
    factory.registerBuilder<gimbal::AimOutpost>("AimOutpost",
      [blackboard, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::AimOutpost>(name, config, *blackboard, behavior_base); });

    // AimBase
    factory.registerBuilder<gimbal::AimBase>("AimBase",
      [blackboard, &behavior_base, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::AimBase>(name, config, *blackboard, behavior_base, subscriber); });

    // TrackEnemy
    factory.registerBuilder<gimbal::TrackEnemy>("TrackEnemy",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::TrackEnemy>(name, config, cmd_tools); });

    // ==================== Gimbal Condition Nodes Registration ====================

    // IsTrackLoss
    factory.registerBuilder<gimbal::IsTrackLoss>("IsTrackLoss",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::IsTrackLoss>(name, config, *blackboard); });

    // IsNeedInverseGimbal
    factory.registerBuilder<gimbal::IsNeedInverseGimbal>("IsNeedInverseGimbal",
      [blackboard, &subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<gimbal::IsNeedInverseGimbal>(name, config, *blackboard, subscriber); });

    // ==================== Shooter Action Nodes Registration ====================

    // SetShooterMode
    factory.registerBuilder<shooter::SetShooterMode>("SetShooterMode",
      [blackboard](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::SetShooterMode>(name, config, *blackboard); });

    // ShooterStop
    factory.registerBuilder<shooter::ShooterStop>("ShooterStop",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::ShooterStop>(name, config, cmd_tools); });

    // ShooterReady
    factory.registerBuilder<shooter::ShooterReady>("ShooterReady",
      [&behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::ShooterReady>(name, config, behavior_base); });

    // ShooterPush
    factory.registerBuilder<shooter::ShooterPush>("ShooterPush",
      [&cmd_tools](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::ShooterPush>(name, config, cmd_tools); });

    // ==================== Shooter Condition Nodes Registration ====================

    // IsTargetNotInvincible
    factory.registerBuilder<shooter::IsTargetNotInvincible>("IsTargetNotInvincible",
      [](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::IsTargetNotInvincible>(name, config); });

    // IsTargetEffective
    factory.registerBuilder<shooter::IsTargetEffective>("IsTargetEffective",
      [&subscriber](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<shooter::IsTargetEffective>(name, config, subscriber); });

    // Chassis Control Node
    factory.registerSimpleAction("ManualSendChassisCmd",
      [&manual_action](BT::TreeNode& leaf) {
        return manual_action.sendChassisCmd();
      });

    // Gimbal Control Node
    factory.registerSimpleAction("ManualSendGimbalCmd",
      [&manual_action](BT::TreeNode& leaf) {
        return manual_action.sendGimbalCmd();
      });

    // Shooter Control Node
    factory.registerSimpleAction("ManualSendShooterCmd",
      [&manual_action](BT::TreeNode& leaf) {
        return manual_action.sendShooterCmd();
      });

    factory.registerBuilder<manual::RemoteControlTurnOff>("RemoteControlTurnOff",
  [&behavior_base, &cmd_tools](const std::string &name, const BT::NodeConfig &config)
  { return std::make_unique<manual::RemoteControlTurnOff>(name, config, behavior_base, cmd_tools); });

    // ==================== Manual Condition Nodes Registration ====================

    // IsRemoteControlTurnOn
    factory.registerBuilder<manual::IsRemoteControlTurnOn>("IsRemoteControlTurnOn",
      [&subscriber, &behavior_base](const std::string &name, const BT::NodeConfig &config)
      { return std::make_unique<manual::IsRemoteControlTurnOn>(name, config, subscriber, behavior_base); });
  }
}


