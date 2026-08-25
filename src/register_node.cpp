//
// Created by root on 2026/3/29.
//

#include "register_node.h"

namespace register_node
{
  void register_node(ros::NodeHandle& bt_nh, tools::CmdTools& cmd_tools, perception::Subscriber& subscriber,
                     BT::BehaviorTreeFactory& factory,
                     tools::NavigationTools& navigation_tools, tools::MiniMapTools& mini_map_tools,
                     tools::ControllerTools& controller_tools, tools::GimbalTools& gimbal_tools, tools::PlannerTools &planner_tools,
                     perception::TfAccessor& tf_accessor , perception::Publisher &publisher)
  {
    factory.registerBuilder<chassis::ChassisSlowGyro>(
      "ChassisSlowGyro",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::ChassisSlowGyro>(name, config, cmd_tools);
      });

    factory.registerBuilder<chassis::AbnormalStillStopAllMotion>(
      "AbnormalStillStopAllMotion",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::AbnormalStillStopAllMotion>(name, config, cmd_tools);
      });

    factory.registerBuilder<chassis::SetGyroInCombat>(
      "SetGyroInCombat",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::SetGyroInCombat>(name, config, cmd_tools);
      });

    // ==================== 2. 依赖 blackboard 和 navigation_tools 的异步/状态节点 ====================
    // 注：由于构造函数需要传入 BT::Blackboard& 引用，我们通过 *config.blackboard 动态获取当前树实例的黑板。

    factory.registerBuilder<chassis::PatrolAbnormalBackHomeGoal>(
      "PatrolAbnormalBackHomeGoal",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<
          chassis::PatrolAbnormalBackHomeGoal>(name, config, *config.blackboard, navigation_tools);
      });

    factory.registerBuilder<chassis::PatrolAttackEnemyPositiveArea>(
      "PatrolAttackEnemyPositiveArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolAttackEnemyPositiveArea>(name, config, *config.blackboard,
                                                                        navigation_tools);
      });

    factory.registerBuilder<chassis::PatrolOwnOutpostArea>(
      "PatrolOwnOutpostArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolOwnOutpostArea>(name, config, *config.blackboard, navigation_tools);
      });

    factory.registerBuilder<chassis::PatrolEnemyOutpostArea>(
      "PatrolEnemyOutpostArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolEnemyOutpostArea>(name, config, navigation_tools);
      });

    factory.registerBuilder<chassis::PatrolSentryPatrolArea>(
      "PatrolSentryPatrolArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolSentryPatrolArea>(name, config, navigation_tools);
      });

    factory.registerBuilder<chassis::GotoReturnBloodArea>(
      "GotoReturnBloodArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoReturnBloodArea>(name, config, *config.blackboard, navigation_tools);
      });

    factory.registerBuilder<chassis::PatrolHoleUpArea>(
      "PatrolHoleUpArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolHoleUpArea>(name, config, *config.blackboard, navigation_tools);
      });

    factory.registerBuilder<chassis::GotoEnemyBaseArea>(
      "GotoEnemyBaseArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoEnemyBaseArea>(name, config, *config.blackboard, navigation_tools);
      });

    factory.registerBuilder<chassis::GotoAttackEnemyEngineer>(
      "GotoAttackEnemyEngineer",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoAttackEnemyEngineer>(name, config, *config.blackboard, navigation_tools);
      });

    // ==================== 3. 其他混合依赖的导航与感知节点 ====================

    factory.registerBuilder<chassis::GotoConductPoint>(
      "GotoConductPoint",
      [&subscriber, &navigation_tools, &mini_map_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoConductPoint>(name, config, subscriber, navigation_tools, mini_map_tools);
      });

    factory.registerBuilder<chassis::ChaseEnemy>(
      "ChaseEnemy",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::ChaseEnemy>(name, config, navigation_tools);
      });

    factory.registerBuilder<chassis::GetKeyboardCommand>(
      "GetKeyboardCommand",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GetKeyboardCommand>(name, config, subscriber);
      });

    // ==================== 4. 仅依赖黑板的控制节点 ====================

    factory.registerBuilder<chassis::SetChassisMode>(
      "SetChassisMode",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::SetChassisMode>(name, config);
      });

    factory.registerBuilder<chassis::SetIsEnableFight>(
      "SetIsEnableFight",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::SetIsEnableFight>(name, config, *config.blackboard);
      });

    // ==================== 5. 高度复杂依赖节点 (ReviveIfDead) ====================

    factory.registerBuilder<ReviveIfDead>(
      "ReviveIfDead",
      [&cmd_tools, &subscriber, &navigation_tools, &controller_tools](
      const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<ReviveIfDead>(
          name, config, cmd_tools, subscriber, navigation_tools, controller_tools);
      });

    // ==================== 1. 仅依赖黑板的云台控制节点 ====================

    factory.registerBuilder<gimbal::SetGimbalMode>(
      "SetGimbalMode",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::SetGimbalMode>(name, config);
      });

    // ==================== 2. 依赖 gimbal_tools 的扫描/控制节点 ====================

    factory.registerBuilder<gimbal::YawSlowRound>(
      "YawSlowRound",
      [&gimbal_tools , &cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::YawSlowRound>(name, config, gimbal_tools,cmd_tools);
      });

    // ==================== 3. 依赖感知与坐标变换的对敌节点 ====================

    factory.registerBuilder<gimbal::InverseGimbal>(
      "InverseGimbal",
      [&gimbal_tools,&cmd_tools ,&subscriber, &tf_accessor](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::InverseGimbal>(name, config, gimbal_tools,cmd_tools, subscriber, tf_accessor);
      });

    factory.registerBuilder<gimbal::TrackEnemy>(
      "TrackEnemy",
      [&cmd_tools, &gimbal_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::TrackEnemy>(name, config, cmd_tools, gimbal_tools);
      });

    // ==================== 1. 仅依赖树配置的同步节点 ====================

    factory.registerBuilder<shooter::SetShooterMode>(
      "SetShooterMode",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<shooter::SetShooterMode>(name, config);
      });

    // ==================== 2. 依赖 cmd_tools 的异步状态节点 ====================

    factory.registerBuilder<shooter::ShooterStop>(
      "ShooterStop",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<shooter::ShooterStop>(name, config, cmd_tools);
      });

    factory.registerBuilder<shooter::ShooterReady>(
      "ShooterReady",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<shooter::ShooterReady>(name, config, cmd_tools);
      });

    factory.registerBuilder<shooter::ShooterPush>(
      "ShooterPush",
      [&cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<shooter::ShooterPush>(name, config, cmd_tools);
      });

    // ==================== 1. 注册 状态节点 (StatefulActionNode) ====================

    factory.registerBuilder<RemoteControlTurnOff>(
      "RemoteControlTurnOff",
      [&cmd_tools, &controller_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<RemoteControlTurnOff>(name, config, cmd_tools,controller_tools);
      });
    // ==================== 仅依赖 controller_tools 的核心控制器节点 ====================

    factory.registerBuilder<StartMainControllers>(
      "StartMainControllers",
      [&controller_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<StartMainControllers>(name, config, controller_tools);
      });

    factory.registerBuilder<StopMainControllers>(
      "StopMainControllers",
      [&controller_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<StopMainControllers>(name, config, controller_tools);
      });

    // ==================== 1. 仅依赖 subscriber 的条件节点 ====================

    factory.registerBuilder<condition_node::IsRefereeOnline>(
      "IsRefereeOnline",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsRefereeOnline>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsGameInBattle>(
      "IsGameInBattle",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsGameInBattle>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsClientMapUpdate>(
      "IsClientMapUpdate",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsClientMapUpdate>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsSentryHpUrgent>(
      "IsSentryHpUrgent",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsSentryHpUrgent>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsTimeRangeCondition>(
      "IsTimeRangeCondition",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsTimeRangeCondition>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsOwnOutpostHpBeyondTheValue>(
      "IsOwnOutpostHpBeyondTheValue",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsOwnOutpostHpBeyondTheValue>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsBulletsRemain>(
      "IsBulletsRemain",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsBulletsRemain>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::CheckTargetType>(
      "CheckTargetType",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::CheckTargetType>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsHasEngineerMarked>(
      "IsHasEngineerMarked",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsHasEngineerMarked>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsEngineerAlive>(
      "IsEngineerAlive",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsEngineerAlive>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsOutpostAlive>(
      "IsOutpostAlive",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsOutpostAlive>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsTargetEffective>(
      "IsTargetEffective",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsTargetEffective>(name, config, subscriber);
      });

    factory.registerBuilder<condition_node::IsRemoteControlTurnOn>(
      "IsRemoteControlTurnOn",
      [&subscriber , &controller_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsRemoteControlTurnOn>(name, config, subscriber, controller_tools);
      });

    // ==================== 2. 依赖黑板的条件节点 ====================

    factory.registerBuilder<condition_node::CheckGimbalMode>(
      "CheckGimbalMode",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::CheckGimbalMode>(name, config, *config.blackboard);
      });

    factory.registerBuilder<condition_node::IsNeedInverseGimbal>(
      "IsNeedInverseGimbal",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsNeedInverseGimbal>(name, config, *config.blackboard, subscriber);
      });

    // ==================== 3. 无外部依赖（或暂未完工）的条件节点 ====================

    factory.registerBuilder<condition_node::IsNeedAvoidDrone>(
      "IsNeedAvoidDrone",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsNeedAvoidDrone>(name, config);
      });

    factory.registerBuilder<condition_node::IsNeedDefenseBase>(
      "IsNeedDefenseBase",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsNeedDefenseBase>(name, config);
      });

    factory.registerBuilder<condition_node::IsDefenseBuffBelowTheThreshold>(
      "IsDefenseBuffBelowTheThreshold",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsDefenseBuffBelowTheThreshold>(name, config);
      });

    factory.registerBuilder<condition_node::IsInOwnHalfArea>(
      "IsInOwnHalfArea",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsInOwnHalfArea>(name, config);
      });

    factory.registerBuilder<condition_node::IsTrackLoss>(
      "IsTrackLoss",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsTrackLoss>(name, config);
      });

    factory.registerBuilder<condition_node::IsTargetNotInvincible>(
      "IsTargetNotInvincible",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsTargetNotInvincible>(name, config);
      });

    factory.registerBuilder<VisionCalibrate>(
      "VisionCalibrate",
      [&subscriber , &bt_nh](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<VisionCalibrate>(name, config,bt_nh , subscriber);
      });

    factory.registerBuilder<OutputRightSwitchState>(
      "OutputRightSwitchState",
      [&subscriber , &navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<OutputRightSwitchState>(name, config,subscriber,navigation_tools);
      });

    factory.registerBuilder<SetIdle>(
      "SetIdle",
      [&controller_tools , &cmd_tools , &publisher , &planner_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<SetIdle>(name, config,controller_tools,cmd_tools,planner_tools,publisher);
      });

    factory.registerBuilder<Test1>(
      "Test1",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<Test1>(name, config);
      });

    factory.registerBuilder<Test2>(
      "Test2",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<Test2>(name, config);
      });

    factory.registerBuilder<chassis::PatrolTestArea>(
      "PatrolTestArea",
      [&navigation_tools , &planner_tools , &cmd_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::PatrolTestArea>(name, config,navigation_tools , planner_tools,cmd_tools);
      });

    factory.registerBuilder<condition_node::IsPoseValid>(
      "IsPoseValid",
      [&tf_accessor](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsPoseValid>(name, config,tf_accessor);
      });

    factory.registerBuilder<Relocate>(
      "Relocate",
      [&bt_nh](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<Relocate>(name, config,bt_nh);
      });

    factory.registerBuilder<RelieveWeakState>(
      "RelieveWeakState",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<RelieveWeakState>(name, config,subscriber);
      });

    factory.registerBuilder<condition_node::IsHeroInTrapezoid>(
      "IsHeroInTrapezoid",
      [&subscriber,&mini_map_tools,&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsHeroInTrapezoid>(name, config,subscriber,mini_map_tools,navigation_tools);
      });

    factory.registerBuilder<condition_node::IsOwnFortressBeenCap>(
      "IsOwnFortressBeenCap",
      [&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<condition_node::IsOwnFortressBeenCap>(name, config,subscriber);
      });

    factory.registerBuilder<manual::ManualSendChassisCmd>(
      "ManualSendChassisCmd",
      [&cmd_tools,&subscriber,&bt_nh](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<manual::ManualSendChassisCmd>(name, config,cmd_tools,subscriber,bt_nh);
      });

    factory.registerBuilder<manual::ManualSendGimbalCmd>(
      "ManualSendGimbalCmd",
      [&cmd_tools,&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<manual::ManualSendGimbalCmd>(name, config,cmd_tools,subscriber);
      });

    factory.registerBuilder<manual::ManualSendShooterCmd>(
      "ManualSendShooterCmd",
      [&cmd_tools,&subscriber](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<manual::ManualSendShooterCmd>(name, config,cmd_tools,subscriber);
      });

    factory.registerBuilder<chassis::GotoTrapezoidArea>(
      "GotoTrapezoidArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoTrapezoidArea>(name, config,navigation_tools);
      });

    factory.registerBuilder<chassis::GotoBaseDefenceArea>(
      "GotoBaseDefenceArea",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoBaseDefenceArea>(name, config,navigation_tools);
      });

    factory.registerBuilder<chassis::GotoOwnFortress>(
      "GotoOwnFortress",
      [&navigation_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<chassis::GotoOwnFortress>(name, config,navigation_tools);
      });

    factory.registerBuilder<ReactiveIfThenElse>(
      "ReactiveIfThenElse",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<ReactiveIfThenElse>(name, config);
      });

    factory.registerBuilder<RunForSeconds>(
      "RunForSeconds",
      [](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<RunForSeconds>(name, config);
      });

    factory.registerBuilder<gimbal::UpdateAimPriority>(
      "UpdateAimPriority",
      [&navigation_tools,&tf_accessor,&subscriber,&publisher](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::UpdateAimPriority>(name, config,tf_accessor,subscriber,publisher,navigation_tools);
      });

    factory.registerBuilder<gimbal::PreAimingOutpost>(
      "PreAimingOutpost",
      [&gimbal_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::PreAimingOutpost>(name, config,gimbal_tools);
      });

    factory.registerBuilder<gimbal::PreAimingBase>(
      "PreAimingBase",
      [&gimbal_tools](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<gimbal::PreAimingBase>(name, config,gimbal_tools);
      });
  }
}


