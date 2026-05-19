//
// Created by root on 2026/3/19.
//

#ifndef NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H

#include <behaviortree_cpp/condition_node.h>
#include "common/types.h"

#include "behaviortree_cpp/action_node.h"
#include "common/tools.h"

namespace condition_node
{
  class IsRefereeOnline : public BT::ConditionNode
  {
  public:
    IsRefereeOnline(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool is_online = subscriber_.isRefereeOnline();
      BT::NodeStatus status = is_online == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsGameInBattle : public BT::ConditionNode
  {
  public:
    IsGameInBattle(const std::string& name, const BT::NodeConfig& config,
                   perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool is_in_battle = subscriber_.getGameStatus().game_progress == rm_msgs::GameStatus::IN_BATTLE;
      BT::NodeStatus status = is_in_battle == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsClientMapUpdate : public BT::ConditionNode
  {
  public:
    IsClientMapUpdate(const std::string& name, const BT::NodeConfig& config,
                      perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool is_update = subscriber_.isClientMapUpdate();
      BT::NodeStatus status = is_update ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsSentryHpUrgent : public BT::ConditionNode
  {
  public:
    IsSentryHpUrgent(const std::string& name, const BT::NodeConfig& config,
                     perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("trigger_blood_return_hp")};
    }

    BT::NodeStatus tick() override
    {
      int trigger_hp;
      if (!getInput("trigger_blood_return_hp", trigger_hp))
      {
        ROS_ERROR("BT can not access key name [trigger_blood_return_hp] , default value is 30");
        trigger_hp = 30;
      }

      bool is_urgent = subscriber_.getGameRobotStatus().remain_hp < trigger_hp;
      BT::NodeStatus status = is_urgent == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsSentryHpReturnMax : public BT::ConditionNode
  {
  public:
    IsSentryHpReturnMax(const std::string& name, const BT::NodeConfig& config,
                        perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      BT::NodeStatus status = subscriber_.getGameRobotStatus().remain_hp >= subscriber_.getGameRobotStatus().max_hp
                                ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsNeedAvoidDrone : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsNeedAvoidDrone(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsNeedDefenseBase : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsNeedDefenseBase(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsTimeRangeCondition : public BT::ConditionNode
  {
  public:
    IsTimeRangeCondition(const std::string& name, const BT::NodeConfig& config,
                         perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("min_time"),
        BT::InputPort<double>("max_time"),
        BT::InputPort<double>("game_total_time")
      };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<double> min_time = getInput<double>("min_time");
      BT::Expected<double> max_time = getInput<double>("max_time");
      double game_total_time;
      if (!getInput("game_total_time", game_total_time))
      {
        ROS_ERROR("BT can not access key name [game_total_time] , default value is 420.0");
        game_total_time = 420.0;
      }
      double present_time = game_total_time - subscriber_.getGameStatus().stage_remain_time;

      if (present_time >= min_time.value() && present_time <= max_time.value())
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsDefenseBuffBelowTheThreshold : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsDefenseBuffBelowTheThreshold(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("defense_buff_threshold")};
    }

    BT::NodeStatus tick() override
    {
    }

  private:
  };

  class IsOwnOutpostHpBeyondTheValue : public BT::ConditionNode
  {
  public:
    IsOwnOutpostHpBeyondTheValue(const std::string& name, const BT::NodeConfig& config,
                                 perception::Subscriber& subscriber) : ConditionNode(name, config),
                                                                       subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("outpost_hp_threshold")};
    }

    BT::NodeStatus tick() override
    {
      int threshold;
      if (!getInput("outpost_hp_threshold", threshold))
      {
        ROS_ERROR("BT can not access key name [outpost_hp_threshold] , default value is 800");
        threshold = 800;
      }
      BT::NodeStatus status = subscriber_.getGameRobotHp().ally_outpost_hp > threshold
                                ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsInOwnHalfArea : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsInOwnHalfArea(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsBulletsRemain : public BT::ConditionNode
  {
  public:
    IsBulletsRemain(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.getBulletAllowance().bullet_allowance_num_17_mm > 0 &&
        subscriber_.getBulletAllowance().bullet_allowance_num_17_mm < 2000 == true)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class CheckTargetType : public BT::ConditionNode //若探测到的目标与给定的目标相同，则返回success，否则返回failure
  {
  public:
    CheckTargetType(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("track_id")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> track_id = getInput<int>("track_id");

      if (track_id.value() == subscriber_.getTrackData().id)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsHasEngineerMarked : public BT::ConditionNode
  {
  public:
    IsHasEngineerMarked(const std::string& name, const BT::NodeConfig& config,
                        perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool has_engineer_marked = subscriber_.hasEngineerMarked();
      return has_engineer_marked == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsEngineerAlive : public BT::ConditionNode
  {
  public:
    IsEngineerAlive(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.getGameRobotHp().ally_2_robot_hp > 0)
        return BT::NodeStatus::SUCCESS;
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsOutpostAlive : public BT::ConditionNode
  {
  public:
    IsOutpostAlive(const std::string& name, const BT::NodeConfig& config,
                   perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.getGameRobotHp().ally_outpost_hp > 0)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsTrackLoss : public BT::ConditionNode
  {
  public:
    IsTrackLoss(const std::string& name, const BT::NodeConfig& config,
                BT::Blackboard& blackboard) : ConditionNode(name, config), blackboard_(blackboard)
    {
    }

    BT::NodeStatus tick() override
    {
      ros::Time last_track_time;
      double lost_track_tolerant_sec;
      try
      {
        last_track_time = blackboard_.get<ros::Time>("last_track_time");
      }
      catch (BT::RuntimeError& e)
      {
        ROS_ERROR("BT can not access key name [last_track_time] , default value is ros::Time(0)");
        last_track_time = ros::Time(0);
      }

      lost_track_tolerant_sec = blackboard_.get<double>("lost_track_tolerant_sec");

      if (ros::Time::now() - last_track_time > ros::Duration(lost_track_tolerant_sec))
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard& blackboard_;
  };

  //TODO : gimbal_mode需保证在gimbal_action_node中正确设定
  class CheckGimbalMode : public BT::ConditionNode // 若实际的云台模式和给定的云台模式相同，则返回success，否则返回failure
  {
  public:
    CheckGimbalMode(const std::string& name, const BT::NodeConfig& config,
                    BT::Blackboard& blackboard) : ConditionNode(name, config), blackboard_(blackboard)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("gimbal_mode_id")};
    }

    BT::NodeStatus tick() override
    {
      try
      {
        gimbal_mode_ = static_cast<int>(blackboard_.get<types::GimbalMode>("gimbal_mode"));
      }
      catch (BT::RuntimeError& e)
      {
        ROS_WARN("BT can not access key name [gimbal_mode] , default is types::GimbalMode::YawSlowRound");
        gimbal_mode_ = static_cast<int>(types::GimbalMode::YawSlowRound);
      }

      BT::Expected<int> gimbal_mode_id = getInput<int>("gimbal_mode_id");
      if (gimbal_mode_id.value() == gimbal_mode_)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard& blackboard_;
    int gimbal_mode_;
  };

  class IsNeedInverseGimbal : public BT::ConditionNode
  {
  public:
    IsNeedInverseGimbal(const std::string& name, const BT::NodeConfig& config, BT::Blackboard& blackboard,
                        perception::Subscriber& subscriber) : BT::ConditionNode(name, config), blackboard_(blackboard),
                                                              subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      std::vector<int> default_aim_rank;
      default_aim_rank = blackboard_.get<std::vector<int>>("default_aim_rank");
      if (subscriber_.has_back_camera_detected_ &&
        subscriber_.back_camera_detection_id_ != 0)
      {
        if (default_aim_rank[subscriber_.back_camera_detection_id_] == 0) //id是攻击优先级所在的数组下标，数组内部的值为攻击优先级
        {
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          subscriber_.has_back_camera_detected_ = false;
          subscriber_.back_camera_detection_id_ = 0;
          return BT::NodeStatus::SUCCESS; //如果不是等于0的优先级（无效），就把云台反过来
        }
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard& blackboard_;
    perception::Subscriber& subscriber_;
  };

  class IsTargetNotInvincible : public BT::ConditionNode //TODO : 需重写无敌检测算法
  {
  public:
    IsTargetNotInvincible(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsTargetEffective : public BT::ConditionNode
  {
  public:
    IsTargetEffective(const std::string& name, const BT::NodeConfig& config,
                      perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      return subscriber_.track_data_.id != 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsRemoteControlTurnOn : public BT::ConditionNode
  {
  public:
    IsRemoteControlTurnOn(const std::string &name , const BT::NodeConfig &config , perception::Subscriber &subscriber , BehaviorBase &behavior_base) : ConditionNode(name , config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (ros::Time::now() - subscriber_.dbus_.stamp < ros::Duration(0.3))
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber &subscriber_;
  };
}

#endif //NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H
