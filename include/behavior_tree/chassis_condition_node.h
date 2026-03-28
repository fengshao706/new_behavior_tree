//
// Created by root on 2026/3/19.
//

#ifndef NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H

#include <behaviortree_cpp/condition_node.h>
#include "common/types.h"

#include "behaviortree_cpp/action_node.h"
#include "common/behavior_base.h"
#include "common/tools.h"
namespace chassis
{
  class IsNavigationReady : public BT::ConditionNode
  {
  public:
    IsNavigationReady(std::string &name , BT::NodeConfig & config , tools::CmdTools &cmd_tools) : ConditionNode(name,config) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus tick() override
    {
      bool is_server_connect = cmd_tools_.mbf_client_->waitForServer(ros::Duration(0.01));
      if (!is_server_connect)
      {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }

  private:
    tools::CmdTools &cmd_tools_;
  };

  class IsRefereeOnline : public BT::ConditionNode
  {
  public:
    IsRefereeOnline(std::string &name , BT::NodeConfig &config , perception::Subscriber & subscriber) : ConditionNode(name,config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.referee_is_online_ == true)
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

  class IsGameInBattle : public BT::ConditionNode
  {
  public:
    IsGameInBattle(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name,config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.game_status_.game_progress == rm_msgs::GameStatus::IN_BATTLE)
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

  class IsClientMapUpdate : public BT::ConditionNode
  {
  public:
    IsClientMapUpdate(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name,config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.client_map_update_ == true)
      {
        subscriber_.client_map_update_ = false; //重新置为false位
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber &subscriber_;
  };

  class IsSentryHpUrgent : public BT::ConditionNode
  {
  public:
    IsSentryHpUrgent(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber , BT::Blackboard &blackboard) : ConditionNode(name,config) , subscriber_(subscriber) , blackboard_(blackboard)
    {
      try
      {
        trigger_blood_return_hp_without_buff = blackboard_.get<int>("trigger_blood_return_hp_without_buff");
      }catch (BT::RuntimeError & e)
      {
        trigger_blood_return_hp_without_buff = 330;
        ROS_ERROR("BT can not access key name [trigger_blood_return_hp_without_buff] , default value is 330");
      }
      try
      {
        trigger_blood_return_hp = blackboard_.get<int>("trigger_blood_return_hp");
      }catch (BT::RuntimeError & e)
      {
        trigger_blood_return_hp = 330;
        ROS_ERROR("BT can not access key name [trigger_blood_return_hp] , default value is 300");
      }
    }

    BT::NodeStatus tick() override
    {
      if ( subscriber_.game_robot_status_.remain_hp <
           (subscriber_.buff_.defence_buff < 30 ?
                trigger_blood_return_hp_without_buff :
                trigger_blood_return_hp) == true)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    int trigger_blood_return_hp_without_buff;
    int trigger_blood_return_hp;
    perception::Subscriber &subscriber_;
    BT::Blackboard &blackboard_;
  };

  class IsSentryHpReturnMax : public BT::ConditionNode
  {
  public:
    IsSentryHpReturnMax(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name,config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.game_robot_status_.remain_hp >=
           subscriber_.game_robot_status_.max_hp)
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

  class IsNeedAvoidDrone : public BT::ConditionNode
  {
  public:
    IsNeedAvoidDrone(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) , blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time need_avoid_drone_time;
      double avoid_drone_time;
      try
      {
        need_avoid_drone_time = blackboard_.get<ros::Time>("need_avoid_drone_time");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [need_avoid_drone_time] , default value is ros::Time(0)");
        need_avoid_drone_time = ros::Time(0);
      }

      try
      {
        avoid_drone_time = blackboard_.get<double>("avoid_drone_time");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [avoid_drone_time] , default value is 20.0");
        avoid_drone_time = 20.0;
      }

      if (ros::Time::now() - need_avoid_drone_time < ros::Duration(avoid_drone_time) == true)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard &blackboard_;
  };

  class IsNeedDefenseBase : public BT::ConditionNode
  {
  public:
    IsNeedDefenseBase(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) , blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      bool need_defense_base;
      try
      {
        need_defense_base = blackboard_.get<bool>("need_defense_base");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [need_defense_base] , default value is false");
        need_defense_base = false;
      }

      if (need_defense_base == true)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard &blackboard_;
  };

  class IsTimeRangeCondition : public BT::ConditionNode
  {
  public:
    IsTimeRangeCondition(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name , config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<double>("min_time") ,
                  BT::InputPort<double>("max_time")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<double> min_time = getInput<double>("min_time");
      BT::Expected<double> max_time = getInput<double>("max_time");
      double present_time = blackboard_.get<double>("present_time");

      if (present_time >= min_time.value() && present_time <= max_time.value())
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    BT::Blackboard & blackboard_;
  };

  class IsDefenseBuffBelowTheThreshold : public BT::ConditionNode
  {
  public:
    IsDefenseBuffBelowTheThreshold(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("defense_buff_threshold")};
    }

    BT::NodeStatus tick() override
    {
      int defense_buff;
      BT::Expected<int> defense_buff_threshold = getInput<int>("defense_buff_threshold");
      try
      {
        defense_buff = blackboard_.get<int>("defense_buff");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [defense_buff] , default value is 0");
        defense_buff = 0;
      }
      if (defense_buff < defense_buff_threshold.value())
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard &blackboard_;
  };

  class IsChasePathFinished : public BT::ConditionNode
  {
  public:
    IsChasePathFinished(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , tools::CmdTools &cmd_tools) : ConditionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time last_track_time;
      double chasing_max_for_time;
      try
      {
        last_track_time = blackboard_.get<ros::Time>("last_track_time"); // TODO : 该变量还未设置，需在getEnemyInfo中探测到对方装甲板的id不为0的时刻中设置
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [last_track_time] , default value is ros::Time(0)");
        last_track_time = ros::Time(0);
      }

      try
      {
        chasing_max_for_time = blackboard_.get<double>("chasing_max_for_time");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [chasing_max_for_time] , default value is 5");
        chasing_max_for_time = 5;
      }
      if (ros::Time::now() - last_track_time > ros::Duration(chasing_max_for_time) || cmd_tools_.mbf_client_->getState().isDone() == true)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }

    }
  private:
    BT::Blackboard &blackboard_;
    tools::CmdTools &cmd_tools_;
  };

  class IsOwnOutpostHpBeyondTheValue : public BT::ConditionNode
  {
  public:
    IsOwnOutpostHpBeyondTheValue(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("outpost_hp_threshold") };
    }

    BT::NodeStatus tick() override
    {
      rm_msgs::GameRobotHp game_robot_hp = blackboard_.get<rm_msgs::GameRobotHp>("game_robot_hp");
      std::string robot_color = blackboard_.get<std::string>("robot_color");
      BT::Expected<int> outpost_hp_threshold = getInput<int>("outpost_hp_threshold");
      if (robot_color == "red" && game_robot_hp.red_outpost_hp > outpost_hp_threshold.value())
      {
        return BT::NodeStatus::SUCCESS;
      }
      if (robot_color == "blue" && game_robot_hp.blue_outpost_hp > outpost_hp_threshold.value())
      {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class IsInOwnHalfArea : public BT::ConditionNode
  {
  public:
    IsInOwnHalfArea(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard ,tools::CmdTools &cmd_tools) : ConditionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , tf_buffer_(cmd_tools_.getTfBuffer())
    {

    }

    BT::NodeStatus tick() override
    {
      std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>> pos_detection_polygons;
      pos_detection_polygons = blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>>>("pos_detection_polygons");
      std::string robot_color = blackboard_.get<std::string>("robot_color");
      std::vector<std::string> own_half_area = blackboard_.get<std::vector<std::string>>(robot_color+"_half_area");

      try
      {
        cur_map2base_ = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        return BT::NodeStatus::FAILURE;
      }

      for (const auto& pair : pos_detection_polygons) //获取当前所在位置的名称
      {
        polygon_in_which_ = "unknown";
        if (tools::isPointInPolygon(cur_map2base_, pair.second) == true){
          polygon_in_which_ =  pair.first;
          break;
        }
      }

      if (polygon_in_which_ == "unknown")
      {
        return BT::NodeStatus::FAILURE;
      }

      for (int i = 0; i < own_half_area.size(); i++)//遍历自家半场的所有区域，如果现在待着的地方能够匹配上任意一个区域，那么就认为自己在自家半场
      {
        if (polygon_in_which_ == own_half_area[i])
          return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  private:
    BT::Blackboard &blackboard_;
    tools::CmdTools &cmd_tools_;
    tf2_ros::Buffer &tf_buffer_;
    geometry_msgs::TransformStamped cur_map2base_;
    std::string polygon_in_which_;
  };

  class IsBulletsRemain : public BT::ConditionNode
  {
  public:
    IsBulletsRemain(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name , config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.bullet_allowance_.bullet_allowance_num_17_mm > 0 &&
           subscriber_.bullet_allowance_.bullet_allowance_num_17_mm < 2000 == true)
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

  class CheckTargetType : public BT::ConditionNode  //若探测到的目标与给定的目标相同，则返回success，否则返回failure
  {
  public:
    CheckTargetType(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("track_id") };
    }

    BT::NodeStatus tick() override
    {
      try
      {
        track_data_=blackboard_.get<rm_msgs::TrackData>("track_data");
      }catch (BT::RuntimeError &e)
      {
        ROS_WARN("BT can not access key name [track_data] , return BT::NodeStatus::FAILURE");
        return BT::NodeStatus::FAILURE;
      }

      BT::Expected<int> track_id = getInput<int>("track_id");

      if (track_id.value() == track_data_.id)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    BT::Blackboard &blackboard_;
    rm_msgs::TrackData track_data_;
  };

  //TODO : gimbal_mode需保证在gimbal_action_node中正确设定
  class CheckGimbalMode : public BT::ConditionNode // 若实际的云台模式和给定的云台模式相同，则返回success，否则返回failure
  {
  public:
    CheckGimbalMode(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name,config) ,blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("gimbal_mode_id") };
    }

    BT::NodeStatus tick() override
    {
      try
      {
        gimbal_mode_ = static_cast<int>(blackboard_.get<types::GimbalMode>("gimbal_mode"));
      }catch (BT::RuntimeError &e)
      {
        ROS_WARN("BT can not access key name [gimbal_mode] , default is types::GimbalMode::YawSlowRound");
        gimbal_mode_ = static_cast<int>(types::GimbalMode::YawSlowRound);
      }

      BT::Expected<int> gimbal_mode_id = getInput<int>("gimbal_mode_id");
      if (gimbal_mode_id.value() == gimbal_mode_)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    BT::Blackboard &blackboard_;
    int gimbal_mode_;
  };

  class IsHasRevived : public BT::ConditionNode
  {
  public:
    IsHasRevived(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name , config) , blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      bool has_revived;
      try
      {
        has_revived = blackboard_.get<bool>("has_revived");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [has_revived] , default value is false");
        has_revived = false;
      }

      if (has_revived == true)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class IsEnableFight : public BT::ConditionNode
  {
  public:
    IsEnableFight(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name , config) , blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      bool is_enable_fight = blackboard_.get<bool>("enable_fight");
      return is_enable_fight == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class IsEnableHoleUp : public BT::ConditionNode
  {
  public:
    IsEnableHoleUp(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name , config) , blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      bool enable_hole_up = blackboard_.get<bool>("enable_hole_up");
      return enable_hole_up == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class IsHasEngineerMarked : public BT::ConditionNode
  {
  public:
    IsHasEngineerMarked(std::string &name , BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name , config) ,subscriber_(subscriber)
    {

    }
    BT::NodeStatus tick() override
    {
      bool has_engineer_marked = subscriber_.has_engineer_marked_;
      return has_engineer_marked == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  private:
    perception::Subscriber &subscriber_;
  };

  class IsEngineerAlive : public BT::ConditionNode
  {
  public:
    IsEngineerAlive(std::string &name , BT::NodeConfig &config, BT::Blackboard &blackboard ,perception::Subscriber &subscriber) : ConditionNode(name , config) ,blackboard_(blackboard) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      std::string robot_color = blackboard_.get<std::string>("robot_color");

      if (robot_color == "red")
      {
        if (subscriber_.game_robot_hp_.blue_2_robot_hp > 0)
          return BT::NodeStatus::SUCCESS;
        else
        {
          subscriber_.has_engineer_marked_ = false;
          return BT::NodeStatus::FAILURE;
        }
      }else
      {
        if (subscriber_.game_robot_hp_.red_2_robot_hp > 0)
          return BT::NodeStatus::SUCCESS;
        else
        {
          subscriber_.has_engineer_marked_ = false;
          return BT::NodeStatus::FAILURE;
        }
      }
    }
  private:
    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;
  };

  class IsOutpostAlive : public BT::ConditionNode
  {
  public:
    IsOutpostAlive(std::string &name ,BT::NodeConfig &config ,BT::Blackboard &blackboard, perception::Subscriber &subscriber) : ConditionNode(name,config) ,blackboard_(blackboard), subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      std::string robot_color = blackboard_.get<std::string>("robot_color");
      if (robot_color == "red")
      {
        if (subscriber_.game_robot_hp_.blue_outpost_hp > 0)
        {
          return BT::NodeStatus::SUCCESS;
        }else
        {
          return BT::NodeStatus::FAILURE;
        }
      }else
      {
        if (subscriber_.game_robot_hp_.red_outpost_hp > 0)
        {
          return BT::NodeStatus::SUCCESS;
        }else
        {
          return BT::NodeStatus::FAILURE;
        }
      }
    }

  private:
    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;

  };

  class IsReachGoal : public BT::ConditionNode
  {
  public:
    IsReachGoal(std::string &name , BT::NodeConfig &config , tools::CmdTools &cmd_tools) : ConditionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      auto state = cmd_tools_.mbf_client_->getState();
      if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    tools::CmdTools &cmd_tools_;
  };

}

#endif //NEW_BEHAVIOR_TREE_CHASSIS_CONDITION_NODE_H