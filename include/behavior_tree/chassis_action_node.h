//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_ACTION_NODE_H

#include "behaviortree_cpp/action_node.h"
#include "common/behavior_base.h"
#include "common/tools.h"

namespace chassis
{
  class ChassisSlowGyro : public BT::SyncActionNode  // 用于赛前的慢速小陀螺
  {
  public:
    ChassisSlowGyro(std::string name , BT::NodeConfig config , BehaviorBase & behavior_base , tools::CmdTools & cmd_tools) : BT::SyncActionNode(name,config) , behavior_base_(behavior_base) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<double>("slow_gyro_vel") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<double> msg = getInput<double>("slow_gyro_vel");
      // Check if expected is valid. If not, throw its error
      if (!msg)
      {
        throw BT::RuntimeError("missing required input [slow_gyro_vel]: ",
                                msg.error() );
      }
      // use the method value() to extract the valid message.

      behavior_base_.sendChassisCmd();
      ros::Time time = ros::Time::now();
      cmd_tools_.vel_2d_cmd_sender_->setAngularZVel(msg.value());//在我给出的配置文件中设定底盘的旋转速度
      cmd_tools_.chassis_cmd_sender_->sendChassisCommand(time, true);
      cmd_tools_.vel_2d_cmd_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BehaviorBase & behavior_base_;
    tools::CmdTools & cmd_tools_;
  };

  class AbnormalStillStopAllMotion : public BT::SyncActionNode
  {
  public:
    AbnormalStillStopAllMotion(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , tools::CmdTools & cmd_tools) : BT::SyncActionNode(name,config) , behavior_base_(behavior_base) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      behavior_base_.sendChassisCmd();
      cmd_tools_.vel_2d_cmd_sender_->setZero();
      cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setZero();
      cmd_tools_.union_cmd_sender_->base_gimbal_cmd_sender_->setZero();
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setZero();
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BehaviorBase & behavior_base_;
    tools::CmdTools & cmd_tools_;
  };

  class PatrolAbnormalBackHomeGoal : public BT::StatefulActionNode
  {
  public:
    PatrolAbnormalBackHomeGoal(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_center_sentry_patrol_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

  class PatrolAttackEnemyPositiveArea : public BT::StatefulActionNode
  {
  public:
    PatrolAttackEnemyPositiveArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      if (robot_color == "red") //因为要打击的是对面的机器人，所以要把robot_color反相
      {
        robot_color = "blue";
      }else
      {
        robot_color = "red";
      }
      target_area_name = "attack_" + robot_color + "_positive_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

  class PatrolOwnOutputArea : public BT::StatefulActionNode
  {
  public:
    PatrolOwnOutputArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_outpost_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

  class PatrolEnemyOutpostArea : public BT::StatefulActionNode
  {
  public:
    PatrolEnemyOutpostArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      if (robot_color == "red") //因为要打击的是对面的机器人，所以要把robot_color反相
      {
        robot_color = "blue";
      }else
      {
        robot_color = "red";
      }
      target_area_name = robot_color + "_outpost_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

  class PatrolSentryPatrolArea : public BT::StatefulActionNode
  {
  public:
    PatrolSentryPatrolArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_center_sentry_patrol_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

  class GotoReturnBloodArea : public BT::StatefulActionNode
  {
  public:
    GotoReturnBloodArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      is_complete = false;
      last_patrol_position_index=-1;
      all_zones=blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_supply_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }

      if (is_complete == true)
      {
        return BT::NodeStatus::SUCCESS;
      }

      auto point = tools::getZonesPosition(target_area_name,blackboard_,last_patrol_position_index,true,is_complete);
      behavior_base_.conduct(point);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    BT::Blackboard &blackboard_;
    int last_patrol_position_index;
    bool is_complete;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::string robot_color;
    std::string target_area_name;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
  };

}

#endif //NEW_BEHAVIOR_TREE_ACTION_NODE_H