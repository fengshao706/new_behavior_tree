//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_ACTION_NODE_H

#include <common/navigation_bridge.h>
#include <rm_common/decision/controller_manager.h>

#include "behaviortree_cpp/action_node.h"
#include "common/behavior_base.h"
#include "common/tools.h"
#include "service_processor/SearchEnablePoint.h"

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

  class PatrolHoleUpArea : public BT::StatefulActionNode
  {
  public:
    PatrolHoleUpArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
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
      target_area_name = robot_color + "_hole_up_points";
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

  class GotoEnemyBaseArea : public BT::StatefulActionNode
  {
  public:
    GotoEnemyBaseArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
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
      target_area_name = robot_color + "_base_area";
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

  class GotoAttackEnemyEngineer : public BT::StatefulActionNode
  {
  public:
    GotoAttackEnemyEngineer(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
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
      target_area_name ="wait_for_" + robot_color + "_engineer";
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

  class PatrolAfterRevivePatrolArea : public BT::StatefulActionNode
  {
  public:
    PatrolAfterRevivePatrolArea(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , BT::Blackboard & blackboard , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
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
      target_area_name =robot_color + "_outpost_area";
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

  class GotoConductPoint : public BT::StatefulActionNode
  {
  public:
    GotoConductPoint(std::string & name , BT::NodeConfig & config , BehaviorBase & behavior_base , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber) : BT::StatefulActionNode(name,config) ,  cmd_tools_(cmd_tools) , behavior_base_(behavior_base) ,subscriber_(subscriber)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      geometry_msgs::PoseStamped target_pose;
      navigation_bridge_.targetPoseTransform(subscriber_.client_map_send_data_.target_position_x,
                                             subscriber_.client_map_send_data_.target_position_y, &target_pose);
      behavior_base_.conduct(target_pose);
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      auto state = cmd_tools_.mbf_client_->getState();  //用于查看是否到达目标点
      if (state.state_==actionlib::SimpleClientGoalState::ACTIVE)
      {
        return BT::NodeStatus::RUNNING;
      }
      if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::FAILURE;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    geometry_msgs::PoseStamped point;
    tools::CmdTools &cmd_tools_;
    BehaviorBase & behavior_base_;
    NavigationBridge navigation_bridge_;
    perception::Subscriber & subscriber_;
  };

  class CreateMbfClient : public BT::SyncActionNode // TODO : 当检测到mbf掉线的时候需要调用它
  {
  public:
    CreateMbfClient(std::string &name , BT::NodeConfig &config , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus tick() override
    {
      cmd_tools_.mbf_client_.reset();
      cmd_tools_.mbf_client_ = std::make_unique<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>(
          "/move_base_flex/move_base", true);
      return BT::NodeStatus::SUCCESS;
    }

  private:
    tools::CmdTools & cmd_tools_;
  };

  class ChaseEnemy : public BT::StatefulActionNode  // TODO : 这个节点也需要检查mbf服务是否在线的前置condition node
  {
  public:
    ChaseEnemy(std::string & name , BT::NodeConfig & config , BT::Blackboard &blackboard , perception::Subscriber & subscriber , tools::CmdTools &cmd_tools , ros::NodeHandle &nh , BehaviorBase &behavior_base) : StatefulActionNode(name,config) , blackboard_(blackboard) ,subscriber_(subscriber) , cmd_tools_(cmd_tools) , tf_buffer_(cmd_tools.getTfBuffer()) , nh_(nh) , behavior_base_(behavior_base)
    {
      search_enable_point_client_ = nh_.serviceClient<service_processor::SearchEnablePoint>("/get_enable_point");
    }

    BT::NodeStatus onStart() override
    {
      try
      {
        chase_freq=blackboard_.get<double>("chase_freq");
      }catch (BT::RuntimeError & e)
      {
        ROS_ERROR("BT can not access key name [chase_freq] , default value is 15.0");
        chase_freq=3.0;
      }
      try
      {
        chase_distance=blackboard_.get<double>("chase_distance");
      }catch (BT::RuntimeError & e)
      {
        ROS_ERROR("BT can not access key name [chase_distance] , default value is 2.0");
        chase_distance=2.0;
      }
      try
      {
        chase_tolerance=blackboard_.get<double>("chase_tolerance");
      }catch (BT::RuntimeError & e)
      {
        ROS_ERROR("BT can not access key name [chase_tolerance] , default value is 0.5");
        chase_tolerance=0.5;
      }
      last_chase_time_=ros::Time::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time time = ros::Time::now();

      if (time - last_chase_time_ > ros::Duration(1 / chase_freq))
      {
        geometry_msgs::PointStamped target_at_map;
        try
        {
          track_point_.point = subscriber_.track_data_.position;
          geometry_msgs::TransformStamped transform_stamped =
              tf_buffer_.lookupTransform("map", subscriber_.track_data_.header.frame_id, ros::Time(0));

          tf2::doTransform(track_point_, target_at_map, transform_stamped);
        }
        catch (tf2::TransformException& ex)
        {
          ROS_ERROR("Failed to transform point: %s", ex.what());
        }
        try
        {
          cur_map2base_ = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
        service_processor::SearchEnablePoint srv;
        srv.request.target_pos = target_at_map;
        srv.request.robot_pos = cur_map2base_;
        srv.request.chase_distance = chase_distance;
        srv.request.chase_tolerance = chase_tolerance;

        if (search_enable_point_client_.call(srv))
        {
          mbf_msgs::MoveBaseGoal mbf_goal;
          mbf_goal.target_pose = srv.response.move_point;
          mbf_goal.direct_track = !srv.response.is_block_on_line;
          cmd_tools_.mbf_client_->sendGoal(mbf_goal);
          last_chase_time_ = ros::Time::now();
          return BT::NodeStatus::RUNNING;
        }
        else
        {
          last_chase_time_ = ros::Time::now();
          return BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      behavior_base_.cancelGoal();
    }

  private:
    double chase_freq;
    double chase_distance;
    double chase_tolerance;
    BT::Blackboard& blackboard_;
    ros::Time last_chase_time_;
    geometry_msgs::PointStamped track_point_;
    perception::Subscriber &subscriber_;
    geometry_msgs::TransformStamped cur_map2base_;
    ros::ServiceClient search_enable_point_client_;
    tools::CmdTools &cmd_tools_;
    tf2_ros::Buffer &tf_buffer_;
    ros::NodeHandle &nh_;
    BehaviorBase &behavior_base_;
  };

  class SetChassisMode : public BT::SyncActionNode
  {
  public:
    SetChassisMode(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providePorts()
    {
      return { BT::InputPort<int>("chassis_mode_id") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> chassis_mode_id = getInput<int>("chassis_mode_id");
      blackboard_.set<types::ChassisMode>("chassis_mode",static_cast<types::ChassisMode>(chassis_mode_id.value()));
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Blackboard &blackboard_;
  };


  class ReviveIfDead : public BT::SyncActionNode
  {
  public:
    ReviveIfDead(const std::string& name, const BT::NodeConfiguration& config, rm_common::ControllerManager &controller_manager, tools::CmdTools &cmd_tools ,perception::Subscriber &subscriber ,
                 double wait_time , BT::Blackboard &blackboard , BehaviorBase &behavior_base)
      : BT::SyncActionNode(name, config), controller_manager_(controller_manager),cmd_tools_(cmd_tools), subscriber_(subscriber) ,wait_time_(wait_time) , blackboard_(blackboard) ,behavior_base_(behavior_base) {}

    static BT::PortsList providedPorts()
    {
      BT::PortsList ports_list;
      return ports_list;
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.game_robot_status_.remain_hp == 0)
      {
        controller_manager_.stopMainControllers();
        is_dead_ = true; //没血了就停止主控制器并把死亡标志置为1
        rm_msgs::SentryCmd sentry_cmd;
        sentry_cmd.sentry_info = 1;
        subscriber_.sentry_cmd_pub_.publish(sentry_cmd);
        blackboard_.set<bool>("need_supply",true);
        blackboard_.set<bool>("has_revived",true);

        behavior_base_.cancelGoal();
        ROS_INFO("Reviving...");
        return BT::NodeStatus::RUNNING; //没复活的时候就一直将状态置为FAIL，不断尝试开启整棵子树
      }
      else if (ros::Time::now() - revival_time_ < ros::Duration(wait_time_))      //当前时刻与刚复活的时刻的时间差小于我们设定的等待时间就认为还没有校准完成
      {
        controller_manager_.startMainControllers();
        behavior_base_.calibrate();
        cmd_tools_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        cmd_tools_.chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
        cmd_tools_.chassis_cmd_sender_->sendChassisCommand(ros::Time::now(), false);
        ROS_INFO("Calibrating...");
        return BT::NodeStatus::RUNNING;
      }
      else
      {
        if (is_dead_)
        {
          revival_time_ = ros::Time::now();
          is_dead_ = false;
        }
        return BT::NodeStatus::SUCCESS;
      }
    };

  private:
    rm_common::ControllerManager &controller_manager_;
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
    bool is_dead_{ false };
    ros::Time revival_time_{ 0. };
    double wait_time_;
    BT::Blackboard &blackboard_;
    BehaviorBase &behavior_base_;
  };

  class SetIsEnableFight : public BT::SyncActionNode
  {
  public:
    SetIsEnableFight(std::string &name ,BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts(){
      return { BT::InputPort<bool>("is_enable_fight") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<bool> is_enable_fight = getInput<bool>("is_enable_fight");
      blackboard_.set<bool>("enable_fight",is_enable_fight.value());
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class GetPresentTime : public BT::SyncActionNode
  {
  public:
    GetPresentTime(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , perception::Subscriber &subscriber) : SyncActionNode(name,config) , blackboard_(blackboard) , subscriber_(subscriber)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<double>("present_time") };
    }

    BT::NodeStatus tick() override
    {
      double game_total_time = blackboard_.get<double>("game_total_time");
      double present_time;
      present_time = game_total_time-subscriber_.game_status_.stage_remain_time;
      setOutput<double>("present_time",present_time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;

  };

  class GetKeyboardCommand : public BT::SyncActionNode
  {
  public:
    GetKeyboardCommand(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , perception::Subscriber &subscriber) : SyncActionNode(name , config) , blackboard_(blackboard) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      auto keyboard_command = subscriber_.client_map_send_data_.command_keyboard;
      blackboard_.set<uint8_t>("keyboard_command",keyboard_command);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;
  };

}

#endif //NEW_BEHAVIOR_TREE_ACTION_NODE_H