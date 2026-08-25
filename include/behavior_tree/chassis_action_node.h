//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H

#include <behaviortree_cpp/condition_node.h>
#include <rm_common/decision/controller_manager.h>

#include "behaviortree_cpp/action_node.h"
#include "common/tools.h"
#include "service_processor/SearchEnablePoint.h"

namespace chassis
{
  class ChassisSlowGyro : public BT::StatefulActionNode  // 用于赛前的慢速小陀螺
  {
  public:
    ChassisSlowGyro(const std::string &name ,const BT::NodeConfig &config  , tools::CmdTools & cmd_tools) : BT::StatefulActionNode(name,config) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<double>("slow_gyro_vel_scale") };
    }

    BT::NodeStatus onStart() override
    {
      BT::Expected<double> msg = getInput<double>("slow_gyro_vel_scale");
      // Check if expected is valid. If not, throw its error
      if (!msg)
      {
        throw BT::RuntimeError("missing required input [slow_gyro_vel_scale]: ",
                                msg.error() );
      }
      slow_gyro_vel_scale = msg.value();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

      // use the method value() to extract the valid message.

      cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
      cmd_tools_.getSenders()->chassis_command_sender_->getMsg()->command_source_frame = "base_link";
      ros::Time time = ros::Time::now();
      cmd_tools_.getSenders()->vel_2d_command_sender_->setAngularZVel(slow_gyro_vel_scale);//在我给出的配置文件中设定底盘的旋转速度
      cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(time, true);
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(time);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->chassis_command_sender_->setZero();
      cmd_tools_.getSenders()->vel_2d_command_sender_->setZero();
    }
  private:
    tools::CmdTools & cmd_tools_;
    double slow_gyro_vel_scale{};
  };

  class AbnormalStillStopAllMotion : public BT::SyncActionNode
  {
  public:
    AbnormalStillStopAllMotion(const std::string & name ,const BT::NodeConfig & config , tools::CmdTools & cmd_tools) : BT::SyncActionNode(name,config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      cmd_tools_.getSenders()->vel_2d_command_sender_->setZero();
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
      cmd_tools_.getSenders()->chassis_command_sender_->setZero();
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(ros::Time::now());
      cmd_tools_.getSenders()->gimbal_command_sender_->sendCommand(ros::Time::now());
      cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(ros::Time::now(),false);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools & cmd_tools_;
  };

  class PatrolAbnormalBackHomeGoal : public BT::StatefulActionNode
  {
  public:
    PatrolAbnormalBackHomeGoal(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) ,navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_center_sentry_patrol_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),2.0,false,false,false);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class PatrolAttackEnemyPositiveArea : public BT::StatefulActionNode
  {
  public:
    PatrolAttackEnemyPositiveArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard ,tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class PatrolOwnOutpostArea : public BT::StatefulActionNode
  {
  public:
    PatrolOwnOutpostArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_outpost_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class PatrolEnemyOutpostArea : public BT::StatefulActionNode
  {
  public:
    PatrolEnemyOutpostArea(const std::string & name ,const BT::NodeConfig & config ,tools::NavigationTools &navigation_tools) : StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color") };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=getInput<std::string>("robot_color").value();
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),10.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class PatrolSentryPatrolArea : public BT::StatefulActionNode
  {
  public:
    PatrolSentryPatrolArea(const std::string & name ,const BT::NodeConfig & config , tools::NavigationTools &navigation_tools) : StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color") };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=getInput<std::string>("robot_color").value();
      target_area_name = robot_color + "_center_sentry_patrol_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoReturnBloodArea : public BT::StatefulActionNode
  {
  public:
    GotoReturnBloodArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_supply_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5,false,false,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class PatrolHoleUpArea : public BT::StatefulActionNode
  {
  public:
    PatrolHoleUpArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      robot_color=blackboard_.get<std::string>("robot_color");
      target_area_name = robot_color + "_hole_up_points";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoEnemyBaseArea : public BT::StatefulActionNode
  {
  public:
    GotoEnemyBaseArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoAttackEnemyEngineer : public BT::StatefulActionNode
  {
  public:
    GotoAttackEnemyEngineer(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    BT::Blackboard &blackboard_;
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoConductPoint : public BT::StatefulActionNode
  {
  public:
    GotoConductPoint(const std::string & name ,const BT::NodeConfig & config , perception::Subscriber &subscriber , tools::NavigationTools &navigation_tools , tools::MiniMapTools &mini_map_tools) : BT::StatefulActionNode(name,config) ,subscriber_(subscriber) , navigation_tools_(navigation_tools) , mini_map_tools_(mini_map_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      target_pose_ = mini_map_tools_.getConductPoint();;
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(target_pose_,5,true,false,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelAllGoals();
      navigation_tools_.resetPatrolState();
    }

  private:
    geometry_msgs::PoseStamped target_pose_;
    perception::Subscriber & subscriber_;
    tools::NavigationTools &navigation_tools_;
    tools::MiniMapTools &mini_map_tools_;
  };

  class ChaseEnemy : public BT::StatefulActionNode // TODO : 未完成追击优先级判断
  {
  public:
    ChaseEnemy(const std::string & name ,const BT::NodeConfig & config , tools::NavigationTools &navigation_tools) : StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.chase();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetLastTargetAtMap();
    }

  private:
    tools::NavigationTools &navigation_tools_;
  };

  class SetChassisMode : public BT::SyncActionNode
  {
  public:
    SetChassisMode(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name,config)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("input_chassis_mode_id"),
                BT::OutputPort<int>("chassis_mode")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> input_chassis_mode_id = getInput<int>("input_chassis_mode_id");
      setOutput<int>("chassis_mode",input_chassis_mode_id.value());
      return BT::NodeStatus::SUCCESS;
    }

  private:

  };


  class SetIsEnableFight : public BT::SyncActionNode
  {
  public:
    SetIsEnableFight(const std::string &name ,const BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name,config) , blackboard_(blackboard)
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

  class GetKeyboardCommand : public BT::SyncActionNode
  {
  public:
    GetKeyboardCommand(const std::string &name ,const BT::NodeConfig &config , perception::Subscriber &subscriber) : SyncActionNode(name , config) , subscriber_(subscriber)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {BT::OutputPort<uint8_t>("keyboard_command")};
    }

    BT::NodeStatus tick() override
    {
      setOutput("keyboard_command",subscriber_.msgGetter<rm_msgs::ClientMapSendData>(perception::Subscriber::TopicId::CLIENT_MAP_SEND_DATA).message.command_keyboard);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    perception::Subscriber &subscriber_;
  };

  class SetGyroInCombat : public BT::SyncActionNode
  {
  public:
    SetGyroInCombat(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::vector<double>>("standby_velocity")}; //TODO : 获取参数处未完成参数类型的适配
    }

    BT::NodeStatus tick() override
    {
      ros::Time time = ros::Time::now();
      std::vector<double> standby_velocity;
      getInput<std::vector<double>>("standby_velocity",standby_velocity);
      cmd_tools_.getSenders()->vel_2d_command_sender_->set2DVel(0.0, 0.0,
                                                standby_velocity[0] *
                                                        std::sin(ros::Time::now().toSec()) +
                                                    standby_velocity[1]);
      cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(time, true);
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };

  class PatrolTestArea : public BT::StatefulActionNode
  {
  public:
    PatrolTestArea(const std::string & name ,const BT::NodeConfig & config  , tools::NavigationTools &navigation_tools , tools::PlannerTools &planner_tools , tools::CmdTools &cmd_tools) : BT::StatefulActionNode(name,config)  , navigation_tools_(navigation_tools) , planner_tools_(planner_tools) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {  };
    }

    BT::NodeStatus onStart() override
    {
      target_area_name = "test_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      planner_tools_.setLimitVelAndSlideWindow(8.2,0.2);
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false,false,false);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      planner_tools_.setLimitVelAndSlideWindow(5.5, 0.2);
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
      cmd_tools_.getSenders()->vel_2d_command_sender_->setZero();
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(ros::Time::now());
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
    tools::PlannerTools &planner_tools_;
    tools::CmdTools &cmd_tools_;
  };

  class GotoTrapezoidArea : public BT::StatefulActionNode
  {
  public:
    GotoTrapezoidArea(const std::string & name ,const BT::NodeConfig & config , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color") };
    }

    BT::NodeStatus onStart() override
    {
      robot_color = getInput<std::string>("robot_color").value();
      target_area_name = robot_color + "_trapezoid_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoBaseDefenceArea : public BT::StatefulActionNode
  {
  public:
    GotoBaseDefenceArea(const std::string & name ,const BT::NodeConfig & config , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color") };
    }

    BT::NodeStatus onStart() override
    {
      robot_color = getInput<std::string>("robot_color").value();
      target_area_name = robot_color + "_base_defence_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

  class GotoOwnFortress : public BT::StatefulActionNode
  {
  public:
    GotoOwnFortress(const std::string & name ,const BT::NodeConfig & config , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color") };
    }

    BT::NodeStatus onStart() override
    {
      robot_color = getInput<std::string>("robot_color").value();
      target_area_name = robot_color + "_fortress_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5.0,false,true,true);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      navigation_tools_.getMbfClient()->cancelGoal();
      navigation_tools_.resetPatrolState();
    }

  private:
    std::string robot_color;
    std::string target_area_name;
    tools::NavigationTools &navigation_tools_;
  };

}

#endif //NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H