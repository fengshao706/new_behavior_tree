//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H

#include <rm_common/decision/controller_manager.h>

#include "behaviortree_cpp/action_node.h"
#include "common/tools.h"
#include "service_processor/SearchEnablePoint.h"

namespace chassis
{
  class ChassisSlowGyro : public BT::SyncActionNode  // 用于赛前的慢速小陀螺
  {
  public:
    ChassisSlowGyro(const std::string &name ,const BT::NodeConfig &config  , tools::CmdTools & cmd_tools) : BT::SyncActionNode(name,config) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<double>("slow_gyro_vel_scale") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<double> msg = getInput<double>("slow_gyro_vel_scale");
      // Check if expected is valid. If not, throw its error
      if (!msg)
      {
        throw BT::RuntimeError("missing required input [slow_gyro_vel_scale]: ",
                                msg.error() );
      }
      // use the method value() to extract the valid message.

      cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
      cmd_tools_.getSenders()->chassis_command_sender_->getMsg()->command_source_frame = "base_link";
      ros::Time time = ros::Time::now();
      cmd_tools_.getSenders()->vel_2d_command_sender_->setAngularZVel(msg.value());//在我给出的配置文件中设定底盘的旋转速度
      cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(time, true);
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools & cmd_tools_;
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),2.0,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false);
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
    PatrolEnemyOutpostArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard ,tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
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
      target_area_name = robot_color + "_outpost_area";
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),10.0,false);
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

  class PatrolSentryPatrolArea : public BT::StatefulActionNode
  {
  public:
    PatrolSentryPatrolArea(const std::string & name ,const BT::NodeConfig & config , BT::Blackboard & blackboard , tools::NavigationTools &navigation_tools) : BT::StatefulActionNode(name,config) , blackboard_(blackboard) , navigation_tools_(navigation_tools)
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,false),5.0,false);
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
      navigation_tools_.patrol(navigation_tools_.getPatrolPoint(target_area_name,true),5.0,false);
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
      navigation_tools_.patrol(target_pose_,5,true);
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
    SetChassisMode(const std::string &name ,const BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name,config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
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
    ReviveIfDead(const std::string& name, const BT::NodeConfiguration& config, tools::CmdTools &cmd_tools ,perception::Subscriber &subscriber ,
                 double wait_time , rm_common::ControllerManager &controller_manager , tools::NavigationTools &navigation_tools , tools::ControllerTools &controller_tools)
      : BT::SyncActionNode(name, config),cmd_tools_(cmd_tools), subscriber_(subscriber) ,wait_time_(wait_time) , controller_manager_(controller_manager) , navigation_tools_(navigation_tools) , controller_tools_(controller_tools) {}

    static BT::PortsList providedPorts()
    {
      return {
        BT::OutputPort<bool>("self_is_weak"),
        BT::OutputPort<ros::Time>("self_weak_until"),
        BT::OutputPort<bool>("need_supply"),
        BT::OutputPort<bool>("has_revived"),
        BT::OutputPort<bool>("confirm_respawn")
      };

    }

    BT::NodeStatus tick() override
    {
      BT::NodeStatus status = BT::NodeStatus::SUCCESS;
      const ros::Time now = ros::Time::now();
      if (subscriber_.getGameRobotStatus().remain_hp == 0)  // 如果订阅到哨兵的剩余血量为0，则关闭主要控制器
      {
        controller_manager_.stopMainControllers();
        is_dead_=true;  // 若血量为零则判定为死亡
        revival_time_ = ros::Time(0.);                    // 等待检测到复活瞬间后重新计时
        setOutput("confirm_respawn",true); // 挂到共享 sentry_cmd，等待后续统一发布
        setOutput("need_supply",true);  // 死亡后复活是残血，表明复活后需要回家补血
        navigation_tools_.getMbfClient()->cancelGoal();  // 在机器人"死亡"或复活时，取消之前设定的目标，以防止机器人执行与当前状态不一致的动作例如继续追击等等
        ROS_INFO_THROTTLE(0.5, "Reviving...");
        return BT::NodeStatus::FAILURE;
      }
      else if (is_dead_ == true) //首次检测到HP从0恢复
      {
        setOutput("has_revived",true);  // 设置一个标志表明哨兵的复活过程已经完成
        if (revival_time_.isZero())
        {
          revival_time_ = now;  // 首次检测到 HP 从 0 恢复，开始复活后等待窗口
          setOutput("self_is_weak",true);//使用黑板向外传值
          setOutput("self_weak_until",now + ros::Duration(30.0));
        }
        if (now - revival_time_ < ros::Duration(wait_time_))  // 复活后短暂等待+校准
        {
          controller_tools_.getControllerManager()->startMainControllers();  // 重启主要控制器
          controller_tools_.calibrate();                                    // 执行校准函数
          cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
          cmd_tools_.getSenders()->chassis_command_sender_->getMsg()->command_source_frame =
              "base_link";                                                                 // 指定命令坐标系为base_link
          cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(now, false);  // 发送底盘命令
          ROS_INFO_THROTTLE(0.5, "Calibrating...");
          return BT::NodeStatus::FAILURE;
        }
        is_dead_ = false;
      }
      return BT::NodeStatus::SUCCESS;
    }

  private:
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
    bool is_dead_{ false };
    ros::Time revival_time_{ 0. };
    double wait_time_;
    rm_common::ControllerManager &controller_manager_;
    tools::NavigationTools &navigation_tools_;
    tools::ControllerTools &controller_tools_;
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
      setOutput("keyboard_command",subscriber_.getClientMapSendData().command_keyboard);
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

}

#endif //NEW_BEHAVIOR_TREE_CHASSIS_ACTION_NODE_H