//
// Created by root on 2026/5/18.
//

#ifndef NEW_BEHAVIOR_TREE_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_ACTION_NODE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <behaviortree_cpp/action_node.h>
#include "common/tools.h"
#include <rm_common/decision/service_caller.h>


class StartMainControllers : public BT::SyncActionNode
{
public:
  StartMainControllers(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name,config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.startMainController();
    controller_tools_.calibrate();
    ros::Duration duration(1.0);
    duration.sleep();
    return BT::NodeStatus::SUCCESS;
  }
private:
  tools::ControllerTools &controller_tools_;
};

class StopMainControllers : public BT::SyncActionNode
{
public:
  StopMainControllers(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name , config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.stopMainController();
    ros::Duration duration(0.5);
    duration.sleep();
    return BT::NodeStatus::SUCCESS;
  }
private:
  tools::ControllerTools &controller_tools_;
};

class VisionCalibrate : public BT::SyncActionNode
{
public:
  VisionCalibrate(const std::string &name , const BT::NodeConfig &config ,ros::NodeHandle &bt_nh ,  perception::Subscriber &subscriber) : SyncActionNode(name , config) , bt_nh_(bt_nh),  detection_switch_nh_(bt_nh,"detection_switch"), subscriber_(subscriber)
  {
    switch_detection_srv_ = std::make_unique<rm_common::SwitchDetectionCaller>(detection_switch_nh_,"/Processor/status_change");
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("robot_color")};
  }

  BT::NodeStatus tick() override
  {
    BT::Expected<std::string> expected = getInput<std::string>("robot_color");
    std::string robot_color = expected.value();
    robot_color == "blue" ? robot_color = "red" : robot_color = "blue"; // 要击打的是敌方，因此颜色反相
    switch_detection_srv_->setEnemyColor(subscriber_.getGameRobotStatus().robot_id, robot_color);

    switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_detection_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_WITHOUT_OUTPOST_BASE);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle &bt_nh_;
  ros::NodeHandle detection_switch_nh_;
  perception::Subscriber &subscriber_;
  std::unique_ptr<rm_common::SwitchDetectionCaller> switch_detection_srv_;
};

class RemoteControlTurnOff : public BT::SyncActionNode
{
public:
  RemoteControlTurnOff(const std::string &name , const BT::NodeConfig &config , tools::CmdTools &cmd_tools , tools::ControllerTools &controller_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    const ros::Time now = ros::Time::now();
    if (controller_tools_.getControllerManager())
    {
      controller_tools_.stopMainController();
      controller_tools_.stopCalibrationController();
    }
    cmd_tools_.getSenders()->vel_2d_command_sender_->setZero();
    cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setZero();
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setZero();
    cmd_tools_.getSenders()->shooter_command_sender_->setZero();
    cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(now, false);
    cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(now);
    cmd_tools_.getSenders()->gimbal_command_sender_->sendCommand(now);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->sendCommand(now);
    cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(now);
    return BT::NodeStatus::SUCCESS;
  }

private:
  tools::CmdTools &cmd_tools_;
  tools::ControllerTools &controller_tools_;
};

class OutputRightSwitchState : public BT::SyncActionNode  // 继承这个同步行为节点
{
public:
  OutputRightSwitchState(const std::string& name, const BT::NodeConfig& config , perception::Subscriber &subscriber , tools::NavigationTools &navigation_tools) : SyncActionNode(name, config) , subscriber_(subscriber) , navigation_tools_(navigation_tools)
  {

  }// 传入感知层，纯读取拨杆状态

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::string>("state")};
  }

  BT::NodeStatus tick() override  // 读取拨杆状态（感知）并根据状态做进入动作（执行），同时输出黑板状态（决策）
  {
    const rm_msgs::DbusData::_s_r_type switch_state = subscriber_.getDbusData().s_r;
    BT::NodeStatus status = BT::NodeStatus::SUCCESS;
    // 未校准且非 idle，拒绝进入自动/手动
    if (switch_state == rm_msgs::DbusData::MID)
    {
      if (state_ == "auto" || state_ == "idle")
      {
        ROS_INFO_STREAM_THROTTLE(0.5,
                                 "mbf client State:" << navigation_tools_.getMbfClient()->getState().isDone());
        navigation_tools_.resetPatrolState();
        navigation_tools_.getMbfClient()->cancelGoal();
        subscriber_.setBackCameraDetected(false);
        subscriber_.setBackCameraDetectionId(0);
        ROS_INFO_THROTTLE(0.5, "enter manual");
      }
      state_ = "manual";
    }
    else if (switch_state == rm_msgs::DbusData::UP)
    {
      state_ = "auto";
    }
    else if (switch_state == rm_msgs::DbusData::DOWN)
    {
      state_ = "idle";
    }
    else
    {
      status = BT::NodeStatus::FAILURE;
      return status;  // 未知状态
    }
    setOutput("state", state_);  // 向行为树输出当前状态
    return status;
  }

private:
  std::string state_;
  perception::Subscriber &subscriber_;
  tools::NavigationTools &navigation_tools_;
};

class SetIdle : public BT::SyncActionNode
{
public:
  SetIdle(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools , tools::CmdTools &cmd_tools , tools::PlannerTools &planner_tools , perception::Publisher &publisher) : SyncActionNode(name , config) , controller_tools_(controller_tools) , cmd_tools_(cmd_tools) , planner_tools_(planner_tools) , publisher_(publisher)
  {

  }

  BT::NodeStatus tick() override
  {
    if (controller_tools_.getControllerManager())
    {
      controller_tools_.startMainController();
      // idle 状态下不启动云台控制器，防止云台因 traj 初始值导致异常旋转
      controller_tools_.getControllerManager()->stopControllers(
          { "controllers/gimbal_controller", "controllers/base_gimbal_controller" });
    }
    controller_tools_.calibrate();
    //视觉校准
    planner_tools_.setGyroSpeed(0);//当前参数文件为0
    cmd_tools_.getSenders()->chassis_command_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    cmd_tools_.getSenders()->chassis_command_sender_->getMsg()->command_source_frame = "base_link";
    cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(ros::Time::now(), false);
    rm_msgs::ManualToReferee manual_to_referee;
    manual_to_referee.stamp = ros::Time::now();
    manual_to_referee.power_limit_state = cmd_tools_.getSenders()->chassis_command_sender_->power_limit_->getState();
    publisher_.getPublishers()->manual_to_referee_pub_.publish(manual_to_referee);// 发布当前的powerlimitstate
    // manual_to_referee_pub_data_.start_burst_time = basic_control_.cmd_tools_.chassis_cmd_sender_->power_limit_->getStartBurstTime();
    return BT::NodeStatus::SUCCESS;
  }

private:
  tools::ControllerTools &controller_tools_;
  tools::CmdTools &cmd_tools_;
  tools::PlannerTools &planner_tools_;
  perception::Publisher &publisher_;
};

class Relocate : public BT::StatefulActionNode
{
public:
  Relocate(const std::string &name , const BT::NodeConfig &config , ros::NodeHandle &bt_nh) : StatefulActionNode(name , config)
  {
    shinji_query_client_ = bt_nh.serviceClient<std_srvs::Empty>("/shinji/query");
  }

  BT::NodeStatus onStart() override
  {
    relocate_status_.store(IDLE);
    canceled_signal_ = std::make_shared<std::atomic_bool>(false);

    std::thread([this , thread_cancel_signal = canceled_signal_]()
    {
      std_srvs::Empty srv;
      if (!shinji_query_client_.exists())
      {
        relocate_status_.store(FAILURE);
        return;
      }
      relocate_status_.store(RUNNING);
      const bool ok = shinji_query_client_.call(srv);
      if (thread_cancel_signal->load() == true) //call完之后，若被外部取消，直接返回，不继续传递状态
      {
        return;
      }
      if (!ok)
      {
        relocate_status_.store(FAILURE);
        ROS_ERROR("shinji_query_client can not get the service respond in Relocate");
      }
      else
      {
        relocate_status_.store(SUCCESS);
      }
    }).detach();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    switch (relocate_status_)
    {
    case IDLE:
      return BT::NodeStatus::RUNNING;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
    case FAILURE:
      return BT::NodeStatus::FAILURE;
    case RUNNING:
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    relocate_status_.store(IDLE);
    canceled_signal_->store(true);
  }

private:
  enum Status { IDLE, RUNNING, SUCCESS, FAILURE };
  std::shared_ptr<std::atomic_bool> canceled_signal_;//线程安全
  std::atomic<Status> relocate_status_{IDLE};
  ros::ServiceClient shinji_query_client_;
};

class ReviveIfDead : public BT::SyncActionNode
  {
  public:
    ReviveIfDead(const std::string& name, const BT::NodeConfiguration& config, tools::CmdTools &cmd_tools ,perception::Subscriber &subscriber ,
                   tools::NavigationTools &navigation_tools , tools::ControllerTools &controller_tools)
      : BT::SyncActionNode(name, config),cmd_tools_(cmd_tools), subscriber_(subscriber)  , navigation_tools_(navigation_tools) , controller_tools_(controller_tools) {}

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
        controller_tools_.stopMainController();
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
        if (now - revival_time_ < ros::Duration(0.8))  // 复活后短暂等待+校准
        {
          controller_tools_.startMainController();  // 重启主要控制器
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
    tools::NavigationTools &navigation_tools_;
    tools::ControllerTools &controller_tools_;
  };

class RelieveWeakState : public BT::StatefulActionNode
{
public:
  RelieveWeakState(const std::string& name , const BT::NodeConfig &config , perception::Subscriber &subscriber) : StatefulActionNode(name , config) , subscriber_(subscriber)
  {

  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("need_supply"),
                  BT::OutputPort<bool>("need_supply_output"),
              BT::OutputPort<int>("chassis_mode")};
  }

  BT::NodeStatus onStart() override
  {
    bool need_supply;
    getInput<bool>("need_supply",need_supply);
    if (need_supply == true)
    {
      return BT::NodeStatus::RUNNING;
    }else
    {
      return BT::NodeStatus::SUCCESS;
    }
  }

  BT::NodeStatus onRunning() override
  {
    if (subscriber_.getGameRobotStatus().remain_hp >= subscriber_.getGameRobotStatus().max_hp)
    {
      setOutput("need_supply_output",false); //重置状态
      return BT::NodeStatus::SUCCESS;
    }else
    {
      setOutput("chassis_mode",7);
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override
  {

  }
private:
  perception::Subscriber &subscriber_;
};

class Test1 : public BT::SyncActionNode
{
public:
  Test1(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name , config)
  {

  }

  BT::NodeStatus tick() override
  {
    ROS_INFO_THROTTLE(0.5,"Test1");
    return BT::NodeStatus::SUCCESS;
  }
};

class Test2 : public BT::SyncActionNode
{
public:
  Test2(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name , config)
  {

  }

  BT::NodeStatus tick() override
  {
    ROS_INFO_THROTTLE(0.5,"Test2");
    return BT::NodeStatus::SUCCESS;
  }
};


#endif //NEW_BEHAVIOR_TREE_ACTION_NODE_H