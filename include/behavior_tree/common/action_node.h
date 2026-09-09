//
// Created by root on 2026/5/18.
//

#ifndef NEW_BEHAVIOR_TREE_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_ACTION_NODE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <behaviortree_cpp/action_node.h>
#include "common/tools.h"
#include "common/dx_track_switch_caller.h"
#include "common/invincible_detection.h"
#include "common/posture_manager.h"


class StartMainControllers : public BT::SyncActionNode
{
public:
  StartMainControllers(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name,config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.startMainController();
    ros::Duration duration(0.5);
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

class StartCalibrationController : public BT::SyncActionNode
{
public:
  StartCalibrationController(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name, config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.calibrate();   // 触发自动校准流：停 shooter → 起 trigger → 完成自动恢复
    ros::Duration duration(0.5);
    duration.sleep();
    return BT::NodeStatus::SUCCESS;
  }
private:
  tools::ControllerTools &controller_tools_;
};

class StopCalibrationController : public BT::SyncActionNode
{
public:
  StopCalibrationController(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name, config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.stopCalibration();   // 强制中止校准并恢复全部主控制器
    return BT::NodeStatus::SUCCESS;
  }
private:
  tools::ControllerTools &controller_tools_;
};

class StartStateControllers : public BT::SyncActionNode
{
public:
  StartStateControllers(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name,config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.startStateController();
    ros::Duration duration(1.0);
    duration.sleep();
    return BT::NodeStatus::SUCCESS;
  }
private:
  tools::ControllerTools &controller_tools_;
};

class StopStateControllers : public BT::SyncActionNode
{
public:
  StopStateControllers(const std::string &name , const BT::NodeConfig &config , tools::ControllerTools &controller_tools) : SyncActionNode(name , config) , controller_tools_(controller_tools)
  {

  }

  BT::NodeStatus tick() override
  {
    controller_tools_.stopStateController();
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
  VisionCalibrate(const std::string &name , const BT::NodeConfig &config ,
                  tools::AutoAimTools &auto_aim_tools) : SyncActionNode(name , config) , auto_aim_tools_(auto_aim_tools)
  {
  }

  BT::NodeStatus tick() override
  {
    auto_aim_tools_.restoreArmorWindow();
    return BT::NodeStatus::SUCCESS;
  }

private:
  tools::AutoAimTools &auto_aim_tools_;
};

class RemoteControlTurnOff : public BT::SyncActionNode
{
public:
  RemoteControlTurnOff(const std::string &name , const BT::NodeConfig &config , tools::CmdTools &cmd_tools , tools::ControllerTools &controller_tools , tools::NavigationTools &navigation_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools) , controller_tools_(controller_tools) , navigation_tools_(navigation_tools)
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
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setZero();
    cmd_tools_.getSenders()->shooter_command_sender_->setZero();
    cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(now, false);
    cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(now);
    cmd_tools_.getSenders()->gimbal_command_sender_->sendCommand(now);
    cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(now);
    navigation_tools_.getMbfClient()->cancelGoal();
    return BT::NodeStatus::SUCCESS;
  }

private:
  tools::CmdTools &cmd_tools_;
  tools::ControllerTools &controller_tools_;
  tools::NavigationTools &navigation_tools_;
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
    const rm_msgs::DbusData::_s_r_type switch_state = subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.s_r;
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
      if (subscriber_.msgGetter<rm_msgs::GameRobotStatus>(perception::Subscriber::TopicId::GAME_ROBOT_STATUS).message.remain_hp == 0)  // 如果订阅到哨兵的剩余血量为0，则关闭主要控制器
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
    if (subscriber_.msgGetter<rm_msgs::GameRobotStatus>(perception::Subscriber::TopicId::GAME_ROBOT_STATUS).message.remain_hp >= subscriber_.msgGetter<rm_msgs::GameRobotStatus>(perception::Subscriber::TopicId::GAME_ROBOT_STATUS).message.max_hp)
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

class SetControlModes : public BT::SyncActionNode
{
public:
  SetControlModes(const std::string& name, const BT::NodeConfig& config, tools::CmdTools& cmd_tools)
    : SyncActionNode(name, config), cmd_tools_(cmd_tools)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("chassis_mode"),
             BT::InputPort<std::string>("gimbal_mode"),
             BT::InputPort<std::string>("shooter_mode") };
  }

  BT::NodeStatus tick() override
  {
    const bool chassis_ok = setChassisMode();
    if (!chassis_ok) return BT::NodeStatus::FAILURE;
    const bool gimbal_ok = setGimbalMode();
    if (!gimbal_ok) return BT::NodeStatus::FAILURE;
    const bool shooter_ok = setShooterMode();
    if (!shooter_ok) return BT::NodeStatus::FAILURE;

    // 三个模式都设置成功后再统一发送，保证发布生效
    cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(ros::Time::now(), false);
    cmd_tools_.getSenders()->gimbal_command_sender_->sendCommand(ros::Time::now());
    cmd_tools_.getSenders()->shooter_command_sender_->checkError(ros::Time::now());
    cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(ros::Time::now());
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool getPort(const char* port, std::string& value)
  {
    const auto expected = getInput<std::string>(port);
    if (!expected)
    {
      ROS_WARN("%s: %s not provided: %s", name().c_str(), port, expected.error().c_str());
      return false;
    }
    value = expected.value();
    return true;
  }

  bool setChassisMode()
  {
    std::string cmd;
    if (!getPort("chassis_mode", cmd)) return false;
    static const std::unordered_map<std::string, uint8_t> kChassisModes{
      {"raw", rm_msgs::ChassisCmd::RAW}, {"follow", rm_msgs::ChassisCmd::FOLLOW},
      {"twist", rm_msgs::ChassisCmd::TWIST}, {"up_slope", rm_msgs::ChassisCmd::UP_SLOPE},
      {"fallen", rm_msgs::ChassisCmd::FALLEN}, {"deploy", rm_msgs::ChassisCmd::DEPLOY},
      {"recovery", rm_msgs::ChassisCmd::RECOVERY} };
    const auto it = kChassisModes.find(cmd);
    if (it == kChassisModes.end())
    {
      ROS_WARN("%s: unknown chassis_mode \"%s\"", name().c_str(), cmd.c_str());
      return false;
    }
    cmd_tools_.getSenders()->chassis_command_sender_->setMode(it->second);
    return true;
  }

  bool setGimbalMode()
  {
    std::string cmd;
    if (!getPort("gimbal_mode", cmd)) return false;
    static const std::unordered_map<std::string, uint8_t> kGimbalModes{
      {"rate", rm_msgs::GimbalCmd::RATE}, {"track", rm_msgs::GimbalCmd::TRACK},
      {"direct", rm_msgs::GimbalCmd::DIRECT}, {"traj", rm_msgs::GimbalCmd::TRAJ} };
    const auto it = kGimbalModes.find(cmd);
    if (it == kGimbalModes.end())
    {
      ROS_WARN("%s: unknown gimbal_mode \"%s\"", name().c_str(), cmd.c_str());
      return false;
    }
    cmd_tools_.getSenders()->gimbal_command_sender_->setMode(it->second);
    return true;
  }

  bool setShooterMode()
  {
    std::string cmd;
    if (!getPort("shooter_mode", cmd)) return false;
    static const std::unordered_map<std::string, uint8_t> kShooterModes{
      {"stop", rm_msgs::ShootCmd::STOP}, {"ready", rm_msgs::ShootCmd::READY},
      {"push", rm_msgs::ShootCmd::PUSH} };
    const auto it = kShooterModes.find(cmd);
    if (it == kShooterModes.end())
    {
      ROS_WARN("%s: unknown shooter_mode \"%s\"", name().c_str(), cmd.c_str());
      return false;
    }
    cmd_tools_.getSenders()->shooter_command_sender_->setMode(it->second);
    return true;
  }

  tools::CmdTools& cmd_tools_;
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


class UpdateEnemyInvincibleState : public BT::SyncActionNode
{
public:
  UpdateEnemyInvincibleState(const std::string &name, const BT::NodeConfig &config,
                     invincible_detection::EnemyInvincibilityManager &enemy_hp_state_tracker,
                     perception::Subscriber &subscriber)
    : SyncActionNode(name, config), enemy_hp_state_tracker_(enemy_hp_state_tracker), subscriber_(subscriber)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<types::EnemyInvincibleTable>("enemy_invincible_table") };
  }

  BT::NodeStatus tick() override
  {
    const auto radar_hp = subscriber_.msgGetter<rm_msgs::RadarWirelessEnemyRobotHp>(
        perception::Subscriber::TopicId::RADAR_WIRELESS_ENEMY_ROBOT_HP);
    const ros::Time now = ros::Time::now();
    enemy_hp_state_tracker_.updateEnemyPositions(); // 显式驱动位置/区域链（snapshot 内也懒刷，此处幂等）
    const ros::Time receive_stamp = radar_hp.stamp; // 接收时刻，非零且单调
    const auto &hp = radar_hp.message;

    enemy_hp_state_tracker_.lifeObserve(1, hp.hero_hp, false, receive_stamp, now);
    enemy_hp_state_tracker_.lifeObserve(2, hp.engineer_hp, false, receive_stamp, now);
    enemy_hp_state_tracker_.lifeObserve(3, hp.infantry_3_hp, false, receive_stamp, now);
    enemy_hp_state_tracker_.lifeObserve(4, hp.infantry_4_hp, false, receive_stamp, now);
    enemy_hp_state_tracker_.lifeObserve(5, hp.reserved, false, receive_stamp, now);
    enemy_hp_state_tracker_.lifeObserve(6, hp.sentry_hp, true, receive_stamp, now);

    types::EnemyInvincibleTable table;
    table.hero       = enemy_hp_state_tracker_.snapshot(1, now);
    table.engineer   = enemy_hp_state_tracker_.snapshot(2, now);
    table.infantry_3 = enemy_hp_state_tracker_.snapshot(3, now);
    table.infantry_4 = enemy_hp_state_tracker_.snapshot(4, now);
    table.aerial     = enemy_hp_state_tracker_.snapshot(5, now);
    table.sentry     = enemy_hp_state_tracker_.snapshot(6, now);
    setOutput("enemy_invincible_table", table);
    return BT::NodeStatus::SUCCESS;
  }

private:
  invincible_detection::EnemyInvincibilityManager &enemy_hp_state_tracker_;
  perception::Subscriber &subscriber_;
};

class UpdateAimPriority : public BT::SyncActionNode
  {
  public:
    UpdateAimPriority(const std::string &name , const BT::NodeConfig &config , perception::TfAccessor &tf_accessor , perception::Subscriber &subscriber, perception::Publisher &publisher , tools::NavigationTools &navigation_tools) : SyncActionNode(name , config) , tf_accessor_(tf_accessor) , subscriber_(subscriber), publisher_(publisher) , navigation_tools_(navigation_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("chassis_mode") ,
                  BT::InputPort<std::string>("robot_color"),
                  BT::InputPort<types::EnemyInvincibleTable>("enemy_invincible_table"),
                BT::OutputPort<std::vector<uint8_t>>("aim_priority")};
    }

    BT::NodeStatus tick() override
    {
      std::vector<uint8_t> aim_priority;
      int input_chassis_mode = getInput<int>("chassis_mode").value();
      std::string robot_color = getInput<std::string>("robot_color").value();
      std::string enemy_color;
      if (robot_color == "blue") //给enemy_color赋值
      {
        enemy_color = "red";
      }else if (robot_color == "red")
      {
        enemy_color = "blue";
      }
      //----------从黑板读取上个节点发布的无敌表（UpdateEnemyInvincibleState 每周期刷新）------------
      types::EnemyInvincibleTable enemy_table = getInput<types::EnemyInvincibleTable>("enemy_invincible_table").value();
      types::ChassisMode chassis_mode = static_cast<types::ChassisMode>(input_chassis_mode);
      switch (chassis_mode)
      {
      case types::ChassisMode::GotoHitEnemyOutpostArea :
        {
          std::vector<uint8_t> src = {1,1,1,1,5,1,1,1};
          aim_priority = src;
          break;
        }
      case types::ChassisMode::GotoEnemyBase :
        {
          std::vector<uint8_t> src = {1,1,1,1,1,1,1,5};
          aim_priority = src;
          break;
        }
      case types::ChassisMode::GotoTrapezoidalHighland :
        {
          std::vector<uint8_t> src = {5,1,3,3,3,1,2,1};
          aim_priority = src;
          break;
        }
      default:
        {
          std::vector<uint8_t> src = {5, 3, 2, 2, 2, 5, 1, 0};
          aim_priority = src;
          break;
        }
      }

      //----------获取目标在map坐标系下的坐标------------
      geometry_msgs::TransformStamped camera_optical_frame2map;
      camera_optical_frame2map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP,perception::TfAccessor::FrameId::CAMERA_OPTICAL_FRAME);
      for (auto& detection : subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::FRONT_CAMERA_DETECTION_DATA).message.detections)

      {
        geometry_msgs::TransformStamped target_at_map, target_at_camera;
        target_at_camera.transform.translation.x = detection.pose.position.x;
        target_at_camera.transform.translation.y = detection.pose.position.y;
        target_at_camera.transform.translation.z = detection.pose.position.z;
        target_at_camera.header.frame_id = "camera_optical_frame";
        target_at_camera.header.stamp = ros::Time::now();
        tf2::doTransform(target_at_camera, target_at_map, camera_optical_frame2map);

        geometry_msgs::Point enemy_position;
        enemy_position.x = target_at_map.transform.translation.x;
        enemy_position.y = target_at_map.transform.translation.y;
        enemy_position.z = target_at_map.transform.translation.z;
        std::string enemy_in_area = navigation_tools_.determinePolygonInWhich(enemy_position);

        if (detection.id > 0 && detection.id <=8)
        {
          //---------目标为建筑时判断建筑是否死亡以及数据是否新鲜-----------
          const bool is_hp_fresh = ros::Time::now() - subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).stamp < ros::Duration(1.5);
          rm_msgs::GameRobotHp game_robot_hp = subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).message;
          if (detection.id == static_cast<int>(types::RobotType::OUTPOST) || detection.id == static_cast<int>(types::RobotType::BASE))
          {
            if (is_hp_fresh == false) //血量数据新鲜度不足直接返回false
            {
              aim_priority[detection.id-1] = 0;
            }
            if (detection.id == static_cast<int>(types::RobotType::OUTPOST) && game_robot_hp.enemy_outpost_hp <= 0)
            {
              aim_priority[detection.id-1] = 0;
            }
            if (detection.id == static_cast<int>(types::RobotType::BASE) && game_robot_hp.enemy_base_hp <=0)
            {
              aim_priority[detection.id-1] = 0;
            }
          }

          //----------判断目标所处的区域并动态调整优先级------------
          if (enemy_in_area == robot_color+"_fortress_area")//目标位于自家堡垒区,将优先级开到最高
          {
            aim_priority[detection.id-1] = 5; //数组下标需要将实际id减去1
          }

          if (enemy_in_area == enemy_color+"_fortress_area")//目标位于敌方堡垒区，将优先级开到最低
          {
            aim_priority[detection.id-1] = 1; //数组下标需要将实际id减去1
          }

          if (detection.id == 2 && enemy_in_area == enemy_color+"_engineer_invincible_area")//目标是工程的情况
          {
            aim_priority[detection.id-1] = 0; //工程在无敌区不打
          }

          //-----------利用黑板无敌表，覆盖之前的优先级，无敌或死亡强制不打-----------
          if (detection.id != static_cast<int>(types::RobotType::OUTPOST) && detection.id != static_cast<int>(types::RobotType::BASE))
          {
            const auto life_info = [&enemy_table](int id) -> const types::EnemyInvincibleInfo*
            {
              switch (id)
              {
                case static_cast<int>(types::RobotType::HERO):        return &enemy_table.hero;       // 1
                case static_cast<int>(types::RobotType::ENGINEER):    return &enemy_table.engineer;   // 2
                case static_cast<int>(types::RobotType::STANDARD_3):  return &enemy_table.infantry_3; // 3
                case static_cast<int>(types::RobotType::STANDARD_4):  return &enemy_table.infantry_4; // 4
                case static_cast<int>(types::RobotType::STANDARD_5):  return &enemy_table.aerial;     // 5 无人机(snapshot index 5)
                case static_cast<int>(types::RobotType::SENTRY):      return &enemy_table.sentry;     // 7 哨兵
                default: return nullptr; // 6 前哨 / 8 基地不在此表域内，由上方建筑生死逻辑负责
              }
            };
            const types::EnemyInvincibleInfo* info = life_info(detection.id);
            if (info != nullptr && info->state != types::EnemyInvincibleState::ALIVE)
            {
              aim_priority[detection.id-1] = 0;
            }
          }
        }
      }

      setOutput<std::vector<uint8_t>>("aim_priority",aim_priority);

      rm_msgs::PriorityArray priority_array;
      priority_array.rank_arr = aim_priority;
      publisher_.getPublishers()->aim_priority_pub_.publish(priority_array);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    perception::TfAccessor &tf_accessor_;
    perception::Subscriber &subscriber_;
    perception::Publisher &publisher_;
    tools::NavigationTools &navigation_tools_;
  };

class UpdatePostureState : public BT::SyncActionNode
{
public:
  UpdatePostureState(const std::string &name, const BT::NodeConfig &config,
                     posture::PostureManager &posture_manager,
                     perception::Subscriber &subscriber,
                     perception::TfAccessor &tf_accessor,
                     tools::NavigationTools &navigation_tools,
                     BT::Blackboard &blackboard)
    : SyncActionNode(name, config), posture_manager_(posture_manager), subscriber_(subscriber),
      tf_accessor_(tf_accessor), navigation_tools_(navigation_tools), blackboard_(blackboard)
  {
  }

  BT::NodeStatus tick() override
  {
    auto *ctx = posture_manager_.getPostureContext();
    if (ctx == nullptr)
      return BT::NodeStatus::FAILURE;

    const auto &gst = subscriber_.msgGetter<rm_msgs::GameStatus>(
        perception::Subscriber::TopicId::GAME_STATUS).message;
    const auto &rst = subscriber_.msgGetter<rm_msgs::GameRobotStatus>(
        perception::Subscriber::TopicId::GAME_ROBOT_STATUS).message;

    ctx->in_battle = (gst.game_progress == rm_msgs::GameStatus::IN_BATTLE);
    ctx->is_dead = (rst.remain_hp == 0);
    ctx->remain_hp = static_cast<int>(rst.remain_hp);
    ctx->max_hp = static_cast<int>(rst.max_hp);
    ctx->present_time = blackboard_.get<double>("game_total_time") - gst.stage_remain_time;
    ctx->bullet_17 = static_cast<int>(subscriber_.msgGetter<rm_msgs::BulletAllowance>(
        perception::Subscriber::TopicId::BULLET_ALLOWANCE).message.bullet_allowance_num_17_mm);
    ctx->low_hp = ctx->remain_hp < blackboard_.get<int>("trigger_blood_return_hp");

    const geometry_msgs::TransformStamped cur_in_map = tf_accessor_.getTfTransform(
        perception::TfAccessor::FrameId::MAP, perception::TfAccessor::FrameId::BASE_LINK);
    geometry_msgs::Point cur;
    cur.x = cur_in_map.transform.translation.x;
    cur.y = cur_in_map.transform.translation.y;
    cur.z = cur_in_map.transform.translation.z;
    ctx->is_in_road_area = navigation_tools_.determinePolygonInWhich(cur)
                           == blackboard_.get<std::string>("road_area_name");

    posture_manager_.update();
    return BT::NodeStatus::SUCCESS;
  }

private:
  posture::PostureManager &posture_manager_;
  perception::Subscriber &subscriber_;
  perception::TfAccessor &tf_accessor_;
  tools::NavigationTools &navigation_tools_;
  BT::Blackboard &blackboard_;
};

#endif //NEW_BEHAVIOR_TREE_ACTION_NODE_H