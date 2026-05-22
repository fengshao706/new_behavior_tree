//
// Created by fengshao on 2026/3/27.
//

#ifndef NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H

#include <behaviortree_cpp/action_node.h>
#include "common/types.h"
#include <behaviortree_cpp/blackboard.h>
#include "common/tools.h"

namespace shooter
{
  class SetShooterMode : public BT::SyncActionNode
  {
  public:
    SetShooterMode(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name , config)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("shooter_mode_id_input"),
      BT::OutputPort<int>("shooter_mode_id_output")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> shooter_mode_id = getInput<int>("shooter_mode_id_input");
      setOutput("shooter_mode_id_output",shooter_mode_id.value());
      return BT::NodeStatus::SUCCESS;
    }

  private:

  };

  class ShooterStop : public BT::StatefulActionNode
  {
  public:
    ShooterStop(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time time = ros::Time::now();
      cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::STOP);
      cmd_tools_.getSenders()->shooter_command_sender_->checkError(ros::Time::now());
      cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(time);
      return BT::NodeStatus::RUNNING;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };

  class ShooterReady : public BT::StatefulActionNode
  {
  public:
    ShooterReady(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
      cmd_tools_.getSenders()->shooter_command_sender_->checkError(ros::Time::now());
      cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(ros::Time::now());
      return BT::NodeStatus::RUNNING;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };

  class ShooterPush : public BT::StatefulActionNode
  {
  public:
    ShooterPush(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time time = ros::Time::now();
      cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      cmd_tools_.getSenders()->shooter_command_sender_->checkError(ros::Time::now());
      cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(time);
      return BT::NodeStatus::RUNNING;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };
}

#endif //NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H