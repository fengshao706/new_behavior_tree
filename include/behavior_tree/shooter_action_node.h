//
// Created by fengshao on 2026/3/27.
//

#ifndef NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H

#include <behaviortree_cpp/action_node.h>
#include "common/types.h"
#include <behaviortree_cpp/blackboard.h>
#include "common/tools.h"
#include "common/behavior_base.h"

namespace shooter
{
  class SetShooterMode : public BT::SyncActionNode
  {
  public:
    SetShooterMode(const std::string &name ,const BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name , config) ,blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("shooter_mode_id") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> shooter_mode_id = getInput<int>("shooter_mode_id");
      blackboard_.set<types::ShooterMode>("shooter_mode",static_cast<types::ShooterMode>(shooter_mode_id.value()));
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Blackboard &blackboard_;
  };

  class ShooterStop : public BT::SyncActionNode
  {
  public:
    ShooterStop(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time time = ros::Time::now();
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->checkError(ros::Time::now());
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };

  class ShooterReady : public BT::SyncActionNode
  {
  public:
    ShooterReady(const std::string &name ,const BT::NodeConfig &config , BehaviorBase &behavior_base) : SyncActionNode(name , config) , behavior_base_(behavior_base)
    {

    }

    BT::NodeStatus tick() override
    {
      behavior_base_.sendShooterCmd();
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BehaviorBase &behavior_base_;
  };

  class ShooterPush : public BT::SyncActionNode
  {
  public:
    ShooterPush(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time time = ros::Time::now();
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->checkError(ros::Time::now());
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };
}

#endif //NEW_BEHAVIOR_TREE_SHOOTER_ACTION_NODE_H