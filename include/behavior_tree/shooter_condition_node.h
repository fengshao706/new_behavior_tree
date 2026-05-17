//
// Created by rootfe on 2026/3/27.
//

#ifndef NEW_BEHAVIOR_TREE_SHOOTER_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_SHOOTER_CONDITION_NODE_H

#include <behaviortree_cpp/action_node.h>
#include "common/types.h"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include "perception_layer.h"

namespace shooter
{
  class IsTargetNotInvincible : public BT::ConditionNode  //TODO : 需重写无敌检测算法
  {
  public:
    IsTargetNotInvincible(const std::string &name ,const BT::NodeConfig &config) : ConditionNode(name , config)
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
    IsTargetEffective(const std::string &name ,const BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name , config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      return subscriber_.track_data_.id != 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

  private:
    perception::Subscriber &subscriber_;
  };
}


#endif //NEW_BEHAVIOR_TREE_SHOOTER_CONDITION_NODE_H