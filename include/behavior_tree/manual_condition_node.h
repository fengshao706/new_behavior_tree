//
// Created by root on 2026/3/29.
//

#ifndef NEW_BEHAVIOR_TREE_MANUAL_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_MANUAL_CONDITION_NODE_H

#include "behaviortree_cpp/condition_node.h"
#include <ros/ros.h>
#include "perception_layer.h"
#include "common/behavior_base.h"

namespace manual
{
  class IsRemoteControlTurnOn : public BT::ConditionNode
  {
  public:
    IsRemoteControlTurnOn(const std::string &name , const BT::NodeConfig &config , perception::Subscriber &subscriber , BehaviorBase &behavior_base) : ConditionNode(name , config) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      if (ros::Time::now() - subscriber_.dbus_.stamp < ros::Duration(0.3))
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
}

#endif //NEW_BEHAVIOR_TREE_MANUAL_CONDITION_NODE_H