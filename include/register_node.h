//
// Created by root on 2026/3/29.
//

#ifndef NEW_BEHAVIOR_TREE_REGITER_NODE_H
#define NEW_BEHAVIOR_TREE_REGITER_NODE_H

#include "behavior_tree/chassis_action_node.h"
#include "behavior_tree/chassis_condition_node.h"
#include "behavior_tree/gimbal_action_node.h"
#include "behavior_tree/gimbal_condition_node.h"
#include "behavior_tree/shooter_action_node.h"
#include "behavior_tree/shooter_condition_node.h"
#include "behavior_tree/manual_condition_node.h"
#include "behavior_tree/manual_action_node.h"
#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "perception_layer.h"
#include "common/tools.h"
#include "behavior_tree/manual_action_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "common/sentry_param_loader.h"

namespace register_node
{
  void register_node(ros::NodeHandle &bt_nh , double &wait_time , BT::Blackboard::Ptr &blackboard , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber , BehaviorBase &behavior_base , manual::SimpleAction &manual_action , BT::BehaviorTreeFactory &factory);
}

#endif //NEW_BEHAVIOR_TREE_REGITER_NODE_H