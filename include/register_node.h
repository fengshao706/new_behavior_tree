//
// Created by root on 2026/3/29.
//

#ifndef NEW_BEHAVIOR_TREE_REGITER_NODE_H
#define NEW_BEHAVIOR_TREE_REGITER_NODE_H

#include "behavior_tree/chassis_action_node.h"
#include "behavior_tree/gimbal_action_node.h"
#include "behavior_tree/shooter_action_node.h"
#include "behavior_tree/manual_action_node.h"
#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "perception_layer.h"
#include "common/tools.h"
#include "behavior_tree/manual_action_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "common/sentry_param_loader.h"
#include "behavior_tree/common/action_node.h"
#include "behavior_tree/condition_node.h"
#include "common/tools.h"

namespace register_node
{
  void register_node(ros::NodeHandle &bt_nh , tools::CmdTools& cmd_tools, perception::Subscriber& subscriber, BT::BehaviorTreeFactory& factory,
                     tools::NavigationTools& navigation_tools, tools::MiniMapTools& mini_map_tools,
                     tools::ControllerTools& controller_tools, tools::GimbalTools& gimbal_tools,tools::PlannerTools &planner_tools,
                     perception::TfAccessor& tf_accessor , perception::Publisher &publisher);
}

#endif //NEW_BEHAVIOR_TREE_REGITER_NODE_H