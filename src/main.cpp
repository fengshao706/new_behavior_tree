//
// Created by root on 2026/3/15.
//
#include <register_node.h>
#include <behavior_tree/chassis_action_node.h>
#include "behavior_tree/chassis_condition_node.h"
#include "behavior_tree/gimbal_action_node.h"
#include "behavior_tree/gimbal_condition_node.h"
#include "behavior_tree/shooter_action_node.h"
#include "behavior_tree/shooter_condition_node.h"

#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "perception_layer.h"
#include "common/tools.h"
#include "behavior_tree/manual_action_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "common/sentry_param_loader.h"

int main(int argc,char * argv[])
{
  ros::init(argc,argv,"rm_behavior_tree");
  ros::NodeHandle bt_nh;
  ros::NodeHandle behavior_tree_nh(bt_nh,"rm_behavior_tree");

  double wait_time = 3.0;
  std::string file_path = bt_nh.param("xml_file_path", std::string(" "));
  auto blackboard = BT::Blackboard::create();
  SentryParamLoader sentry_param_loader(bt_nh,blackboard);

  tools::CmdTools cmd_tools(behavior_tree_nh);
  perception::Subscriber subscriber(cmd_tools,bt_nh,*blackboard);
  BehaviorBase behavior_base(behavior_tree_nh,cmd_tools,subscriber,*blackboard);

  manual::SimpleAction manual_action(behavior_tree_nh,cmd_tools,subscriber);
  ROS_INFO("------------------complete------------------------");
  BT::BehaviorTreeFactory factory;

  register_node::register_node(bt_nh,wait_time,blackboard,cmd_tools,subscriber,behavior_base,manual_action,factory);

  BT::Tree tree = factory.createTreeFromFile("/home/wjr/2026_rm_ws/src/rm_sentry/decision/new_behavior_tree/config/new_tree.xml",blackboard);
  BT::Groot2Publisher groot2_publisher(tree,5555);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Rate rate(50);
  int test = 0;
  while (ros::ok())
  {
    tree.tickExactlyOnce();
    test += 1;
    if (test >= 50)
    {
      ROS_INFO("---------complete a circle------------");
      test = 0;
    }
    rate.sleep();
  }
  spinner.stop();
}