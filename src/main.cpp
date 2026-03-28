//
// Created by root on 2026/3/15.
//
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
  std::string file_path = bt_nh.param("xml_file_path", std::string(" "));
  auto blackboard = BT::Blackboard::create();
  SentryParamLoader sentry_param_loader(bt_nh,blackboard);
  tools::CmdTools cmd_tools(bt_nh);
  perception::Subscriber subscriber(cmd_tools,bt_nh,*blackboard);
  BehaviorBase behavior_base(bt_nh,cmd_tools,subscriber,*blackboard);

  manual::SimpleAction manual_action(bt_nh,cmd_tools,subscriber);

  BT::BehaviorTreeFactory factory;



  BT::Tree tree = factory.createTreeFromFile(file_path,blackboard);;
  BT::Groot2Publisher groot2_publisher(tree,5555);
  tree.tickWhileRunning();
}