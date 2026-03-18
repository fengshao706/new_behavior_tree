//
// Created by root on 2026/3/15.
//
#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "perception_layer.h"
#include "common/tools.h"
#include "behavior_tree/manual_action_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

int main(int argc,char * argv[])
{
  ros::init(argc,argv,"rm_behavior_tree");
  ros::NodeHandle bt_nh;
  std::string file_path = bt_nh.param("xml_file_path", std::string(" "));

  CmdTools cmd_tools(bt_nh);
  perception::Subscriber subscriber(cmd_tools,bt_nh);
  manual::SimpleAction manual_action(bt_nh,cmd_tools,subscriber);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction("ManualSendChassisCmd",std::bind(&manual::SimpleAction::sendChassisCmd,&manual_action));
  factory.registerSimpleAction("ManualSendGimbalCmd",std::bind(&manual::SimpleAction::sendGimbalCmd,&manual_action));
  factory.registerSimpleAction("ManualSendShooterCmd",std::bind(&manual::SimpleAction::sendShooterCmd,&manual_action));

  BT::Tree tree = factory.createTreeFromFile(file_path);;
  BT::Groot2Publisher groot2_publisher(tree,5555);
  tree.tickWhileRunning();
}