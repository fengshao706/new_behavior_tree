//
// Created by root on 2026/3/15.
//
#include <register_node.h>
#include <behavior_tree/chassis_action_node.h>
#include "behavior_tree/gimbal_action_node.h"
#include "behavior_tree/shooter_action_node.h"

#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "perception_layer.h"
#include "common/tools.h"
#include "behavior_tree/manual_action_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "common/sentry_param_loader.h"
#include "common/posture_manager.h"

int main(int argc,char * argv[])
{
  ros::init(argc,argv,"rm_behavior_tree");
  ros::NodeHandle nh;
  ros::NodeHandle bt_nh(nh,"rm_behavior_tree");

  double wait_time = 3.0;
  std::string file_path = bt_nh.param("xml_file_path", std::string(" "));
  auto blackboard = BT::Blackboard::create();
  SentryParamLoader sentry_param_loader(bt_nh,blackboard);

  perception::Publisher publisher(bt_nh);
  tools::CmdTools cmd_tools(bt_nh , *blackboard);
  tools::PlannerTools planner_tools(bt_nh);
  ROS_INFO("---------------------TEST-----------------------");
  tools::ControllerTools controller_tools(bt_nh);

  perception::Subscriber subscriber(cmd_tools,bt_nh);
  tools::MiniMapTools mini_map_tools(*blackboard , publisher , subscriber);
  perception::TfAccessor tf_accessor(bt_nh,subscriber);
  tools::GimbalTools gimbal_tools(tf_accessor,cmd_tools,bt_nh);
  tools::NavigationTools navigation_tools(*blackboard ,subscriber,tf_accessor,cmd_tools,planner_tools);
  subscriber.setNavigationTools(&navigation_tools); //TODO : 需要优化实现

  manual::SimpleAction manual_action(bt_nh,cmd_tools,subscriber);
  posture::PostureManager posture_manager(bt_nh,*blackboard,publisher);
  ROS_INFO("------------------complete------------------------");
  BT::BehaviorTreeFactory factory;

  register_node::register_node(bt_nh , cmd_tools , subscriber , factory , navigation_tools , mini_map_tools , controller_tools , gimbal_tools ,planner_tools, tf_accessor,publisher);

  BT::Tree tree = factory.createTreeFromFile("/home/wjr/rmuc_ws/src/rm_sentry/decision/new_behavior_tree/config/untitled_1.xml",blackboard);
  BT::Groot2Publisher groot2_publisher(tree,5555);
  ros::Rate rate(200);
  int test = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickExactlyOnce();
    controller_tools.ControllerUpdate();
    posture_manager.update();
    test += 1;
    if (test >= 200)
    {
      ROS_INFO("---------complete a circle------------");
      test = 0;
    }
    rate.sleep();
  }
}