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
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <chrono>

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR ""
#endif

std::string get_current_time_string() {
  const auto now = std::chrono::system_clock::now();
  const auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

int main(int argc,char * argv[])
{
  ros::init(argc,argv,"rm_behavior_tree");
  ros::NodeHandle nh;
  ros::NodeHandle bt_nh(nh,"rm_behavior_tree");

  double wait_time = 3.0;
  auto blackboard = BT::Blackboard::create();
  SentryParamLoader sentry_param_loader(bt_nh,blackboard);

  perception::Publisher publisher(bt_nh);
  tools::CmdTools cmd_tools(bt_nh , *blackboard);
  tools::PlannerTools planner_tools(bt_nh);
  ROS_INFO("---------------------TEST-----------------------");
  tools::ControllerTools controller_tools(bt_nh);

  perception::Subscriber subscriber(bt_nh);
  tools::MiniMapTools mini_map_tools(*blackboard , publisher , subscriber);
  perception::TfAccessor tf_accessor(bt_nh,subscriber);
  tools::GimbalTools gimbal_tools(tf_accessor,cmd_tools,bt_nh);
  tools::NavigationTools navigation_tools(*blackboard ,subscriber,tf_accessor,cmd_tools,planner_tools);

  posture::PostureManager posture_manager(bt_nh,*blackboard,publisher);
  ROS_INFO("------------------complete------------------------");
  BT::BehaviorTreeFactory factory;

  register_node::register_node(bt_nh , cmd_tools , subscriber , factory , navigation_tools , mini_map_tools , controller_tools , gimbal_tools ,planner_tools, tf_accessor,publisher);

  std::filesystem::path root_path(PROJECT_ROOT_DIR);

  std::filesystem::path xml_path = root_path / "config" / "untitled_1.xml";
  std::filesystem::path log_path = root_path / "log" / get_current_time_string().append(".btlog");

  std::cout << "Loading XML from: " << xml_path << std::endl;
  std::cout << "Saving Log to: " << log_path << std::endl;

  ROS_INFO("Loading XML from: %s" , xml_path.c_str());
  ROS_INFO("Saving Log to: %s" , log_path.c_str());

  BT::Tree tree = factory.createTreeFromFile(xml_path,blackboard);
  BT::ReactiveSequence::EnableException(false);

  BT::FileLogger2 logger2(tree,log_path);

  BT::Groot2Publisher groot2_publisher(tree,5555);
  ros::Rate rate(2000);
  int test = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickExactlyOnce();
    controller_tools.ControllerUpdate();
    posture_manager.update();
    test += 1;
    if (test >= 2000)
    {
      ROS_INFO("---------complete a circle------------");
      test = 0;
    }
    rate.sleep();
  }
}