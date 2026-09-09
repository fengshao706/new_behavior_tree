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
#include "common/invincible_detection.h"
#include "common/dx_track_switch_caller.h"
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
  ROS_INFO("---------------ros init----------------");
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
  invincible_detection::EnemyInvincibilityManager enemy_invincibility_manager(navigation_tools , mini_map_tools , tf_accessor , subscriber , blackboard->get<std::string>("robot_color"));
  auto_aim::DxTrackSwitchCaller dx_track_switch_caller(bt_nh);
  tools::AutoAimTools auto_aim_tools(bt_nh , cmd_tools , dx_track_switch_caller);
  ROS_INFO("------------------complete------------------------");
  BT::BehaviorTreeFactory factory;

  register_node::register_node(bt_nh , cmd_tools , subscriber , factory , navigation_tools , mini_map_tools , controller_tools , gimbal_tools ,planner_tools, tf_accessor,publisher,enemy_invincibility_manager , dx_track_switch_caller , auto_aim_tools , posture_manager);

  std::filesystem::path root_path(PROJECT_ROOT_DIR);

  std::filesystem::path xml_path = root_path / "config" / "test.xml";
  std::filesystem::path log_path = root_path / "log" / get_current_time_string().append(".btlog");

  std::cout << "Loading XML from: " << xml_path << std::endl;
  std::cout << "Saving Log to: " << log_path << std::endl;

  ROS_INFO("Loading XML from: %s" , xml_path.c_str());
  ROS_INFO("Saving Log to: %s" , log_path.c_str());

  BT::Tree tree = factory.createTreeFromFile(xml_path,blackboard);
  BT::ReactiveSequence::EnableException(false);

  BT::FileLogger2 logger2(tree,log_path);

  BT::Groot2Publisher groot2_publisher(tree,5555);
  ros::Rate rate(500);
  ros::Publisher shinji_result_pub_ =
      bt_nh.advertise<geometry_msgs::TransformStamped>("/shinji/result", 1, false);
  ros::Time last_initialpose_stamp = subscriber.msgGetter<geometry_msgs::PoseWithCovarianceStamped>(
      perception::Subscriber::TopicId::RVIZ_2D_POSE).stamp;  // 启动时播种，避免启动即误发
  int test = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickExactlyOnce();
    controller_tools.ControllerUpdate();

    const auto initial_pose = subscriber.msgGetter<geometry_msgs::PoseWithCovarianceStamped>(
        perception::Subscriber::TopicId::RVIZ_2D_POSE);
    if (initial_pose.stamp != last_initialpose_stamp)  // 有新点击，时间戳更新
    {
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = initial_pose.stamp;
      msg.header.frame_id = "map";
      msg.child_frame_id = "camera_init";  // 与 shinji 输出一致
      const auto& p = initial_pose.message.pose.pose;
      msg.transform.translation.x = p.position.x;
      msg.transform.translation.y = p.position.y;
      msg.transform.translation.z = p.position.z;
      msg.transform.rotation = p.orientation;
      shinji_result_pub_.publish(msg);
      last_initialpose_stamp = initial_pose.stamp;
      ROS_INFO_STREAM_THROTTLE(1.0, "RvizPoseCorrection: map->camera_init = ("
        << p.position.x << ", " << p.position.y << ")");
    }

    test += 1;
    if (test >= 500)
    {
      ROS_INFO("---------complete a circle------------");
      test = 0;
    }
    rate.sleep();
  }
}