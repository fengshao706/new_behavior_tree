//
// Created by luotinkai on 2/3/23.
//

#pragma once

#include "union_command_sender.h"
#include "perception_layer.h"
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <unordered_map>
#include <bitset>
#include <angles/angles.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/ori_tool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/keepOutZone.h>
#include "rm_common/decision/controller_manager.h"
#include "rm_common/decision/calibration_queue.h"

class BehaviorBase
{
public:
  BehaviorBase(ros::NodeHandle& nh, tools::CmdTools& cmd_tools, perception::Subscriber & subscriber , BT::Blackboard &blackboard)
    : cmd_tools_(cmd_tools), subscriber_(subscriber),tf_listener_(tf_buffer_) ,controller_manager_(nh), blackboard_(blackboard)
  {
    ros::NodeHandle auto_nh(nh, "auto");
    auto_nh.getParam("standby_velocity", standby_velocity_);
    auto_nh.getParam("invincible_check", invincible_check_);
    ros::NodeHandle yaw_nh(nh, "yaw");
    yaw_nh.getParam("gimbal_vel_coeff", gimbal_vel_coeff_);
    ros::NodeHandle pitch_nh(nh, "pitch");
    pitch_nh.getParam("pitch_vel_coeff", pitch_vel_coeff_);
  }

  void getDoubleTypeParams(const XmlRpc::XmlRpcValue& double_type_params,
                           std::unordered_map<std::string, double>& behavior_double_type_params)
  {
    for (const auto& double_type_param : double_type_params)
    {
      ROS_ASSERT(double_type_param.second.hasMember("param"));
      ROS_ASSERT(double_type_param.second["param"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      behavior_double_type_params.insert(
          std::make_pair(double_type_param.first, static_cast<double>(double_type_param.second["param"])));
    }
  }

  virtual void sendChassisCmd() //依赖于command_sender
  {
    cmd_tools_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);//原始模式，表示底盘不再跟随云台，直接使用我发出的速度指令
    cmd_tools_.chassis_cmd_sender_->getMsg()->command_source_frame = "base_link"; //意味着我给出的速度是相对于底盘中心的
  }

  virtual void cancelGoal()
  {
    if (cmd_tools_.mbf_client_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE ||
        cmd_tools_.mbf_client_->getState().state_ == actionlib::SimpleClientGoalState::PENDING)
    {
      ROS_INFO("cancel mbf goal");
      cmd_tools_.mbf_client_->cancelGoal();
    }
  }

  void setGimbalDirectPoint(geometry_msgs::PointStamped point_of_map)
  {
    geometry_msgs::PointStamped point_of_odom;
    ros::Time time = ros::Time::now();
    geometry_msgs::TransformStamped odom2map;
    try
    {
      odom2map = tf_buffer_.lookupTransform("odom", "map", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    tf2::doTransform(point_of_map, point_of_odom, odom2map);
    cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
    cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setPoint(point_of_odom);
    // TODO:need update
    cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  }

  void setGimbalRate()
  {
    ros::Time time = ros::Time::now();
    cmd_tools_.setStackGimbalMode(rm_msgs::GimbalCmd::RATE);
    cmd_tools_.setStackGimbalRate(cmd_tools_.union_cmd_sender_->yaw_direct_, cmd_tools_.union_cmd_sender_->yaw_direct_,
                                  cmd_tools_.union_cmd_sender_->pitch_direct_);
    cmd_tools_.sendStackGimbalCommand(time);
  }

  virtual void sendShooterCmd()
  {
    ros::Time time = ros::Time::now();
    cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
  }

  void setGryoInCombat()
  {
    ros::Time time = ros::Time::now();
    if (auto_control_info_.own_outpost_hp_ <= 0)
      cmd_tools_.vel_2d_cmd_sender_->set2DVel(0.0, 0.0,
                                              static_cast<double>(standby_velocity_[0]) *
                                                      std::sin(ros::Time::now().toSec()) +
                                                  static_cast<double>(standby_velocity_[1]));
    else
      cmd_tools_.vel_2d_cmd_sender_->set2DVel(0.0, 0.0, 0.0);
    cmd_tools_.chassis_cmd_sender_->sendChassisCommand(time, true);
    cmd_tools_.vel_2d_cmd_sender_->sendCommand(time);
  }

  void stopAllMotion()
  {
    cmd_tools_.vel_2d_cmd_sender_->setZero();
    cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setZero();
    cmd_tools_.union_cmd_sender_->base_gimbal_cmd_sender_->setZero();
    cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setZero();
  }

  bool timeoutJudgement(ros::Time begin, double duration)
  {
    return (ros::Time::now() - begin).toSec() > duration;
  }

  void lidarTwist(double yaw_vel)
  {
    try
    {
      map2yaw_ = tf_buffer_.lookupTransform("map", "yaw", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    double yaw = yawFromQuat(map2yaw_.transform.rotation);

    if (circle_count_ <= 0)
      cmd_tools_.union_cmd_sender_->yaw_direct_ = cmd_tools_.smoothlyYawOutput(yaw_vel);
    if (circle_count_ > cmd_tools_.union_cmd_sender_->circle_)
      cmd_tools_.union_cmd_sender_->yaw_direct_ = cmd_tools_.smoothlyYawOutput(-yaw_vel);

    if (cmd_tools_.union_cmd_sender_->yaw_direct_ == yaw_vel || cmd_tools_.union_cmd_sender_->yaw_direct_ == -yaw_vel)
    {
      if (yaw - last_yaw_ > M_PI)
      {
        circle_count_--;
      }
      else if (yaw - last_yaw_ < -M_PI)
      {
        circle_count_++;
      }
      last_yaw_ = yaw;
    }
  }

  void yawStrafe(double left_angle, double right_angle, double angle_interval)
  {
    double reverse_interval = (2 * M_PI - angle_interval) / 2;
    try
    {
      map2yaw_ = tf_buffer_.lookupTransform("map", "yaw", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    double current_yaw = yawFromQuat(map2yaw_.transform.rotation);
//    ROS_INFO_STREAM("left_angle:" << left_angle << "right_angle:" << right_angle << "current_yaw:" << current_yaw);
    if (angles::shortest_angular_distance(current_yaw, left_angle) <= reverse_interval &&
        angles::shortest_angular_distance(current_yaw, left_angle) > 0)
      cmd_tools_.union_cmd_sender_->yaw_direct_ =
          cmd_tools_.smoothlyYawOutput(static_cast<double>(gimbal_vel_coeff_[0]));
    else if (angles::shortest_angular_distance(current_yaw, right_angle) >= -reverse_interval &&
             angles::shortest_angular_distance(current_yaw, right_angle) < 0)
      cmd_tools_.union_cmd_sender_->yaw_direct_ =
          cmd_tools_.smoothlyYawOutput(-static_cast<double>(gimbal_vel_coeff_[0]));
  }

  void pitchStrafe(double min_angel, double max_angle, double pitch_outside_vel, double pitch_inside_vel,
                   double breach_threshold)
  {
    try
    {
      yaw2pitch_ = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    double roll_temp, pitch, yaw_temp;
    quatToRPY(yaw2pitch_.transform.rotation, roll_temp, pitch, yaw_temp);

    if (pitch >= max_angle)
    {
      cmd_tools_.union_cmd_sender_->pitch_direct_ = -pitch_inside_vel;
      if (pitch - max_angle >= breach_threshold)
        cmd_tools_.union_cmd_sender_->pitch_direct_ = -pitch_outside_vel;
    }
    else if (pitch <= min_angel)
    {
      cmd_tools_.union_cmd_sender_->pitch_direct_ = pitch_inside_vel;
      if (pitch - min_angel <= -breach_threshold)
        cmd_tools_.union_cmd_sender_->pitch_direct_ = pitch_outside_vel;
    }
  }

  void conduct(const geometry_msgs::PoseStamped& point)  // TODO : 需确保在到达之前不要重复tick
  {
    mbf_goal_.target_pose = point;
    mbf_goal_.direct_track = false;
    cmd_tools_.mbf_client_->sendGoal(mbf_goal_); //发送目标位置
    // TODO : 需要在行为树中对是否到达做判断
    ROS_INFO_STREAM("Present target point is: " << mbf_goal_.target_pose.pose.position.x << ","
      << mbf_goal_.target_pose.pose.position.y);
  }

  void calibrate()
  {
    try
    {
      ros::NodeHandle nh;
      XmlRpc::XmlRpcValue gimbal_calibration, shooter_calibration, barrel_calibration;
      nh.getParam("gimbal_calibration", gimbal_calibration);
      gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_calibration, nh, controller_manager_);
      nh.getParam("shooter_calibration", shooter_calibration);
      shooter_calibration_ = new rm_common::CalibrationQueue(shooter_calibration, nh, controller_manager_);
      nh.getParam("barrel_calibration", barrel_calibration);
      barrel_calibration_ = new rm_common::CalibrationQueue(barrel_calibration, nh, controller_manager_);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
    //  ros::Time time = ros::Time::now();
    gimbal_calibration_->reset();
    shooter_calibration_->reset();
    bool has_calibrated_barrel = blackboard_.get<bool>("has_calibrated_barrel");
    if (!has_calibrated_barrel)
    {
      barrel_calibration_->reset();
      has_calibrated_barrel = true;
      blackboard_.set<bool>("has_calibrated_barrel",has_calibrated_barrel);
    }
    else
    {
      barrel_calibration_->stopController();
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->init();
    }
    //  cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    //  cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
  }


  ros::Subscriber vel_sub_;
  tools::CmdTools& cmd_tools_;
  perception::Subscriber & subscriber_;
  XmlRpc::XmlRpcValue standby_velocity_, gimbal_vel_coeff_, invincible_check_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double target_yaw_, pitch_vel_coeff_;

  geometry_msgs::TransformStamped map2yaw_{};
  geometry_msgs::TransformStamped yaw2pitch_{};
  geometry_msgs::TransformStamped map2base_{};

  double last_yaw_{};
  int circle_count_{};
  mbf_msgs::MoveBaseGoal mbf_goal_;
  rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{}, *barrel_calibration_{};
  rm_common::ControllerManager controller_manager_;
  BT::Blackboard &blackboard_;
};