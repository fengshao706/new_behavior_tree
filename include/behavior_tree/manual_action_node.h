//
// Created by root on 2026/3/15.
//

#ifndef NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H

#include "ros/ros.h"
#include "common/tools.h"
#include "perception_layer.h"
#include "behaviortree_cpp/action_node.h"

namespace manual
{
  class SimpleAction
  {
  public:
    explicit SimpleAction(ros::NodeHandle& nh, CmdTools& cmd_tools, perception::Subscriber& subscriber)
      : cmd_tools_(cmd_tools), subscriber_(subscriber)
    {
      ros::NodeHandle vel_nh(nh, "vel");
      if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
        ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
      if (!vel_nh.getParam("gyro_rotate_reduction", gyro_rotate_reduction_))
        ROS_ERROR("Gyro rotate reduction no defined (namespace: %s)", nh.getNamespace().c_str());
      vel_nh.getParam("still_gyro_vel", still_gyro_vel_);
    }

    BT::NodeStatus sendChassisCmd()
    {
      ros::Time time = ros::Time::now();
      bool is_gyro;

      if (std::abs(subscriber_.dbus_.wheel) > 0.01)
      {
        cmd_tools_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        is_gyro = true;
      }
      else
      {
        cmd_tools_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
        is_gyro = false;
      }
      cmd_tools_.vel_2d_cmd_sender_->setAngularZVel(
        (std::abs(subscriber_.dbus_.ch_r_y) > 0.01 || std::abs(subscriber_.dbus_.ch_r_x) > 0.01)
          ? subscriber_.dbus_.wheel * gyro_rotate_reduction_
          : subscriber_.dbus_.wheel * still_gyro_vel_);
      cmd_tools_.vel_2d_cmd_sender_->setLinearXVel(is_gyro
                                                     ? subscriber_.dbus_.ch_r_y * gyro_move_reduction_
                                                     : subscriber_.dbus_.ch_r_y);
      cmd_tools_.vel_2d_cmd_sender_->setLinearYVel(is_gyro
                                                     ? -subscriber_.dbus_.ch_r_x * gyro_move_reduction_
                                                     : -subscriber_.dbus_.ch_r_x);
      cmd_tools_.chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
      cmd_tools_.chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
      cmd_tools_.chassis_cmd_sender_->sendChassisCommand(time, is_gyro);
      cmd_tools_.vel_2d_cmd_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus sendGimbalCmd()
    {
      ros::Time time = ros::Time::now();
      if (subscriber_.track_data_.id == 0 || subscriber_.shoot_cmd_.mode == 0)
      {
        cmd_tools_.setStackGimbalMode(rm_msgs::GimbalCmd::RATE);
        cmd_tools_.setStackGimbalRate(-subscriber_.dbus_.ch_l_x, -subscriber_.dbus_.ch_l_x, -subscriber_.dbus_.ch_l_y);
        if (cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::RATE)
          cmd_tools_.chassis_cmd_sender_->setFollowVelDes(
            cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->getMsg()->rate_yaw);
        else
          cmd_tools_.chassis_cmd_sender_->setFollowVelDes(0);
        if (cmd_tools_.union_cmd_sender_->base_gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::RATE)
          cmd_tools_.chassis_cmd_sender_->setFollowVelDes(
            cmd_tools_.union_cmd_sender_->base_gimbal_cmd_sender_->getMsg()->rate_yaw);
        else
          cmd_tools_.chassis_cmd_sender_->setFollowVelDes(0);
      }
      else
      {
        cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
        cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->getSpeed());
      }
      cmd_tools_.sendStackGimbalCommand(time);
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus sendShooterCmd()
    {
      ros::Time now_time = ros::Time::now();
      if (subscriber_.dbus_.s_l == rm_msgs::DbusData::UP)
      {
        if (now_time.toSec() - last_time_.toSec() > 0.00001 || continue_shoot_)
        {
          if (one_shoot_)
            continue_shoot_ = true;
          last_time_ = now_time;
          one_shoot_ = true;
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        }
        else
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
      }
      else
      {
        continue_shoot_ = false;
        one_shoot_ = false;
        if (subscriber_.dbus_.s_l == rm_msgs::DbusData::MID)
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
        else
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
      }

      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->checkError(ros::Time::now());
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(now_time);
    return BT::NodeStatus::SUCCESS;
    }

    //  void setZero()
    //  {
    //    cmd_tools_.vel_2d_cmd_sender_->setZero();
    //    cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setZero();
    //  }

    int getBuffState(uint32_t value, int bit_idx)
    {
      std::string binary = std::bitset < 32 > (value).to_string();
      ROS_INFO_STREAM("STRING:" << binary);
      return binary[bit_idx] - '0';
    }

  private:
    ros::Time last_time_{};
    bool one_shoot_{false};
    bool continue_shoot_{false};
    CmdTools& cmd_tools_;
    perception::Subscriber& subscriber_;
    double gyro_move_reduction_{1.};
    double gyro_rotate_reduction_{1.};
    double still_gyro_vel_{1.};
  };
}

#endif //NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H