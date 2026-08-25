//
// Created by root on 2026/3/15.
//

#ifndef NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H

#include <ros/ros.h>
#include "common/tools.h"
#include "perception_layer.h"
#include <behaviortree_cpp/action_node.h>

namespace manual
{
  class ManualSendChassisCmd : public BT::StatefulActionNode
  {
  public:
    ManualSendChassisCmd(const std::string &name , const BT::NodeConfig &config , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber , ros::NodeHandle &bt_nh) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools) , subscriber_(subscriber) , bt_nh_(bt_nh)
    {

    }

    BT::NodeStatus onStart() override
    {
      ros::NodeHandle vel_nh(bt_nh_, "vel");
      if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
        ROS_ERROR("Gyro move reduction no defined (namespace: %s)", bt_nh_.getNamespace().c_str());
      if (!vel_nh.getParam("gyro_rotate_reduction", gyro_rotate_reduction_))
        ROS_ERROR("Gyro rotate reduction no defined (namespace: %s)", bt_nh_.getNamespace().c_str());
      vel_nh.getParam("still_gyro_vel", still_gyro_vel_);

      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time time = ros::Time::now();
      bool is_gyro;

      if (std::abs(subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.wheel) > 0.01)
      {
        cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        is_gyro = true;
      }
      else
      {
        cmd_tools_.getSenders()->chassis_command_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
        is_gyro = false;
      }
      cmd_tools_.getSenders()->vel_2d_command_sender_->setAngularZVel(
        (std::abs(subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_y) > 0.01 || std::abs(subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_x) > 0.01)
          ? subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.wheel * gyro_rotate_reduction_
          : subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.wheel * still_gyro_vel_);
      cmd_tools_.getSenders()->vel_2d_command_sender_->setLinearXVel(is_gyro
                                                     ? subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_y * gyro_move_reduction_
                                                     : subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_y);
      cmd_tools_.getSenders()->vel_2d_command_sender_->setLinearYVel(is_gyro
                                                     ? -subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_x * gyro_move_reduction_
                                                     : -subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_r_x);
      cmd_tools_.getSenders()->chassis_command_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
      cmd_tools_.getSenders()->chassis_command_sender_->getMsg()->command_source_frame = "yaw";
      cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(time, is_gyro);
      cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(time);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->chassis_command_sender_->setZero();
      cmd_tools_.getSenders()->vel_2d_command_sender_->setZero();
    }

  private:
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
    ros::NodeHandle &bt_nh_;
    double gyro_move_reduction_{};
    double gyro_rotate_reduction_{};
    double still_gyro_vel_{};
  };

  class ManualSendGimbalCmd : public BT::StatefulActionNode
  {
  public:
    ManualSendGimbalCmd(const std::string &name , const BT::NodeConfig &config , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools), subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time time = ros::Time::now();
      if (subscriber_.msgGetter<rm_msgs::TrackData>(perception::Subscriber::TopicId::TRACK_DATA).message.id == 0 || subscriber_.msgGetter<rm_msgs::ShootCmd>(perception::Subscriber::TopicId::SHOOT_CMD).message.mode == 0)
      {
        cmd_tools_.setStackGimbalMode(rm_msgs::GimbalCmd::RATE);
        cmd_tools_.setStackGimbalRate(-subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_l_x, -subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_l_x, -subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.ch_l_y);
        if (cmd_tools_.getSenders()->gimbal_command_sender_->getMsg()->mode == rm_msgs::GimbalCmd::RATE)
          cmd_tools_.getSenders()->chassis_command_sender_->setFollowVelDes(
            cmd_tools_.getSenders()->gimbal_command_sender_->getMsg()->rate_yaw);
        else
          cmd_tools_.getSenders()->chassis_command_sender_->setFollowVelDes(0);
        if (cmd_tools_.getSenders()->base_gimbal_command_sender_->getMsg()->mode == rm_msgs::GimbalCmd::RATE)
          cmd_tools_.getSenders()->chassis_command_sender_->setFollowVelDes(
            cmd_tools_.getSenders()->base_gimbal_command_sender_->getMsg()->rate_yaw);
        else
          cmd_tools_.getSenders()->chassis_command_sender_->setFollowVelDes(0);
      }
      else
      {
        cmd_tools_.getSenders()->gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
        cmd_tools_.getSenders()->gimbal_command_sender_->setBulletSpeed(
          cmd_tools_.getSenders()->shooter_command_sender_->getSpeed());
      }
      cmd_tools_.sendStackGimbalCommand(time);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->base_gimbal_command_sender_->setZero();
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }

  private:
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
  };

  class ManualSendShooterCmd : public BT::StatefulActionNode
  {
  public:
    ManualSendShooterCmd(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools, perception::Subscriber &subscriber) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      ros::Time now_time = ros::Time::now();
      if (subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.s_l == rm_msgs::DbusData::UP)
      {
        if (now_time.toSec() - last_time_.toSec() > 0.00001 || continue_shoot_)
        {
          if (one_shoot_)
            continue_shoot_ = true;
          last_time_ = now_time;
          one_shoot_ = true;
          cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        }
        else
          cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
      }
      else
      {
        continue_shoot_ = false;
        one_shoot_ = false;
        if (subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).message.s_l == rm_msgs::DbusData::MID)
          cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
        else
          cmd_tools_.getSenders()->shooter_command_sender_->setMode(rm_msgs::ShootCmd::STOP);
      }

      cmd_tools_.getSenders()->shooter_command_sender_->checkError(ros::Time::now());
      cmd_tools_.getSenders()->shooter_command_sender_->sendCommand(now_time);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->shooter_command_sender_->setZero();
    }

  private:
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
    ros::Time last_time_{};
    bool one_shoot_{false};
    bool continue_shoot_{false};
  };
}

#endif //NEW_BEHAVIOR_TREE_MANUAL_ACTION_NODE_H