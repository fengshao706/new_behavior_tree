//
// Created by luotinkai on 2022/7/10.
//
#ifndef NEW_BEHAVIOR_TREE_UNION_COMMAND_SENDER_H
#define NEW_BEHAVIOR_TREE_UNION_COMMAND_SENDER_H

#include "rm_common/decision/command_sender.h"
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>

using namespace rm_common;

class UnionCommandSender
{
public:
    UnionCommandSender(ros::NodeHandle& nh)
    {
        ros::NodeHandle gimbal_nh(nh, "gimbal");
        ros::NodeHandle base_gimbal_nh(nh, "base_gimbal");
        gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh);
        base_gimbal_cmd_sender_ = new GimbalCommandSender(base_gimbal_nh);
        ros::NodeHandle switcher_nh(nh, "switcher");
        //    ros::NodeHandle shooter_ID1_nh(switcher_nh, "shooter_ID1");
        //    double_barrel_cmd_sender_ = new ShooterCommandSender(shooter_ID1_nh);
        double_barrel_cmd_sender_ = new DoubleBarrelCommandSender(switcher_nh);
        ros::NodeHandle auto_nh(nh, "auto");
        try
        {
            auto_nh.getParam("pitch", pitch_value_);
            pitch_min_ = static_cast<double>(pitch_value_[0]);
            pitch_max_ = static_cast<double>(pitch_value_[1]);
            pitch_inside_vel_ = static_cast<double>(pitch_value_[2]);
            pitch_outside_vel_ = static_cast<double>(pitch_value_[3]);

            auto_nh.getParam("circle", circle_);
            auto_nh.getParam("breach_threshold", breach_threshold_);
        }
        catch (XmlRpc::XmlRpcException& e)
        {
            ROS_ERROR("%s", e.getMessage().c_str());
        }
    };
    GimbalCommandSender* gimbal_cmd_sender_;
    GimbalCommandSender* base_gimbal_cmd_sender_;
    //  ShooterCommandSender* double_barrel_cmd_sender_{};
    DoubleBarrelCommandSender* double_barrel_cmd_sender_{};
    double pitch_min_{}, pitch_max_{}, pitch_inside_vel_{}, pitch_outside_vel_{};
    int circle_{ 1 };
    double breach_threshold_{ 0.1 };
    double yaw_direct_{ 1. }, pitch_direct_{ 1. };
    XmlRpc::XmlRpcValue pitch_value_;
    bool has_target_pitch_{ false };
};


#endif //NEW_BEHAVIOR_TREE_UNION_COMMAND_SENDER_H