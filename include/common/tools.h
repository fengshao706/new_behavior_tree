//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_TOOLS_H
#define NEW_BEHAVIOR_TREE_TOOLS_H

//
// Created by spy on 2023/7/26.
//
#pragma once

#include "rm_common/decision/service_caller.h"
#include <math.h>
#include <rm_msgs/EnableGyro.h>
#include <rm_msgs/SetLimitVel.h>
#include <rm_msgs/PriorityArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TransformStamped.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/RecoveryAction.h>
#include <global_planner/GlobalPlannerConfig.h>
#include <dynamic_reconfigure/client.h>
#include "rm_common/decision/command_sender.h"
#include "rm_common/filters/filters.h"
#include "rm_common/decision/service_caller.h"
#include "../union_command_sender.h"
#include "behaviortree_cpp/blackboard.h"

namespace tools
{
  class EnableGyroServiceCaller : public rm_common::ServiceCallerBase<rm_msgs::EnableGyro>
  {
  public:
    explicit EnableGyroServiceCaller(ros::NodeHandle& nh) : ServiceCallerBase<rm_msgs::EnableGyro>(nh, "/enable_gyro")
    {
      service_.request.gyro_speed = 0.0;
      callService();
    }
    void setGyro(double gyro_speed)
    {
      service_.request.gyro_speed = gyro_speed;
    }
    bool isGyro()
    {
      return service_.response.is_gyro;
    }
    void enable()
    {
      callService();
    }
  };

  class SetLimitVelServiceCaller : public rm_common::ServiceCallerBase<rm_msgs::SetLimitVel>
  {
  public:
    explicit SetLimitVelServiceCaller(ros::NodeHandle& nh, double init_limit_vel)
      : ServiceCallerBase<rm_msgs::SetLimitVel>(nh, "/set_limit_vel")
    {
      service_.request.limit_vel = init_limit_vel;
      callService();
    }
    void setLimitVel(double& limit_vel)
    {
      service_.request.limit_vel = limit_vel;
      //    ROS_INFO("set planner's limit vel: %f", service_.request.limit_vel);
      callService();
    }
    void setSideWindow(double slide_window)
    {
      service_.request.slide_window = slide_window;
      callService();
    }
    double getLimitVel()
    {
      return service_.response.current_limit_vel;
    }
  };

  class CmdTools
  {
  public:
    explicit CmdTools(ros::NodeHandle& nh)
    {
      ros::NodeHandle chassis_nh(nh, "chassis");
      chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
      ros::NodeHandle union_nh(nh, "union");
      union_cmd_sender_ = new UnionCommandSender(union_nh);
      ros::NodeHandle vel_nh(nh, "vel");
      vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);

      mbf_client_ =
          std::make_unique<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>("/move_base_flex/move_base", true);
      dClient_ = new dynamic_reconfigure::Client<global_planner::GlobalPlannerConfig>("/move_base_flex/GlobalPlanner");

      ros::NodeHandle yaw_nh(nh, "yaw");
      yaw_nh.getParam("acc", yaw_acc_);
      ros::NodeHandle pitch_nh(nh, "pitch");
      pitch_nh.getParam("acc", pitch_acc_);
      ramp_yaw_ = new RampFilter<double>(yaw_acc_, 0.01);
      ramp_pitch_ = new RampFilter<double>(pitch_acc_, 0.01);
      if (!yaw_pid_.init(ros::NodeHandle(yaw_nh, "pid")))
        ROS_WARN("yaw pid has not define.");
    }

    void yawPidCompute(const double angle)
    {
      double cmd = yaw_pid_.computeCommand(angle, ros::Duration(0.01));
      /*if (cmd > 0.5)
        cmd = std::copysign(0.5, cmd);
      if (cmd < -0.5)
        cmd = std::copysign(-0.5, cmd);*/
      union_cmd_sender_->yaw_direct_ = smoothlyYawOutput(cmd);
    }

    double smoothlyYawOutput(const double cmd)
    {
      ramp_yaw_->setAcc(yaw_acc_);
      ramp_yaw_->input(cmd);
      return ramp_yaw_->output();
    }

    double smoothlyPitchOutput(const double cmd)
    {
      ramp_pitch_->setAcc(pitch_acc_);
      ramp_pitch_->input(cmd);
      return ramp_pitch_->output();
    }

    void setGlobalPlannerParam(int lethal_cost, int neutral_cost)
    {
      if (last_lethal_cost_ != lethal_cost || last_neutral_cost_ != neutral_cost)
      {
        global_planner::GlobalPlannerConfig config;
        dClient_->getCurrentConfiguration(config);
        ROS_INFO_STREAM("cur neutral cost: " << config.neutral_cost << " target neutral_cost:" << neutral_cost
                                             << " cur lethal cost:" << config.lethal_cost
                                             << " target lethal cost:" << lethal_cost);
        config.neutral_cost = neutral_cost;
        config.lethal_cost = lethal_cost;
        last_neutral_cost_ = neutral_cost;
        last_lethal_cost_ = lethal_cost;
        dClient_->setConfiguration(config);
      }
    }

    void getGlobalPlannerDefaultConfig()
    {
      //    global_planner::GlobalPlannerConfig config;
      //    dClient_->getCurrentConfiguration(config);
      //    default_neutral_cost_ = config.neutral_cost;
      //    default_lethal_cost_ = config.lethal_cost;
      //    last_neutral_cost_ = config.neutral_cost;
      //    last_lethal_cost_ = config.lethal_cost;
      //    ROS_INFO_STREAM_THROTTLE(1.0, "default neutral cost:" << default_neutral_cost_
      //                                                          << "default lethal cost:" << default_lethal_cost_);
    }

    void sendStackGimbalCommand(ros::Time time)
    {
      union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
      union_cmd_sender_->base_gimbal_cmd_sender_->sendCommand(time);
    }

    void setStackGimbalMode(int mode)
    {
      union_cmd_sender_->gimbal_cmd_sender_->setMode(mode);
      union_cmd_sender_->base_gimbal_cmd_sender_->setMode(mode);
    }

    void setStackGimbalRate(double scale_base_yaw, double scale_yaw, double scale_pitch)
    {
      union_cmd_sender_->gimbal_cmd_sender_->setRate(scale_yaw, scale_pitch);
      union_cmd_sender_->base_gimbal_cmd_sender_->setRate(scale_base_yaw, 0.0);
    }

    void setStackGimbalPoint()
    {
    }

    rm_common::ChassisCommandSender* chassis_cmd_sender_{};
    UnionCommandSender* union_cmd_sender_{};
    rm_common::Vel2DCommandSender* vel_2d_cmd_sender_;
    //  std::pair<ros::Time, geometry_msgs::TransformStamped> check_obstacle_{};

    std::unique_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> mbf_client_;
    dynamic_reconfigure::Client<global_planner::GlobalPlannerConfig>* dClient_;

    control_toolbox::Pid yaw_pid_;
    // PID

    RampFilter<double>* ramp_yaw_{};
    RampFilter<double>* ramp_pitch_{};
    double pitch_acc_, yaw_acc_;

    int last_neutral_cost_, last_lethal_cost_, default_neutral_cost_, default_lethal_cost_;

    //  std::deque<std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>> goal_deque;
  };

  /**
   *@brief 该函数用于获取某个区域的用于导航的坐标点
   *@param area_name 所要获取的点所在的区域
   *@param blackboard 黑板
   *@param last_patrol_position_index 上一次的导航点，当使用队列模式的时候，该导航点下标每次会加一,你需要在外部维护一个这个变量
   *@param sequential_patrol_enable 是否使用队列模式，若为否，则使用随机模式
   *@param is_complete 所有的巡逻点是否已经全部巡逻完毕，该变量需要在外部进行维护并可读取它的值以做进一步分析
   * **/

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name,BT::Blackboard &blackboard,int &last_patrol_position_index , bool sequential_patrol_enable , bool & is_complete)
  {
    auto all_zones=blackboard.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");

    std::vector<geometry_msgs::PoseStamped> points = all_zones[area_name];

    if (last_patrol_position_index == points.size()-2) //当下标到了区域点最多的情况，为防止溢出，就将其赋值为-1
    {
      is_complete = true;
    }
    if (sequential_patrol_enable == true)
    {
      last_patrol_position_index = ((last_patrol_position_index + 1) % points.size());
      return points[last_patrol_position_index];
    }
    else
    {
      unsigned int rand_index = (rand() % points.size());
      return points[rand_index];
    }
  }


}


#endif //NEW_BEHAVIOR_TREE_TOOLS_H