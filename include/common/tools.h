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
    explicit EnableGyroServiceCaller(ros::NodeHandle& nh);

    void setGyro(double gyro_speed);

    bool isGyro();

    void enable();
  };

  class SetLimitVelServiceCaller : public rm_common::ServiceCallerBase<rm_msgs::SetLimitVel>
  {
  public:
    explicit SetLimitVelServiceCaller(ros::NodeHandle& nh,const double init_limit_vel);

    void setLimitVel(const double& limit_vel);

    void setSlideWindow(const double slide_window);

    double getLimitVel() const;
  };

  class CmdTools
  {
  public:
    struct Senders
    {
      std::unique_ptr<rm_common::ChassisCommandSender> chassis_command_sender_;
      std::unique_ptr<rm_common::GimbalCommandSender> gimbal_command_sender_;
      std::unique_ptr<rm_common::GimbalCommandSender> base_gimbal_command_sender_;
      std::unique_ptr<rm_common::ShooterCommandSender> shooter_command_sender_;
      std::unique_ptr<rm_common::Vel2DCommandSender> vel_2d_command_sender_;
    };

    CmdTools(ros::NodeHandle& nh);

    Senders * getSenders() const;

    auto getMbfClient() const;

    auto getDClient() const;

    double getYawDirect() const;

    void setYawDirect(double yaw_direct);

    void yawPidCompute(const double angle);

    double smoothlyYawOutput(const double cmd);

    double smoothlyPitchOutput(const double cmd);

    void setGlobalPlannerParam(int lethal_cost, int neutral_cost);

    void getGlobalPlannerDefaultConfig();

    void sendStackGimbalCommand(ros::Time time);

    void setStackGimbalMode(int mode);

    void setStackGimbalRate(double scale_base_yaw, double scale_yaw, double scale_pitch);

    void setStackGimbalPoint();

    tf2_ros::Buffer& getTfBuffer();

  private:
    std::unique_ptr<Senders> senders_;
    std::unique_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> mbf_client_;
    std::unique_ptr<dynamic_reconfigure::Client<global_planner::GlobalPlannerConfig>> dClient_;

    control_toolbox::Pid yaw_pid_;
    // PID

    RampFilter<double>* ramp_yaw_{};
    RampFilter<double>* ramp_pitch_{};
    double pitch_acc_, yaw_acc_;
    double yaw_direct_{};

    int last_neutral_cost_, last_lethal_cost_, default_neutral_cost_, default_lethal_cost_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
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

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name,BT::Blackboard &blackboard,int &last_patrol_position_index , bool sequential_patrol_enable , bool & is_complete);

  bool isPointInPolygon(const geometry_msgs::TransformStamped& point,
                        const std::vector<geometry_msgs::PointStamped>& polygon);

  std::string determinePolygonInWhich(const geometry_msgs::TransformStamped& point , std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>> pos_detection_polygons);

}


#endif //NEW_BEHAVIOR_TREE_TOOLS_H