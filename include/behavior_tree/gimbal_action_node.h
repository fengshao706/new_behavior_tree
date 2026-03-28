//
// Created by fengshao on 2026/3/26.
//

#ifndef NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H

#include <common/tools.h>

#include "common/types.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "geometry_msgs/TransformStamped.h"
#include "rm_common/ori_tool.h"
#include "common/behavior_base.h"

namespace gimbal
{
  class SetGimbalMode : public BT::SyncActionNode
  {
  public:
    SetGimbalMode(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name , config) , blackboard_(blackboard)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("gimbal_mode_id") };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> gimbal_mode_id = getInput<int>("gimbal_mode_id");
      blackboard_.set<types::GimbalMode>("gimbal_mode",static_cast<types::GimbalMode>(gimbal_mode_id.value()));
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Blackboard &blackboard_;
  };

  class YawSlowRound : public BT::SyncActionNode
  {
  public:
    YawSlowRound(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , tools::CmdTools &cmd_tools ,BehaviorBase &behavior_base) : SyncActionNode(name , config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    BT::NodeStatus tick() override
    {
      XmlRpc::XmlRpcValue gimbal_vel_coeff = blackboard_.get<XmlRpc::XmlRpcValue>("gimbal_vel_coeff");

      behavior_base_.lidarTwist(gimbal_vel_coeff[1]);
      behavior_base_.pitchStrafe(cmd_tools_.union_cmd_sender_->pitch_min_, cmd_tools_.union_cmd_sender_->pitch_max_,
                  cmd_tools_.union_cmd_sender_->pitch_outside_vel_, cmd_tools_.union_cmd_sender_->pitch_inside_vel_,
                  cmd_tools_.union_cmd_sender_->breach_threshold_);
      behavior_base_.setGimbalRate();
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BT::Blackboard &blackboard_;
    tools::CmdTools &cmd_tools_;
    BehaviorBase &behavior_base_;
  };

  class LidarTowardsFront : public BT::SyncActionNode
  {
  public:
    LidarTowardsFront(std::string &name , BT::NodeConfig &config , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber , BehaviorBase &behavior_base) : SyncActionNode(name , config) , cmd_tools_(cmd_tools) , subscriber_(subscriber) , behavior_base_(behavior_base) , tf_buffer_(cmd_tools.getTfBuffer())
    {
      axis_z_.header.frame_id = "livox_frame";
      axis_z_.point.x = 0;
      axis_z_.point.y = 0;
      axis_z_.point.z = 1;
    }

    BT::NodeStatus tick() override
    {
      geometry_msgs::PointStamped lidar_z;
      try
      {
        lidar_z = tf_buffer_.transform(axis_z_, "base_link");
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }
      double lidar_angle = atan2(lidar_z.point.y, lidar_z.point.x);
      double vx = subscriber_.odom_.twist.twist.linear.x;
      double vy = subscriber_.odom_.twist.twist.linear.y;
      if (hypot(vx, vy) > 0.2)
        target_yaw_ = atan2(vy, vx);
      double yaw_error = target_yaw_ - lidar_angle;
      if (yaw_error > M_PI)
        yaw_error -= 2 * M_PI;
      else if (yaw_error < -M_PI)
        yaw_error += 2 * M_PI;
      cmd_tools_.yawPidCompute(yaw_error);
      behavior_base_.pitchStrafe(cmd_tools_.union_cmd_sender_->pitch_min_, cmd_tools_.union_cmd_sender_->pitch_max_,
                  cmd_tools_.union_cmd_sender_->pitch_outside_vel_, cmd_tools_.union_cmd_sender_->pitch_inside_vel_,
                  cmd_tools_.union_cmd_sender_->breach_threshold_);
      behavior_base_.setGimbalRate();
      return BT::NodeStatus::SUCCESS;
    }

  private:
    tools::CmdTools &cmd_tools_;
    perception::Subscriber &subscriber_;
    BehaviorBase &behavior_base_;
    tf2_ros::Buffer &tf_buffer_;
    geometry_msgs::PointStamped axis_z_;
    double target_yaw_;
  };

  class RoundSearchEnemy : public BT::SyncActionNode
  {
  public:
    RoundSearchEnemy(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , tools::CmdTools &cmd_tools , BehaviorBase &behavior_base) : SyncActionNode(name , config) , blackboard_(blackboard) , cmd_tools_(cmd_tools) , behavior_base_(behavior_base)
    {

    }

    BT::NodeStatus tick() override
    {
      XmlRpc::XmlRpcValue gimbal_vel_coeff = blackboard_.get<XmlRpc::XmlRpcValue>("gimbal_vel_coeff");

      behavior_base_.lidarTwist(gimbal_vel_coeff[0]);
      behavior_base_.pitchStrafe(cmd_tools_.union_cmd_sender_->pitch_min_, cmd_tools_.union_cmd_sender_->pitch_max_,
                  cmd_tools_.union_cmd_sender_->pitch_outside_vel_, cmd_tools_.union_cmd_sender_->pitch_inside_vel_,
                  cmd_tools_.union_cmd_sender_->breach_threshold_);
      behavior_base_.setGimbalRate();
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Blackboard &blackboard_;
    tools::CmdTools &cmd_tools_;
    BehaviorBase &behavior_base_;
  };

  class InverseGimbal : public BT::SyncActionNode
  {
  public:
    InverseGimbal(std::string &name , BT::NodeConfig &config , BehaviorBase &behavior_base , perception::Subscriber &subscriber , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , behavior_base_(behavior_base) , subscriber_(subscriber) , tf_buffer_(cmd_tools.getTfBuffer())
    {

    }

    BT::NodeStatus tick() override
    {
      geometry_msgs::TransformStamped map2back_camera;

      map2back_camera = tf_buffer_.lookupTransform("map", "back_camera_optical_frame", ros::Time(0));
      tf2::doTransform(subscriber_.back_of_camera_, back_of_map_, map2back_camera);

      behavior_base_.setGimbalDirectPoint(back_of_map_);
      if (!behavior_base_.gimbal_inverse_timer_.hasStarted())
      {
        behavior_base_.gimbal_inverse_timer_.start();
      }
      return BT::NodeStatus::SUCCESS;
    }
  private:
    BehaviorBase &behavior_base_;
    perception::Subscriber &subscriber_;
    tf2_ros::Buffer &tf_buffer_;
    geometry_msgs::PointStamped back_of_map_;
  };

  class AimOutpost : public BT::StatefulActionNode
  {
  public:
    AimOutpost(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , BehaviorBase &behavior_base) : StatefulActionNode(name , config) , blackboard_(blackboard) , behavior_base_(behavior_base)
    {

    }

    BT::NodeStatus onStart() override
    {
      blue_outpost_poses_ = blackboard_.get<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses");
      red_outpost_poses_ = blackboard_.get<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses");
      robot_color_ = blackboard_.get<std::string>("robot_color");
      if (robot_color_ == "blue") //因为要打击敌人的前哨站，所以要将颜色反相
      {
        robot_color_ = "red";
      }else
      {
        robot_color_ = "blue";
      }
      aim_per_point_sec_ = blackboard_.get<double>("aim_per_point_sec");
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

      behavior_base_.setGimbalDirectPoint(robot_color_ == "blue" ? blue_outpost_poses_[current_point_ % blue_outpost_poses_.size()] :
                                           red_outpost_poses_[current_point_ % red_outpost_poses_.size()]);

      if (record_aim_time_.isZero()) {  //首次调用的时候初始化时间
        record_aim_time_ = ros::Time::now();
      }

      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_))
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(robot_color_ == "blue" ? blue_outpost_poses_.size() : red_outpost_poses_.size()))
        {
          ROS_INFO("Fail to attack outpost.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {

    }
  private:
    BT::Blackboard &blackboard_;
    BehaviorBase &behavior_base_;
    int current_point_ = 0;
    ros::Time record_aim_time_;
    std::vector<geometry_msgs::PointStamped> blue_outpost_poses_;
    std::vector<geometry_msgs::PointStamped> red_outpost_poses_;
    std::string robot_color_;
    double aim_per_point_sec_;
  };

  class AimBase : public BT::StatefulActionNode
  {
  public:
    AimBase(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard , BehaviorBase &behavior_base , perception::Subscriber &subscriber) : StatefulActionNode(name , config) , blackboard_(blackboard) , behavior_base_(behavior_base) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      blue_base_poses_ = blackboard_.get<std::vector<geometry_msgs::PointStamped>>("blue_base_poses");
      red_base_poses_ = blackboard_.get<std::vector<geometry_msgs::PointStamped>>("red_base_poses");
      robot_color_ = blackboard_.get<std::string>("robot_color");
      if (robot_color_ == "blue") //因为要打击敌人的前哨站，所以要将颜色反相
      {
        robot_color_ = "red";
      }else
      {
        robot_color_ = "blue";
      }
      aim_per_point_sec_ = blackboard_.get<double>("aim_per_point_sec");
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      behavior_base_.setGimbalDirectPoint(robot_color_ == "blue" ? blue_base_poses_[current_point_ % blue_base_poses_.size()] :
                                         red_base_poses_[current_point_ % red_base_poses_.size()]);

      if (record_aim_time_.isZero()) {  //首次调用的时候初始化时间
        record_aim_time_ = ros::Time::now();
      }

      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_))
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(robot_color_ == "blue" ? blue_base_poses_.size() : red_base_poses_.size()))
        {
          subscriber_.client_map_update_ = false;
          ROS_INFO("Fail to attack base.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {

    }
  private:
    BT::Blackboard &blackboard_;
    BehaviorBase &behavior_base_;
    int current_point_ = 0;
    ros::Time record_aim_time_;
    std::vector<geometry_msgs::PointStamped> blue_base_poses_;
    std::vector<geometry_msgs::PointStamped> red_base_poses_;
    std::string robot_color_;
    double aim_per_point_sec_;
    perception::Subscriber &subscriber_;
  };

  class TrackEnemy : public BT::SyncActionNode
  {
  public:
    TrackEnemy(std::string &name , BT::NodeConfig &config , tools::CmdTools &cmd_tools) : SyncActionNode(name , config) , cmd_tools_(cmd_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time time = ros::Time::now();
      cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(
          cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->getSpeed());
      cmd_tools_.union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    tools::CmdTools &cmd_tools_;
  };
}


#endif //NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H