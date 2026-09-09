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
#include <rm_msgs/PriorityArray.h>
#include "rm_common/ori_tool.h"
#include "common/invincible_detection.h"

namespace gimbal
{
  class SetGimbalMode : public BT::SyncActionNode
  {
  public:
    SetGimbalMode(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name , config)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("input_gimbal_mode_id"),
                BT::OutputPort<int>("gimbal_mode")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> input_gimbal_mode_id = getInput<int>("input_gimbal_mode_id");
      setOutput<int>("gimbal_mode", input_gimbal_mode_id.value());
      return BT::NodeStatus::SUCCESS;
    }

  private:

  };

  class YawSlowRound : public BT::StatefulActionNode
  {
  public:
    YawSlowRound(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools , tools::CmdTools &cmd_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("yaw_vel"), //yaw轴旋转速度
        BT::InputPort<int>("scan_range_circles"),
        BT::InputPort<double>("pitch_inside_vel"),
        BT::InputPort<double>("pitch_outside_vel"),
        BT::InputPort<double>("pitch_min"),
        BT::InputPort<double>("pitch_max"),
        BT::InputPort<double>("breach_thresholds")
      }; //yaw每转多少圈就回复
    }

    BT::NodeStatus onStart() override
    {
      gimbal_tools_.resetTrajToCurrent();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      double yaw_vel;
      int scan_range_circles;
      double pitch_inside_vel;
      double pitch_outside_vel;
      double pitch_min;
      double pitch_max;
      double breach_thresholds;

      getInput<double>("yaw_vel",yaw_vel);
      getInput<int>("scan_range_circles",scan_range_circles);
      getInput<double>("pitch_inside_vel",pitch_inside_vel);
      getInput<double>("pitch_outside_vel",pitch_outside_vel);
      getInput<double>("pitch_min",pitch_min);
      getInput<double>("pitch_max",pitch_max);
      getInput<double>("breach_thresholds",breach_thresholds);
      gimbal_tools_.lidarTwist(yaw_vel,scan_range_circles);
      gimbal_tools_.updatePitchStrafeDirect(pitch_min, pitch_max,
                  pitch_outside_vel, pitch_inside_vel,
                  breach_thresholds);
      gimbal_tools_.setStackGimbalRate();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }
  private:
    tools::GimbalTools &gimbal_tools_;
    tools::CmdTools &cmd_tools_;
  };

  class InverseGimbal : public BT::StatefulActionNode //需用timeout节点维持运行一小段时间
  {
  public:
    InverseGimbal(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber , perception::TfAccessor &tf_accessor) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools) , cmd_tools_(cmd_tools) , tf_accessor_(tf_accessor), subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      geometry_msgs::TransformStamped back_camera2map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP , perception::TfAccessor::FrameId::BACK_CAMERA_OPTICAL_FRAME);
      geometry_msgs::PointStamped target2back;
      target2back.header.frame_id = "back_camera_optical_frame";
      target2back.point.x = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.x;
      target2back.point.y = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.y;
      target2back.point.z = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.z;
      tf2::doTransform(target2back, back_of_map_, back_camera2map); //获取目标在map上面的位置
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(back_of_map_);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }
  private:
    tools::GimbalTools &gimbal_tools_;
    tools::CmdTools &cmd_tools_;
    perception::TfAccessor &tf_accessor_;
    perception::Subscriber &subscriber_;
    geometry_msgs::PointStamped back_of_map_;
  };

  class TrackEnemy : public BT::StatefulActionNode
  {
  public:
    TrackEnemy(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools ,tools::GimbalTools &gimbal_tools , tools::AutoAimTools &auto_aim_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools) , gimbal_tools_(gimbal_tools) , auto_aim_tools_(auto_aim_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("target_type", "armor")};
    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      const std::string target_type = getInput<std::string>("target_type").value();
      if (target_type == "armor")
      {
        auto_aim_tools_.restoreArmorWindow();
      }else if (target_type == "small_buff")
      {
        auto_aim_tools_.enterBuffWindow(rm_msgs::StatusChangeRequest::SMALL_BUFF,ros::Time::now());
      }else if (target_type == "big_buff")
      {
        auto_aim_tools_.enterBuffWindow(rm_msgs::StatusChangeRequest::BIG_BUFF,ros::Time::now());
      }
      gimbal_tools_.setStackGimbalTrack();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }
  private:
    tools::CmdTools &cmd_tools_;
    tools::GimbalTools &gimbal_tools_;
    tools::AutoAimTools &auto_aim_tools_;
  };

  class PreAimingOutpost : public BT::StatefulActionNode
  {
  public:
    PreAimingOutpost(const std::string &name , const BT::NodeConfig &config , tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color"),
                  BT::InputPort<double>("aim_per_point_sec"),
                BT::InputPort<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses") ,
                  BT::InputPort<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses")};
    }

    BT::NodeStatus onStart() override
    {
      aim_per_point_sec_ = getInput<double>("aim_per_point_sec").value();
      robot_color_ = getInput<std::string>("robot_color").value();
      if (robot_color_ == "blue")
      {
        enemy_outpost_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses").value();
      }else
      {
        enemy_outpost_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses").value();
      }
      record_aim_time_ = ros::Time::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(enemy_outpost_positions_[current_point_ % enemy_outpost_positions_.size()]);
      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_)) //大于设定的超时时间时换下一个点
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(enemy_outpost_positions_.size()))
        {
          ROS_INFO("Fail to attack outpost.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      current_point_ = 0;
    }
  private:
    std::string robot_color_;
    std::vector<geometry_msgs::PointStamped> enemy_outpost_positions_;
    int current_point_ {0};
    double aim_per_point_sec_;
    ros::Time record_aim_time_ = ros::Time::now();
    tools::GimbalTools &gimbal_tools_;
  };

  class PreAimingBase : public BT::StatefulActionNode
  {
  public:
    PreAimingBase(const std::string &name , const BT::NodeConfig &config , tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color"),
                  BT::InputPort<double>("aim_per_point_sec"),
                BT::InputPort<std::vector<geometry_msgs::PointStamped>>("blue_base_poses") ,
                  BT::InputPort<std::vector<geometry_msgs::PointStamped>>("red_base_poses")};
    }

    BT::NodeStatus onStart() override
    {
      aim_per_point_sec_ = getInput<double>("aim_per_point_sec").value();
      robot_color_ = getInput<std::string>("robot_color").value();
      if (robot_color_ == "blue")
      {
        enemy_base_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("blue_base_poses").value();
      }else
      {
        enemy_base_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("red_base_poses").value();
      }
      record_aim_time_ = ros::Time::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(enemy_base_positions_[current_point_ % enemy_base_positions_.size()]);
      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_)) //大于设定的超时时间时换下一个点
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(enemy_base_positions_.size()))
        {
          ROS_INFO("Fail to attack outpost.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      current_point_ = 0;
    }
  private:
    std::string robot_color_;
    std::vector<geometry_msgs::PointStamped> enemy_base_positions_;
    int current_point_ {0};
    double aim_per_point_sec_;
    ros::Time record_aim_time_ = ros::Time::now();
    tools::GimbalTools &gimbal_tools_;
  };
}


#endif //NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H