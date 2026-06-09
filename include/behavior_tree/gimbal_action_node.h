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

namespace gimbal
{
  class SetGimbalMode : public BT::SyncActionNode
  {
  public:
    SetGimbalMode(const std::string &name ,const BT::NodeConfig &config , BT::Blackboard &blackboard) : SyncActionNode(name , config) , blackboard_(blackboard)
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

  class YawSlowRound : public BT::StatefulActionNode
  {
  public:
    YawSlowRound(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools)
    {

    }

    BT::PortsList providedPorts()
    {
      return {
        BT::InputPort("yaw_vel"), //yaw轴旋转速度
        BT::InputPort("scan_range_circles"),
        BT::InputPort("pitch_inside_vel"),
        BT::InputPort("pitch_outside_vel"),
        BT::InputPort("pitch_min"),
        BT::InputPort("pitch_max"),
        BT::InputPort("breach_thresholds")
      }; //yaw每转多少圈就回复
    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      BT::Expected<double> yaw_vel;
      BT::Expected<double> scan_range_circles;
      BT::Expected<double> pitch_inside_vel;
      BT::Expected<double> pitch_outside_vel;
      BT::Expected<double> pitch_min;
      BT::Expected<double> pitch_max;
      BT::Expected<double> breach_thresholds;

      getInput("yaw_vel",yaw_vel);
      getInput("scan_range_circles",scan_range_circles);
      getInput("pitch_inside_vel",pitch_inside_vel);
      getInput("pitch_outside_vel",pitch_outside_vel);
      getInput("pitch_min",pitch_min);
      getInput("pitch_max",pitch_max);
      getInput("breach_thresholds",breach_thresholds);
      gimbal_tools_.lidarTwist(yaw_vel.value(),scan_range_circles.value());
      gimbal_tools_.updatePitchStrafeDirect(pitch_min.value(), pitch_max.value(),
                  pitch_outside_vel.value(), pitch_inside_vel.value(),
                  breach_thresholds.value());
      gimbal_tools_.setStackGimbalRate();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {

    }
  private:
    tools::GimbalTools &gimbal_tools_;
  };

  class InverseGimbal : public BT::StatefulActionNode //需用timeout节点维持运行一小段时间
  {
  public:
    InverseGimbal(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools , perception::Subscriber &subscriber , perception::TfAccessor &tf_accessor) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools) , tf_accessor_(tf_accessor), subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      geometry_msgs::TransformStamped map2back_camera = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP , perception::TfAccessor::FrameId::BACK_CAMERA_OPTICAL_FRAME);
      tf2::doTransform(subscriber_.getBackCameraDetection(), back_of_map_, map2back_camera); //获取目标在map上面的位置
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(back_of_map_);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {

    }
  private:
    tools::GimbalTools &gimbal_tools_;
    perception::TfAccessor &tf_accessor_;
    perception::Subscriber &subscriber_;
    geometry_msgs::PointStamped back_of_map_;
  };

  class TrackEnemy : public BT::StatefulActionNode
  {
  public:
    TrackEnemy(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools ,tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools) , gimbal_tools_(gimbal_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setStackGimbalTrack();
      return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {

    }
  private:
    tools::CmdTools &cmd_tools_;
    tools::GimbalTools &gimbal_tools_;
    double bullet_speed_{};
  };
}


#endif //NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H