//
// Created by fengshao on 2026/3/26.
//

#ifndef NEW_BEHAVIOR_TREE_GIMBAL_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_GIMBAL_CONDITION_NODE_H

#include <behaviortree_cpp/condition_node.h>

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/action_node.h>
#include "ros/ros.h"
#include "perception_layer.h"

namespace gimbal
{
  class IsTrackLoss : public BT::ConditionNode
  {
  public:
    IsTrackLoss(std::string &name , BT::NodeConfig &config , BT::Blackboard &blackboard) : ConditionNode(name ,config) ,blackboard_(blackboard)
    {

    }

    BT::NodeStatus tick() override
    {
      ros::Time last_track_time;
      double lost_track_tolerant_sec;
      try
      {
        last_track_time = blackboard_.get<ros::Time>("last_track_time");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [last_track_time] , default value is ros::Time(0)");
        last_track_time = ros::Time(0);
      }

      lost_track_tolerant_sec = blackboard_.get<double>("lost_track_tolerant_sec");

      if (ros::Time::now() - last_track_time > ros::Duration(lost_track_tolerant_sec))
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    BT::Blackboard &blackboard_;
  };

  class IsNeedInverseGimbal : public BT::ConditionNode
  {
  public:
    IsNeedInverseGimbal(std::string& name, BT::NodeConfig& config , BT::Blackboard &blackboard ,  perception::Subscriber &subscriber) : BT::ConditionNode(name, config) , blackboard_(blackboard) , subscriber_(subscriber)
    {

    }

    BT::NodeStatus tick() override
    {
      std::vector<int> default_aim_rank;
      default_aim_rank = blackboard_.get<std::vector<int>>("default_aim_rank");
      if (subscriber_.has_back_camera_detected_ &&
        subscriber_.back_camera_detection_id_ != 0)
      {
        if (default_aim_rank[subscriber_.back_camera_detection_id_] == 0) //id是攻击优先级所在的数组下标，数组内部的值为攻击优先级
        {
          return BT::NodeStatus::FAILURE;
        }else
        {
          subscriber_.has_back_camera_detected_ = false;
          subscriber_.back_camera_detection_id_ = 0;
          return BT::NodeStatus::SUCCESS;  //如果不是等于0的优先级（无效），就把云台反过来
        }
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;
  };
}

#endif //NEW_BEHAVIOR_TREE_GIMBAL_CONDITION_NODE_H