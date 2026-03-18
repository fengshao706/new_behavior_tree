//
// Created by root on 2026/3/15.
//

#ifndef NEW_BEHAVIOR_TREE_NAVIGATION_BRIDGE_H
#define NEW_BEHAVIOR_TREE_NAVIGATION_BRIDGE_H

#include "tf2/LinearMath/Quaternion.h"
#include "XmlRpc.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Transform.h"
#include "rm_msgs/MapSentryData.h"
#include "nav_msgs/Path.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "types.h"

class NavigationBridge
{
public:
  NavigationBridge()
  {
    ros::NodeHandle root_nh;
    root_nh.getParam("map2minimap", map2minimap_);
  }

  void minimapToWorld()
  {
    tf2::Transform world2minimap_;
    world2minimap_.setOrigin(
        tf2::Vector3(static_cast<double>(map2minimap_[0]), static_cast<double>(map2minimap_[1]), 0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, static_cast<double>(map2minimap_[2]));
    world2minimap_.setRotation(quaternion);
    minimap2world_ = world2minimap_.inverse();
  }

  void pathPointTransform(rm_msgs::MapSentryData* map_sentry_data, const geometry_msgs::PoseStamped& goal, int num,
                          bool is_start_point)
  {
    if (num >= 49)
      return;
    tf2::Transform minimap2path, world2path;
    world2path.setOrigin(tf2::Vector3(goal.pose.position.x, goal.pose.position.y, 0.0));
    world2path.setRotation(tf2::Quaternion(0, 0, 0, 1));
    minimap2path = minimap2world_ * world2path;
    if (is_start_point)
    {
      map_sentry_data->start_position_x = minimap2path.getOrigin().x() * 10.0;
      map_sentry_data->start_position_y = minimap2path.getOrigin().y() * 10.0;
    }
    else
    {
      map_sentry_data->delta_x[num] = (int8_t)(minimap2path.getOrigin().x() * 10.0 - last_point_x_ * 10.0);
      map_sentry_data->delta_y[num] = (int8_t)(minimap2path.getOrigin().y() * 10.0 - last_point_y_ * 10.0);
    }
    last_point_x_ = minimap2path.getOrigin().x();
    last_point_y_ = minimap2path.getOrigin().y();
  }

  void pathTransform(const nav_msgs::Path& goal_path, rm_msgs::MapSentryData* map_sentry_data)
  {
    map_sentry_data->stamp = goal_path.header.stamp;
    map_sentry_data->intention = control_state_;  // TODO : 这个需要从外部赋值
    int num = 0;
    int step = ceil(goal_path.poses.size() / 10.0);
    for (const auto& goal : goal_path.poses)
    {
      if (step != 0)
      {
        if (num == 0)
          pathPointTransform(map_sentry_data, goal, num / step - 1, true);
        else if (num % step == 0)
          pathPointTransform(map_sentry_data, goal, num / step - 1, false);
      }
      num = num + 1;
    }
  }

  void targetPoseTransform(float sub_x, float sub_y, geometry_msgs::PoseStamped* target_pose)
  {
    tf2::Transform minimap2target, world2target;
    minimap2target.setOrigin(tf2::Vector3(sub_x, sub_y, 0));
    minimap2target.setRotation(tf2::Quaternion(0, 0, 0, 1));
    world2target = minimap2world_.inverse() * minimap2target;
    target_pose->header.frame_id = "map";
    target_pose->pose.position.x = world2target.getOrigin().x();
    target_pose->pose.position.y = world2target.getOrigin().y();
    target_pose->pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  }
private:
  XmlRpc::XmlRpcValue map2minimap_;
  tf2::Transform minimap2world_;
  double last_point_x_{}, last_point_y_{};
  int control_state_{ 3 };
};

#endif //NEW_BEHAVIOR_TREE_NAVIGATION_BRIDGE_H