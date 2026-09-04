//
// Created by Reid on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_SENTRY_PARAM_LOADER_H
#define NEW_BEHAVIOR_TREE_SENTRY_PARAM_LOADER_H

#include "ros/ros.h"
#include "behaviortree_cpp/blackboard.h"
#include "XmlRpcValue.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "common/types.h"
#include "common/chase_policy.h"

class SentryParamLoader  //需在main函数中构造
{
public:
  SentryParamLoader(ros::NodeHandle & bt_nh , BT::Blackboard::Ptr & blackboard) : bt_nh_(bt_nh) , blackboard_(blackboard)
  {
    ROS_INFO("1");
    load_robot_color();
    ROS_INFO("2");
    chassis_behavior_param_load();
    ROS_INFO("3");
    gimbal_behavior_param_load();
    ROS_INFO("4");
    chassis_vel_param_load();
    ROS_INFO("5");
    planner_param_load();
    ROS_INFO("6");
    get_region_key_points();
    ROS_INFO("7");
    load_zone_configs();
    ROS_INFO("8");
    load_default_aim_rank();
    ROS_INFO("9");
    param_initialization();
  }

  void chassis_behavior_param_load()
  {
    ros::NodeHandle chassis_behavior_nh=ros::NodeHandle(bt_nh_,"chassis_behavior");
    ros::NodeHandle auto_nh(bt_nh_,"auto");
    int trigger_blood_return_hp;
    int trigger_blood_return_hp_without_buff;
    int trigger_run_away_outpost_hp;
    int trigger_ban_chase_outpost_hp;
    double max_planning_period;
    double stand_at_conduct_point_sec;
    double chase_freq;
    double chase_distance;
    double chase_tolerance;
    double avoid_drone_time;
    double game_total_time;
    double chasing_max_for_time;
    bool attack_engineer_enable;
    bool attack_outpost_enable;
    bool enable_chase;

    XmlRpc::XmlRpcValue chase_restricted_zones;
    XmlRpc::XmlRpcValue standby_velocity;
    ROS_ASSERT(
    chassis_behavior_nh.getParam("trigger_blood_return_hp", trigger_blood_return_hp) &&
    chassis_behavior_nh.getParam("trigger_blood_return_hp_without_buff", trigger_blood_return_hp_without_buff) &&
    chassis_behavior_nh.getParam("trigger_run_away_outpost_hp", trigger_run_away_outpost_hp) &&//离开前哨站的血量阈值
    chassis_behavior_nh.getParam("trigger_ban_chase_outpost_hp", trigger_ban_chase_outpost_hp) &&//禁止追击敌人到前哨站区域的血量阈值
    chassis_behavior_nh.getParam("chase_restricted_zones", chase_restricted_zones) &&//限制追击的区域
    chassis_behavior_nh.getParam("enable_chase", enable_chase) &&//追击总开关
    chassis_behavior_nh.getParam("max_planning_period",max_planning_period) &&
    chassis_behavior_nh.getParam("stand_at_conduct_point_sec",stand_at_conduct_point_sec) &&
    chassis_behavior_nh.getParam("chase_freq",chase_freq) &&
    chassis_behavior_nh.getParam("chase_distance",chase_distance) &&
    chassis_behavior_nh.getParam("chase_tolerance",chase_tolerance) == true);

    ROS_ASSERT(
    auto_nh.getParam("avoid_drone_time",avoid_drone_time) &&
    auto_nh.getParam("game_total_time",game_total_time) &&
    auto_nh.getParam("chasing_max_for_time",chasing_max_for_time) &&
    auto_nh.getParam("attack_engineer_enable",attack_engineer_enable) &&
    auto_nh.getParam("attack_outpost_enable",attack_outpost_enable) &&
    auto_nh.getParam("standby_velocity", standby_velocity) == true);

    std::vector<chase_policy::ChaseRestrictedZoneConfig> chase_restricted_zone_configs;
    for (int i = 0; i < chase_restricted_zones.size(); ++i)
    {
      chase_policy::ChaseRestrictedZoneConfig c;
      c.name = static_cast<std::string>(chase_restricted_zones[i]["name"]);
      ROS_ASSERT(chase_restricted_zones[i].hasMember("is_target_area"));
      c.is_target_area = static_cast<bool>(chase_restricted_zones[i]["is_target_area"]);
      c.begin_time = static_cast<int>(chase_restricted_zones[i]["begin_time"]);
      c.end_time = static_cast<int>(chase_restricted_zones[i]["end_time"]);
      chase_restricted_zone_configs.push_back(c);
    }

    blackboard_->set<int>("trigger_blood_return_hp",trigger_blood_return_hp);
    blackboard_->set<int>("trigger_blood_return_hp_without_buff", trigger_blood_return_hp_without_buff);
    blackboard_->set<int>("trigger_run_away_outpost_hp", trigger_run_away_outpost_hp);
    blackboard_->set<int>("trigger_ban_chase_outpost_hp", trigger_ban_chase_outpost_hp);

    blackboard_->set<XmlRpc::XmlRpcValue>("standby_velocity", standby_velocity);
    blackboard_->set<double>("max_planning_period",max_planning_period);
    blackboard_->set<double>("stand_at_conduct_point_sec",stand_at_conduct_point_sec);
    blackboard_->set<double>("chase_freq",chase_freq);
    blackboard_->set<double>("chase_distance",chase_distance);
    blackboard_->set<double>("chase_tolerance",chase_tolerance);
    blackboard_->set<double>("avoid_drone_time",avoid_drone_time);
    blackboard_->set<double>("game_total_time",game_total_time);
    blackboard_->set<double>("chasing_max_for_time",chasing_max_for_time);
    blackboard_->set<bool>("attack_engineer_enable",attack_engineer_enable);
    blackboard_->set<bool>("attack_outpost_enable",attack_outpost_enable);
    blackboard_->set<bool>("enable_chase",enable_chase);
    blackboard_->set<std::vector<chase_policy::ChaseRestrictedZoneConfig>>("chase_restricted_zones", chase_restricted_zone_configs);
  }

  void gimbal_behavior_param_load()
  {
    ros::NodeHandle auto_nh(bt_nh_,"auto");
    ros::NodeHandle yaw_nh(bt_nh_, "yaw");
    ros::NodeHandle gimbal_behavior_nh(bt_nh_, "gimbal_behavior");

    double lost_track_tolerant_sec;
    double gimbal_inverse_sec;
    double aim_per_point_sec;
    XmlRpc::XmlRpcValue gimbal_vel_coeff;
    XmlRpc::XmlRpcValue blue_outpost_positions, red_outpost_positions, red_base_positions, blue_base_positions;
    std::vector<geometry_msgs::PointStamped> blue_outpost_poses, red_outpost_poses, blue_base_poses, red_base_poses;

    ROS_ASSERT(
    auto_nh.getParam("lost_track_tolerant_sec",lost_track_tolerant_sec) &&
    yaw_nh.getParam("gimbal_vel_coeff", gimbal_vel_coeff) &&
    gimbal_behavior_nh.getParam("blue_outpost_positions", blue_outpost_positions) &&
    gimbal_behavior_nh.getParam("red_outpost_positions", red_outpost_positions) &&
    gimbal_behavior_nh.getParam("red_base_positions", red_base_positions) &&
    gimbal_behavior_nh.getParam("blue_base_positions", blue_base_positions) &&
    gimbal_behavior_nh.getParam("gimbal_inverse_sec",gimbal_inverse_sec) &&
    gimbal_behavior_nh.getParam("aim_per_point_sec",aim_per_point_sec) == true);

    for (int i = 0; i < blue_outpost_positions.size(); i++)
    {
      geometry_msgs::PointStamped outpost_pose;
      outpost_pose.point.x = static_cast<double>(blue_outpost_positions[i][0]);
      outpost_pose.point.y = static_cast<double>(blue_outpost_positions[i][1]);
      outpost_pose.point.z = static_cast<double>(blue_outpost_positions[i][2]);
      blue_outpost_poses.push_back(outpost_pose);
    }

    for (int i = 0; i < red_outpost_positions.size(); i++)
    {
      geometry_msgs::PointStamped outpost_pose;
      outpost_pose.point.x = static_cast<double>(red_outpost_positions[i][0]);
      outpost_pose.point.y = static_cast<double>(red_outpost_positions[i][1]);
      outpost_pose.point.z = static_cast<double>(red_outpost_positions[i][2]);
      red_outpost_poses.push_back(outpost_pose);
    }
    for (int i = 0; i < blue_base_positions.size(); i++)
    {
      geometry_msgs::PointStamped base_pose;
      base_pose.point.x = static_cast<double>(blue_base_positions[i][0]);
      base_pose.point.y = static_cast<double>(blue_base_positions[i][1]);
      base_pose.point.z = static_cast<double>(blue_base_positions[i][2]);
      blue_base_poses.push_back(base_pose);
    }
    for (int i = 0; i < red_base_positions.size(); i++)
    {
      geometry_msgs::PointStamped base_pose;
      base_pose.point.x = static_cast<double>(red_base_positions[i][0]);
      base_pose.point.y = static_cast<double>(red_base_positions[i][1]);
      base_pose.point.z = static_cast<double>(red_base_positions[i][2]);
      red_base_poses.push_back(base_pose);
    }

    blackboard_->set<double>("lost_track_tolerant_sec",lost_track_tolerant_sec);
    blackboard_->set<XmlRpc::XmlRpcValue>("gimbal_vel_coeff",gimbal_vel_coeff);
    blackboard_->set<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses",blue_outpost_poses);
    blackboard_->set<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses",red_outpost_poses);
    blackboard_->set<std::vector<geometry_msgs::PointStamped>>("blue_base_poses",blue_base_poses);
    blackboard_->set<std::vector<geometry_msgs::PointStamped>>("red_base_poses",red_base_poses);
    blackboard_->set<double>("gimbal_inverse_sec",gimbal_inverse_sec);
    blackboard_->set<double>("aim_per_point_sec",aim_per_point_sec);
  }

  void chassis_vel_param_load() // TODO : 当前未完成全部参数加载任务
  {
    ros::NodeHandle rm_behavior_tree_nh(bt_nh_,"rm_behavior_tree");
    ros::NodeHandle chassis_vel_nh=ros::NodeHandle(rm_behavior_tree_nh,"vel");
    double slow_gyro_vel = chassis_vel_nh.param("slow_gyro_vel",0.5);

    blackboard_->set<double>("still_gyro_vel",slow_gyro_vel);
  }

  void planner_param_load()
  {
    ros::NodeHandle rm_behavior_tree_nh(bt_nh_,"rm_behavior_tree");
    ros::NodeHandle planner_nh=ros::NodeHandle(bt_nh_,"planner");
    double default_limit_vel;
    double slope_side_window;
    double default_side_window;
    int neutral_cost;
    int lethal_cost;

    ROS_ASSERT(
    planner_nh.getParam("default_limit_vel", default_limit_vel) &&
    planner_nh.getParam("slope_side_window", slope_side_window) &&//坡道附近的代价膨胀值
    planner_nh.getParam("default_side_window", default_side_window) &&//普通路段的代价膨胀值
    planner_nh.getParam("neutral_cost", neutral_cost) &&//可通过区域的代价
    planner_nh.getParam("lethal_cost", lethal_cost) == true//不可通过区域的代价
    );
    blackboard_->set<double>("default_limit_vel",default_limit_vel);
    blackboard_->set<double>("slope_side_window",slope_side_window);
    blackboard_->set<double>("default_side_window",default_side_window);
    blackboard_->set<int>("neutral_cost",neutral_cost);
    blackboard_->set<int>("lethal_cost",lethal_cost);
  }

  void get_region_key_points() //用于下面的多边形
  {

    ros::NodeHandle auto_nh(bt_nh_,"auto");
    XmlRpc::XmlRpcValue region_key_points;
    ROS_ASSERT(auto_nh.getParam("region_key_points",region_key_points) == true);
    for (int i = 0; i < region_key_points.size(); i++)
    {
      geometry_msgs::PointStamped region_key_point;
      region_key_point.point.x = static_cast<double>(region_key_points[i][0]);
      region_key_point.point.y = static_cast<double>(region_key_points[i][1]);
      region_key_point.point.z = static_cast<double>(region_key_points[i][2]);
      region_key_points_.push_back(region_key_point);
    }
  }

  void load_zone_configs()
  {
    ros::NodeHandle auto_nh(bt_nh_,"auto");
    ros::NodeHandle chassis_behavior_nh(bt_nh_,"chassis_behavior");
    XmlRpc::XmlRpcValue zones;
    auto_nh.getParam("zones",zones);
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones; //所有的区域的所有坐标
    std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>> pos_detection_polygons;

    for (const auto& zone : zones)
    {
      ROS_ASSERT(zone.second.hasMember("position") and
                 zone.second.hasMember("pos_detection_polygon"));
      ROS_ASSERT(zone.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray and
                 zone.second["pos_detection_polygon"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::vector<geometry_msgs::PoseStamped> points;
      for (int i = 0; i < zone.second["position"].size(); ++i)
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = static_cast<double>(zone.second["position"][i][0]);
        pose_stamped.pose.position.y = static_cast<double>(zone.second["position"][i][1]);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, static_cast<double>(zone.second["position"][i][2]));
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);
        points.push_back(pose_stamped);
      }
      all_zones.insert(std::make_pair(zone.first, points));
      //--------------------------------------------------------------------------
      std::vector<geometry_msgs::PointStamped> polygon_points;
      for (int i = 0; i < zone.second["pos_detection_polygon"].size(); ++i)
      {
        geometry_msgs::PointStamped polygon_point;
        polygon_point.header.frame_id = "map";

        polygon_point = region_key_points_[static_cast<int>(zone.second["pos_detection_polygon"][i]) - 1];
        polygon_points.push_back(polygon_point);
      }
      pos_detection_polygons.insert(std::make_pair(zone.first, polygon_points));
    }
    //-----------------------------------------------------------------------
    //----------------------------------------------------------------------
    std::vector<std::string> red_half_area;
    std::vector<std::string> blue_half_area;
    chassis_behavior_nh.getParam("red_half_area",red_half_area);
    chassis_behavior_nh.getParam("blue_half_area",blue_half_area);

    blackboard_->set<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones",all_zones);
    blackboard_->set<std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>>>("pos_detection_polygons",pos_detection_polygons);
    blackboard_->set<std::vector<std::string>>("red_half_area",red_half_area);
    blackboard_->set<std::vector<std::string>>("blue_half_area",blue_half_area);
  }

  void load_default_aim_rank()
  {
    ros::NodeHandle auto_nh(bt_nh_,"auto");
    XmlRpc::XmlRpcValue default_aim_priority;
    std::vector<int> src_default_aim_rank;
    auto_nh.getParam("default_aim_priority",default_aim_priority);
    for (int i=0;i<default_aim_priority.size();i++)
    {
      src_default_aim_rank.push_back(default_aim_priority[i]);
    }
    std::vector<uint8_t> default_aim_rank(src_default_aim_rank.begin() , src_default_aim_rank.end()); //做类型转换
    blackboard_->set<std::vector<uint8_t>>("aim_priority",default_aim_rank);
  }

  void load_robot_color()
  {
    std::string color;
    bt_nh_.getParam("robot_color",color);
    ROS_ASSERT(color == "blue" || color == "red");
    blackboard_->set<std::string>("robot_color",color);
  }

  void param_initialization()
  {
    blackboard_->set<ros::Time>("need_avoid_drone_time",ros::Time(0)); //该值用于记录云台手按下避开无人机的按键时的时刻，初始化为0，后续将在subscriber里面进行更新
    blackboard_->set<bool>("need_defense_base",false);
    blackboard_->set<bool>("need_still_gyro",false);
    blackboard_->set<double>("present_time",0.0);
    blackboard_->set<int>("chassis_mode",0);
    blackboard_->set<bool>("has_revived",false);
    blackboard_->set<bool>("need_supply",false);
    blackboard_->set<bool>("has_calibrated_barrel",false);
    blackboard_->set<bool>("enable_fight",false);
    blackboard_->set<bool>("enable_hole_up",false);
    blackboard_->set<bool>("need_enable_fight",false);
    blackboard_->set<bool>("ignore_buff",false);
    blackboard_->set<ros::Time>("last_track_time",ros::Time::now());
    blackboard_->set<int>("gimbal_mode",0);
    blackboard_->set<int>("circle_count",0); //该参数在gimbal_action_node中被调用
    blackboard_->set<double>("last_yaw",0.0); //该参数在gimbal_action_node中被调用
    blackboard_->set<bool>("has_determined_goal",false);
    blackboard_->set<bool>("has_reached_goal",false);
    blackboard_->set<ros::Time>("reach_time",ros::Time::now());
    blackboard_->set<int>("sentry_intention",static_cast<int>(types::SentryIntention::MoveToTheTargetPoint));
    blackboard_->set<bool>("is_need_aim_outpost",false);
    blackboard_->set<bool>("is_need_aim_base",false);
  }

  void loadMapParam()
  {
    std::vector<double> minimap2map;
    bt_nh_.getParam("minimap2map",minimap2map);
    blackboard_->set<std::vector<double>>("minimap2map",minimap2map);

    ros::NodeHandle pose_sanity_nh(bt_nh_,"pose_sanity");
    std::vector<double> map_bounds;
    ROS_ASSERT(pose_sanity_nh.getParam("map_bounds",map_bounds) == true);
    blackboard_->set<std::vector<double>>("map_bounds",map_bounds);
  }


private:
  ros::NodeHandle bt_nh_;
  BT::Blackboard::Ptr blackboard_;
  std::vector<geometry_msgs::PointStamped> region_key_points_;
};

#endif //NEW_BEHAVIOR_TREE_SENTRY_PARAM_LOADER_H