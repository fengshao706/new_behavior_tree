//
// Created by Reid on 2026/3/5.
//
#include "ros/ros.h"
#include "behaviortree_cpp/blackboard.h"
#include "XmlRpcValue.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "common/types.h"

class SentryParamLoader  // TODO : 需在main函数中构造
{
public:
  SentryParamLoader(ros::NodeHandle &nh , BT::Blackboard::Ptr & blackboard) : nh(nh) , blackboard_(blackboard)
  {
    load_robot_color();
    chassis_behavior_param_load();
    planner_param_load();
    load_zone_configs();
    param_initialization();
  }

  void chassis_behavior_param_load()
  {
    ros::NodeHandle chassis_behavior_nh=ros::NodeHandle(nh,"chassis_behavior");
    ros::NodeHandle auto_nh(nh,"auto");
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

    XmlRpc::XmlRpcValue chase_restricted_zones;
    chassis_behavior_nh.getParam("trigger_blood_return_hp", trigger_blood_return_hp);
    chassis_behavior_nh.getParam("trigger_blood_return_hp_without_buff", trigger_blood_return_hp_without_buff);
    chassis_behavior_nh.getParam("trigger_run_away_outpost_hp", trigger_run_away_outpost_hp);//离开前哨站的血量阈值
    chassis_behavior_nh.getParam("trigger_ban_chase_outpost_hp", trigger_ban_chase_outpost_hp);//禁止追击敌人到前哨站区域的血量阈值
    chassis_behavior_nh.getParam("chase_restricted_zones", chase_restricted_zones);//限制追击的区域
    chassis_behavior_nh.getParam("max_planning_period",max_planning_period);
    chassis_behavior_nh.getParam("stand_at_conduct_point_sec",stand_at_conduct_point_sec);
    chassis_behavior_nh.getParam("chase_freq",chase_freq);
    chassis_behavior_nh.getParam("chase_distance",chase_distance);
    chassis_behavior_nh.getParam("chase_tolerance",chase_tolerance);

    auto_nh.getParam("avoid_drone_time",avoid_drone_time);
    auto_nh.getParam("game_total_time",game_total_time);
    auto_nh.getParam("chasing_max_for_time",chasing_max_for_time);
    auto_nh.getParam("attack_engineer_enable",attack_engineer_enable);
    auto_nh.getParam("attack_outpost_enable",attack_outpost_enable);

    blackboard_->set<int>("trigger_blood_return_hp",trigger_blood_return_hp);
    blackboard_->set<int>("trigger_blood_return_hp_without_buff", trigger_blood_return_hp_without_buff);
    blackboard_->set<int>("trigger_run_away_outpost_hp", trigger_run_away_outpost_hp);
    blackboard_->set<int>("trigger_ban_chase_outpost_hp", trigger_ban_chase_outpost_hp);
    blackboard_->set<XmlRpc::XmlRpcValue>("chase_restricted_zones",chase_restricted_zones);
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
  }

  void chassis_vel_param_load() // TODO : 当前未完成全部参数加载任务
  {
    ros::NodeHandle chassis_vel_nh=ros::NodeHandle(nh,"vel");
    double slow_gyro_vel = chassis_vel_nh.param("slow_gyro_vel",0.5);

    blackboard_->set<double>("still_gyro_vel",slow_gyro_vel);
  }

  void planner_param_load()
  {
    ros::NodeHandle planner_nh=ros::NodeHandle(nh,"planner");
    double default_limit_vel;
    double slope_side_window;
    double default_side_window;
    int neutral_cost;
    int lethal_cost;
    planner_nh.getParam("default_limit_vel", default_limit_vel);
    planner_nh.getParam("slope_side_window", slope_side_window);//坡道附近的代价膨胀值
    planner_nh.getParam("default_side_window", default_side_window);//普通路段的代价膨胀值
    planner_nh.getParam("neutral_cost", neutral_cost);//可通过区域的代价
    planner_nh.getParam("lethal_cost", lethal_cost);//不可通过区域的代价
    blackboard_->set<double>("default_limit_vel",default_limit_vel);
    blackboard_->set<double>("slope_side_window",slope_side_window);
    blackboard_->set<double>("default_side_window",default_side_window);
    blackboard_->set<int>("neutral_cost",neutral_cost);
    blackboard_->set<int>("lethal_cost",lethal_cost);
  }

  void get_region_key_points() //用于下面的多边形
  {
    ros::NodeHandle auto_nh(nh,"auto");
    XmlRpc::XmlRpcValue region_key_points;
    auto_nh.getParam("region_key_points",region_key_points);
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
    ros::NodeHandle auto_nh(nh,"auto");
    ros::NodeHandle chassis_behavior_nh(nh,"chassis_behavior");
    XmlRpc::XmlRpcValue zones;
    auto_nh.getParam("zones",zones);
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones; //所有的区域的所有坐标
    std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>> pos_detection_polygons;

    for (const auto& zone : zones)
    {
      ROS_ASSERT(zone.second.hasMember("position") and zone.second.hasMember("aim_direct") and
                 zone.second.hasMember("pos_detection_polygon"));
      ROS_ASSERT(zone.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray and
                 zone.second["aim_direct"].getType() == XmlRpc::XmlRpcValue::TypeArray and
                 zone.second["pos_detection_polygon"].getType() == XmlRpc::XmlRpcValue::TypeArray); // TODO : 将其放入gtest中
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

    XmlRpc::XmlRpcValue chase_restricted_zone_params;
    std::unordered_map<std::string,types::CHASE_JUDGE> chase_restricted_zones;
    auto_nh.getParam("chase_restricted_zones",chase_restricted_zone_params);
    for (const auto & zone_params : chase_restricted_zone_params)
    {
      std::vector<std::string> degree_restriction_to_each_region;
      for (int i=0;i<zone_params.second["areas"].size();i++)
      {
        degree_restriction_to_each_region.push_back(zone_params.second["areas"][i]);
      }
      types::CHASE_JUDGE chase_judge;
      chase_judge.chase_restricted_zone=degree_restriction_to_each_region;
      chase_judge.outpost_hp_threshold=zone_params.second["outpost_hp_threshold"];
      chase_restricted_zones.insert(std::make_pair(zone_params.first,chase_judge));
    }

    //----------------------------------------------------------------------
    std::vector<std::string> red_half_area;
    std::vector<std::string> blue_half_area;
    chassis_behavior_nh.getParam("red_half_area",red_half_area);
    chassis_behavior_nh.getParam("blue_half_area",blue_half_area);

    blackboard_->set<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones",all_zones);
    blackboard_->set<std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>>>("pos_detection_polygons",pos_detection_polygons);
    blackboard_->set<std::unordered_map<std::string,types::CHASE_JUDGE>>("chase_restricted_zones",chase_restricted_zones);
    blackboard_->set<std::vector<std::string>>("red_half_area",red_half_area);
    blackboard_->set<std::vector<std::string>>("blue_half_area",blue_half_area);
  }

  void load_default_aim_rank()
  {
    ros::NodeHandle auto_nh(nh,"auto");
    XmlRpc::XmlRpcValue default_aim_priority;
    std::vector<int> default_aim_rank;
    auto_nh.getParam("default_aim_priority",default_aim_priority); //TODO : 这里需要使用gtest测试确保类型为数组且=内部的数据为整数
    for (int i=0;i<default_aim_priority.size();i++)
    {
      default_aim_rank.push_back(default_aim_priority[i]);
    }
    blackboard_->set<std::vector<int>>("default_aim_rank",default_aim_rank);
  }

  void load_robot_color()
  {
    std::string color;
    nh.getParam("color",color);
    blackboard_->set<std::string>("robot_color",color);
  }

  void param_initialization()
  {
    blackboard_->set<ros::Time>("need_avoid_drone_time",ros::Time(0)); //该值用于记录云台手按下避开无人机的按键时的时刻，初始化为0，后续将在subscriber里面进行更新
    blackboard_->set<bool>("need_defense_base",false);
    blackboard_->set<bool>("need_still_gyro",false);
    blackboard_->set<double>("present_time",0.0);
    blackboard_->set<types::ChassisMode>("chassis_mode",types::ChassisMode::ChassisSlowGyro);
    blackboard_->set<bool>("has_revived",false);
    blackboard_->set<bool>("need_supply",false);
    blackboard_->set<bool>("has_calibrated_barrel",false);
    blackboard_->set<bool>("enable_fight",false);
    blackboard_->set<bool>("enable_hole_up",false);
    blackboard_->set<bool>("need_enable_fight",false);
    blackboard_->set<bool>("ignore_buff",false);
  }


private:
  ros::NodeHandle nh;
  BT::Blackboard::Ptr blackboard_;
  std::vector<geometry_msgs::PointStamped> region_key_points_;
};