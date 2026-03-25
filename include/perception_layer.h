//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H
#define NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H

#include <rm_msgs/DbusData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/RobotHurt.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/ManualToReferee.h>
#include <rm_msgs/MapSentryData.h>
#include <rm_msgs/EventData.h>
#include <rm_msgs/BulletAllowance.h>
#include <rm_msgs/RobotsPositionData.h>
#include "rm_msgs/Buff.h"
#include "rm_msgs/RfidStatus.h"
#include "rm_msgs/TargetDetectionArray.h"
#include "rm_msgs/SentryInfo.h"
#include "rm_msgs/SentryCmd.h"
#include "rm_msgs/SentryAttackingTarget.h"
#include "rm_msgs/ClientMapSendData.h"

#include "dynamic_reconfigure/client.h"
#include <rm_msgs/ShootState.h>
#include "rm_msgs/RadarToSentry.h"
#include "rm_msgs/DartRemainingTime.h"
#include <mbf_msgs/MoveBaseAction.h>
#include "common/tools.h"
#include "visualization_msgs/Marker.h"

namespace perception
{
  class Subscriber
  {
  public:
    explicit Subscriber(tools::CmdTools& cmd_tools, ros::NodeHandle& nh , BT::Blackboard & blackboard) : cmd_tools_(cmd_tools) , blackboard_(blackboard)
    {
      ros::NodeHandle subscriber_nh;
      map_sentry_data_pub_ = subscriber_nh.advertise<rm_msgs::MapSentryData>("/map_sentry_data", 10);
      marker_pub_ = subscriber_nh.advertise<visualization_msgs::Marker>("/radar_marker", 1);
      dbus_sub_ = subscriber_nh.subscribe<rm_msgs::DbusData>("/rm_ecat_hw/dbus", 10, &Subscriber::dbusCallback,
                                                             this);
      track_sub_ = subscriber_nh.subscribe<rm_msgs::TrackData>("/track", 10, &Subscriber::trackCallback, this);
      gimbal_des_error_sub_ = subscriber_nh.subscribe<rm_msgs::GimbalDesError>(
        "/controllers/gimbal_controller/error", 10,
        &Subscriber::gimbalDesErrorCallback, this);
      game_robot_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotStatus>(
        "/rm_referee/game_robot_status", 10, &Subscriber::gameRobotStatusCallback, this);
      power_heat_data_sub_ = subscriber_nh.subscribe<rm_msgs::PowerHeatData>("/rm_referee/power_heat_data", 10,
                                                                             &Subscriber::powerHeatDataCallback, this);
      game_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameStatus>("/rm_referee/game_status", 10,
                                                                      &Subscriber::gameStatusCallback, this);
      robot_hurt_sub_ = subscriber_nh.subscribe<rm_msgs::RobotHurt>("/rm_referee/robot_hurt_data", 10,
                                                                    &Subscriber::robotHurtCallback, this);
      robot_hp_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotHp>("/rm_referee/game_robot_hp", 10,
                                                                    &Subscriber::robotHpCallback, this);
      sentry_info_sub_ = subscriber_nh.subscribe<rm_msgs::SentryInfo>(
        "/rm_referee/sentry_info", 10, &Subscriber::sentryCmdCallBack, this);
      client_map_send_data_sub_ = subscriber_nh.subscribe<rm_msgs::ClientMapSendData>(
        "/rm_referee/client_map_send_data", 10, &Subscriber::clientMapSendDataCallback, this);
      event_data_sub_ =
        subscriber_nh.subscribe<rm_msgs::EventData>("/rm_referee/event_data", 10,
                                                    &Subscriber::eventDataCallback, this);
      bullet_allowance_sub_ = subscriber_nh.subscribe<rm_msgs::BulletAllowance>(
        "/rm_referee/bullet_allowance_data", 10, &Subscriber::bulletAllowanceCallback, this);
      global_planner_sub_ = subscriber_nh.subscribe<nav_msgs::Path>("/move_base_flex/GlobalPlanner/plan", 10,
                                                                    &Subscriber::globalPlannerCallback, this);
      robots_position_sub_ = subscriber_nh.subscribe<rm_msgs::RobotsPositionData>(
        "robot_position", 1, &Subscriber::robotsPositionCallback, this);
      buff_sub_ =
        subscriber_nh.subscribe<rm_msgs::Buff>("/rm_referee/robot_buff", 1, &Subscriber::robotBuffCallback,
                                               this);
      dart_info_sub_ = subscriber_nh.subscribe<rm_msgs::DartRemainingTime>(
        "/rm_referee/dart_remaining_time_data", 1,
        &Subscriber::dartCallBack, this);
      rfid_statu_sub_ = subscriber_nh.subscribe<rm_msgs::RfidStatus>("/rm_referee/rfid_status_data", 1,
                                                                     &Subscriber::rfidStatuCallBack, this);
      radar_to_sentry_sub_ = nh.subscribe<rm_msgs::RadarToSentry>("/rm_referee/radar_to_sentry", 1,
                                                                  &Subscriber::radarToSentryCallback, this);
      allow_shoot_sub_ = subscriber_nh.subscribe<rm_msgs::ShootBeforehandCmd>(
        "/controllers/gimbal_controller/bullet_solver/shoot_beforehand_cmd", 10,
        &Subscriber::ShootBeforehandCmdCallback, this);
      shoot_command_sub_ = subscriber_nh.subscribe<rm_msgs::ShootCmd>(
        "/controllers/shooter_controller/command", 10,
        &Subscriber::shootCommandCallback, this);
      // Used to update referee data.
      goal_subscriber =
        nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, &Subscriber::rvizGoalCallback,
                                                 this);
      odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &Subscriber::odomCallback, this);
      back_camera_detection_sub_ = nh.subscribe<rm_msgs::TargetDetectionArray>(
        "/detection_back", 10, &Subscriber::backCameraDetectionCallback, this);
      aim_priority_pub_ = nh.advertise<rm_msgs::PriorityArray>("/armor_processor/priority/priority_arr", 1);
      sentry_state_pub_ = nh.advertise<std_msgs::String>("/custom_info", 1);
      sentry_cmd_pub_ = nh.advertise<rm_msgs::SentryCmd>("/sentry_cmd", 1);
      conduct_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/conduct_point_in_map", 1);
      attacking_target_pub_ = nh.advertise<rm_msgs::SentryAttackingTarget>("/sentry_target_to_referee", 1);
      game_robot_status_.remain_hp = 400;
    }

    rm_msgs::DbusData dbus_;
    rm_msgs::RobotHurt robot_hurt_msgs_{};
    rm_msgs::TrackData track_data_;
    rm_msgs::GameRobotStatus game_robot_status_;
    rm_msgs::GameRobotHp game_robot_hp_{};
    rm_msgs::GameStatus game_status_;
    rm_msgs::ClientMapSendData client_map_send_data_;
    rm_msgs::EventData event_data_;
    rm_msgs::BulletAllowance bullet_allowance_;
    rm_msgs::RobotsPositionData robots_position_;
    rm_msgs::DartRemainingTime dart_info_;
    rm_msgs::Buff buff_;
    rm_msgs::RfidStatus rfid_statu_;
    rm_msgs::PowerHeatData power_heat_data_;
    nav_msgs::Path goal_planner_;
    nav_msgs::Odometry odom_;
    rm_msgs::ShootCmd shoot_cmd_;
    rm_msgs::SentryInfo sentry_info_;
    rm_msgs::RadarToSentry radar_to_sentry_info_;

    ros::Publisher map_sentry_data_pub_;
    ros::Publisher aim_priority_pub_;
    ros::Publisher sentry_state_pub_;
    ros::Publisher sentry_cmd_pub_;
    ros::Publisher conduct_point_pub_;
    ros::Publisher attacking_target_pub_;
    ros::Publisher marker_pub_;
    ros::Time last_map_data_update_{};
    bool referee_is_online_{false};
    bool client_map_update_{false};
    bool goal_planner_update_{false};
    bool has_back_camera_detected_{false}, has_engineer_marked_{false};
    geometry_msgs::PointStamped back_of_camera_;
    int back_camera_detection_id_ = 0;

  private:
    void backCameraDetectionCallback(const rm_msgs::TargetDetectionArray::ConstPtr& data)
    {
      if (!data->detections.empty() && !has_back_camera_detected_)
      {
        has_back_camera_detected_ = true;
        back_camera_detection_id_ = data->detections[0].id;
        back_of_camera_.header.frame_id = "back_camera_optical_frame";
        back_of_camera_.point.x = data->detections[0].pose.position.x;
        back_of_camera_.point.y = data->detections[0].pose.position.y;
        back_of_camera_.point.z = data->detections[0].pose.position.z;
      }
    }

    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data)
    {
      dbus_ = *data;
      cmd_tools_.chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateRefereeStatus(referee_is_online_);
    }

    void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
    {
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateGimbalDesError(*data);
    }

    void trackCallback(const rm_msgs::TrackData::ConstPtr& data)
    {
      track_data_ = *data;
      blackboard_.set<rm_msgs::TrackData>("track_data",track_data_);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateTrackData(*data);
    }

    void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
    {
      double game_total_time;
      try
      {
        game_total_time = blackboard_.get<double>("game_total_time");
      }catch (BT::RuntimeError &e)
      {
        ROS_ERROR("BT can not access key name [game_total_time] , default value is 420.0");
        game_total_time = 420.0;
      }
      cmd_tools_.chassis_cmd_sender_->updateGameStatus(*data);
      game_status_ = *data;
      double present_time;
      present_time = game_total_time - game_status_.stage_remain_time;;
      blackboard_.set<double>("present_time",present_time);
    }

    void robotHurtCallback(const rm_msgs::RobotHurt::ConstPtr& data)
    {
      robot_hurt_msgs_ = *data;
    }

    void robotBuffCallback(const rm_msgs::Buff::ConstPtr& data)
    {
      buff_ = *data;
      int defense_buff = buff_.defence_buff;
      blackboard_.set<int>("defense_buff",defense_buff);
    }

    void dartCallBack(const rm_msgs::DartRemainingTime::ConstPtr& data)
    {
      dart_info_ = *data;
    }

    void rfidStatuCallBack(const rm_msgs::RfidStatus::ConstPtr& data)
    {
      rfid_statu_ = *data;
    }

    void sentryCmdCallBack(const rm_msgs::SentryInfo::ConstPtr& data)
    {
      sentry_info_ = *data;
    }

    void ShootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd::ConstPtr& data)
    {
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateShootBeforehandCmd(*data);
    }

    void shootCommandCallback(const rm_msgs::ShootCmd::ConstPtr& data)
    {
      shoot_cmd_ = *data;
    }

    void robotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
    {
      game_robot_hp_ = *data;
      blackboard_.set<rm_msgs::GameRobotHp>("game_robot_hp",game_robot_hp_);
    }

    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
    {
      game_robot_status_ = *data;
      cmd_tools_.chassis_cmd_sender_->updateGameRobotStatus(*data);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateGameRobotStatus(*data);
    }

    void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
    {
      referee_is_online_ = (ros::Time::now() - data->stamp < ros::Duration(0.3));
      cmd_tools_.chassis_cmd_sender_->updatePowerHeatData(*data);
      cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updatePowerHeatData(*data);
      power_heat_data_ = *data;
    }

    void clientMapSendDataCallback(const rm_msgs::ClientMapSendData::ConstPtr& data)
    {
      if (*data != client_map_send_data_)
      {
        client_map_send_data_ = *data;
        ros::Time need_avoid_drone_time;
        switch (client_map_send_data_.command_keyboard)
        {
        case rm_msgs::ClientMapSendData::KEY_D:
          {
            need_avoid_drone_time = ros::Time::now();
            blackboard_.set<ros::Time>("need_avoid_drone_time",need_avoid_drone_time);
            break;
          }
        case rm_msgs::ClientMapSendData::KEY_H:
          {
            bool need_defense_base = blackboard_.get<bool>("need_defense_base");
            need_defense_base = !need_defense_base;
            blackboard_.set<bool>("need_defense_base",need_defense_base);
            break;
          }
        case rm_msgs::ClientMapSendData::KEY_G:
          {
            bool need_still_gyro = blackboard_.get<bool>("need_still_gyro");
            need_still_gyro = !need_still_gyro;
            blackboard_.set<bool>("need_still_gyro",need_still_gyro);
            break;
          }
        default:
          break;
          client_map_update_ = true;
        }
      }
    }

    void radarToSentryCallback(const rm_msgs::RadarToSentry::ConstPtr& data)
    {
      radar_to_sentry_info_ = *data;
      if (!has_engineer_marked_ && data->engineer_marked)
        has_engineer_marked_ = true;
    }

    void eventDataCallback(const rm_msgs::EventData::ConstPtr& data)
    {
      event_data_ = *data;
    }

    void bulletAllowanceCallback(const rm_msgs::BulletAllowance::ConstPtr& data)
    {
      bullet_allowance_ = *data;
    }

    void robotsPositionCallback(const rm_msgs::RobotsPositionData::ConstPtr& data)
    {
      robots_position_ = *data;
    }

    void globalPlannerCallback(const nav_msgs::Path::ConstPtr& data)
    {
      goal_planner_ = *data;
      goal_planner_update_ = true;
    }

    void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      geometry_msgs::PoseStamped goal = *msg;
      goal.pose.orientation.w = 1.0;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;
      goal.header.frame_id = "map";

      mbf_msgs::MoveBaseGoal mbf_goal;
      mbf_goal.target_pose = goal;
      //    cmd_tools_.mbf_client_->waitForServer();
      cmd_tools_.mbf_client_->sendGoal(mbf_goal);
      //  Debug in rviz
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      odom_ = *msg;
    }

    tools::CmdTools& cmd_tools_;
    BT::Blackboard & blackboard_;

    ros::Subscriber dbus_sub_;
    ros::Subscriber track_sub_;
    ros::Subscriber robot_hp_sub_;
    ros::Subscriber game_robot_status_sub_, game_status_sub_, robot_hurt_sub_;
    ros::Subscriber gimbal_des_error_sub_;
    ros::Subscriber power_heat_data_sub_, capacity_sub_;
    ros::Subscriber client_map_send_data_sub_;
    ros::Subscriber global_planner_sub_;
    ros::Subscriber goal_subscriber;
    ros::Subscriber points_subscriber;
    ros::Subscriber odom_sub_;
    ros::Subscriber event_data_sub_;
    ros::Subscriber bullet_allowance_sub_;
    ros::Subscriber robots_position_sub_;
    ros::Subscriber dart_info_sub_;
    ros::Subscriber buff_sub_;
    ros::Subscriber sentry_info_sub_;
    ros::Subscriber rfid_statu_sub_;
    ros::Subscriber back_camera_detection_sub_;
    ros::Subscriber allow_shoot_sub_;
    ros::Subscriber shoot_command_sub_;
    ros::Subscriber radar_to_sentry_sub_;
  };

  class Perception
  {
  public:
    Perception(BT::Blackboard &blackboard) : blackboard_(blackboard)
    {
      ros::NodeHandle auto_nh(nh,"auto");
      auto_nh.getParam("game_total_time",game_total_time);
    }

    double get_present_time()
    {
      double present_time;
      present_time = game_total_time-subscriber_->game_status_.stage_remain_time;
      blackboard_.set<double>("present_time",present_time);
      return present_time;
    }


  private:
    ros::NodeHandle nh;
    Subscriber * subscriber_;
    double game_total_time;
    BT::Blackboard & blackboard_;
  };
}

#endif //NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H
