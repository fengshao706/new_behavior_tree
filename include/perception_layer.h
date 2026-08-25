//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H
#define NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H

#include <any>
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
#include "visualization_msgs/Marker.h"
#include "rm_msgs/ShootCmd.h"
#include <rm_msgs/ShootBeforehandCmd.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/PowerManagementSampleAndStatusData.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs//Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "common/tools.h"

namespace tools
{
  class CmdTools;
  class NavigationTools;
  class MiniMapTools;
  class GimbalTools;
}

namespace perception{

  class Subscriber
  {
  public:
    enum class TopicId
    {
      TRACK_DATA,
      GAME_ROBOT_STATUS,
      DBUS_DATA,
      SHOOT_CMD,
      POWER_HEAT_DATA,
      CAPACITY_DATA,
      GAME_STATUS,
      ROBOT_HURT_DATA,
      GAME_ROBOT_HP,
      SENTRY_INFO,
      CLIENT_MAP_SEND_DATA,
      EVENT_DATA,
      BULLET_ALLOWANCE,
      GLOBAL_PLANNER_DATA,
      ROBOT_POSITION,
      BUFF_DATA,
      RFID_DATA,
      RADAR_TO_SENTRY_DATA,
      ALLOW_SHOOT,
      SHOOT_CMD_DATA,
      PLANNER_GOAL,
      ODOM_DATA,
      BACK_CAMERA_DETECTION_DATA,
      FRONT_CAMERA_DETECTION_DATA
    };

    template <typename MsgType>
    struct ReturnMsg
    {
      MsgType message;
      ros::Time stamp = ros::Time::now();
    };

    Subscriber(ros::NodeHandle &bt_nh) : bt_nh_(bt_nh)
    {
      register_subscriber<rm_msgs::DbusData>(TopicId::DBUS_DATA,"/rm_ecat_hw/dbus");
      register_subscriber<rm_msgs::GameRobotStatus>(TopicId::GAME_ROBOT_STATUS,"/rm_referee/game_robot_status");
      register_subscriber<rm_msgs::TrackData>(TopicId::TRACK_DATA,"/track");
      register_subscriber<rm_msgs::ShootCmd>(TopicId::SHOOT_CMD,"/controllers/shooter_controller/command");
      register_subscriber<rm_msgs::PowerHeatData>(TopicId::POWER_HEAT_DATA,"/rm_referee/power_heat_data");
      register_subscriber<rm_msgs::PowerManagementSampleAndStatusData>(TopicId::CAPACITY_DATA,"/rm_referee/power_management/sample_and_status");
      register_subscriber<rm_msgs::GameStatus>(TopicId::GAME_STATUS,"/rm_referee/game_status");
      register_subscriber<rm_msgs::RobotHurt>(TopicId::ROBOT_HURT_DATA,"/rm_referee/robot_hurt_data");
      register_subscriber<rm_msgs::GameRobotHp>(TopicId::GAME_ROBOT_HP,"/rm_referee/game_robot_hp");
      register_subscriber<rm_msgs::SentryInfo>(TopicId::SENTRY_INFO,"/rm_referee/sentry_info");
      register_subscriber<rm_msgs::ClientMapSendData>(TopicId::CLIENT_MAP_SEND_DATA,"/rm_referee/client_map_send_data");
      register_subscriber<rm_msgs::EventData>(TopicId::EVENT_DATA,"/rm_referee/event_data");
      register_subscriber<rm_msgs::BulletAllowance>(TopicId::BULLET_ALLOWANCE,"/rm_referee/bullet_allowance_data");
      register_subscriber<nav_msgs::Path>(TopicId::GLOBAL_PLANNER_DATA,"/move_base_flex/GlobalPlanner/plan");
      register_subscriber<rm_msgs::RobotsPositionData>(TopicId::ROBOT_POSITION,"robot_position");
      register_subscriber<rm_msgs::Buff>(TopicId::BUFF_DATA,"/rm_referee/robot_buff");
      register_subscriber<rm_msgs::RfidStatus>(TopicId::RFID_DATA,"/rm_referee/rfid_status_data");
      register_subscriber<rm_msgs::RadarToSentry>(TopicId::RADAR_TO_SENTRY_DATA,"/rm_referee/radar_to_sentry");
      register_subscriber<rm_msgs::ShootBeforehandCmd>(TopicId::ALLOW_SHOOT,"/controllers/gimbal_controller/bullet_solver/shoot_beforehand_cmd");
      register_subscriber<rm_msgs::ShootCmd>(TopicId::SHOOT_CMD_DATA,"/controllers/shooter_controller/command");
      register_subscriber<geometry_msgs::PoseStamped>(TopicId::PLANNER_GOAL,"/move_base_simple/goal");
      register_subscriber<nav_msgs::Odometry>(TopicId::ODOM_DATA,"/odom");
      register_subscriber<rm_msgs::TargetDetectionArray>(TopicId::BACK_CAMERA_DETECTION_DATA,"/detection_back");
      register_subscriber<rm_msgs::TargetDetectionArray>(TopicId::FRONT_CAMERA_DETECTION_DATA,"/detection_front");

      initSubscriber();
    }

    template<typename MsgType>
    ReturnMsg<MsgType> msgGetter(TopicId topic_id)
    {
      ReturnMsg<MsgType> return_msg;
      return_msg.message = std::any_cast<MsgType>(dir_.at(topic_id).message);
      return_msg.stamp = dir_.at(topic_id).stamp;

      return return_msg;
    }

  private:
    struct TopicDetail
    {
      std::string topic_name;
      ros::Time stamp = ros::Time::now();
      std::any message;
      std::function<void()> creator;
    };

    template<typename MsgType>
    void register_subscriber(TopicId id ,const std::string &topic_name)
    {
      TopicDetail detail;
      detail.topic_name = topic_name;
      detail.stamp = ros::Time::now();
      detail.creator = [this,id]()
      {
        subscribers_.push_back(bt_nh_.subscribe<MsgType>(dir_.at(id).topic_name,
          10,
          [this,id](const typename MsgType::ConstPtr &msg){callback<MsgType>(msg,id);}));
      };

      dir_.emplace(id,std::move(detail));
    };

    template<typename MsgType>
    void callback(const typename MsgType::ConstPtr &msg , TopicId topic_id)
    {
      dir_[topic_id].message = *msg;
      dir_[topic_id].stamp = ros::Time::now();
    }

    void initSubscriber()
    {
      for (auto &temp : dir_)
      {
        temp.second.creator();
      }
    }
    std::map<TopicId,TopicDetail> dir_;
    ros::NodeHandle &bt_nh_;
    std::vector<ros::Subscriber> subscribers_;
  };

  class TfAccessor
  {
  public:
    enum class FrameId
    {
      MAP,
      BASE_LINK,
      BACK_CAMERA_OPTICAL_FRAME,
      ODOM,
      YAW,
      PITCH,
      CAMERA_OPTICAL_FRAME,
      TRACK,
      LIVOX
    };

    TfAccessor(ros::NodeHandle &bt_nh , Subscriber &subscriber);

    /**@brief 用于获取tf变换信息
     *@param target_frame target_frame,可查询的frame请查阅FrameId
     *@param source_frame source_frame,可查询的frame请查阅FrameId
     * **/
    geometry_msgs::TransformStamped getTfTransform(const FrameId &target_frame ,const FrameId &source_frame) const;

  private:
    ros::TimerCallback tfUpdateCallback();

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::NodeHandle &bt_nh_;
    Subscriber &subscriber_;
    const std::map<FrameId, std::string> frame_map = {
      {FrameId::MAP, "map"},
      {FrameId::BASE_LINK, "base_link"},
      {FrameId::BACK_CAMERA_OPTICAL_FRAME, "back_camera_optical_frame"},
      {FrameId::ODOM, "odom"},
      {FrameId::YAW, "yaw"},
      {FrameId::PITCH, "pitch"},
      {FrameId::CAMERA_OPTICAL_FRAME, "camera_optical_frame"},
      {FrameId::LIVOX, "livox_frame"}
    };
  };

  class Publisher
  {
  public:
    struct Pubs
    {
      ros::Publisher map_sentry_data_pub_;
      ros::Publisher aim_priority_pub_;
      ros::Publisher sentry_state_pub_;
      ros::Publisher sentry_cmd_pub_;
      ros::Publisher conduct_point_pub_;
      ros::Publisher attacking_target_pub_;
      ros::Publisher marker_pub_;
      ros::Publisher manual_to_referee_pub_;
    };

    struct Msgs
    {
      rm_msgs::SentryCmd sentry_cmd;
    };

    Publisher(ros::NodeHandle& bt_nh);

    Pubs* getPublishers();

    Msgs* getPublishMsgs();

  private:
    std::unique_ptr<Pubs> publishers_;
    std::unique_ptr<Msgs> publish_msgs;
  };

}

#endif //NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H