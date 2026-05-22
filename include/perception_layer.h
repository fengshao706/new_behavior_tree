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

namespace perception{

  class Subscriber
  {
  public:
    explicit Subscriber(tools::CmdTools& cmd_tools, ros::NodeHandle& nh);

    // Thread-safe data access methods
    rm_msgs::TrackData getTrackData() const;

    rm_msgs::GameRobotStatus getGameRobotStatus() const;

    rm_msgs::ShootCmd getShootCmd() const;

    void setTrackData(const rm_msgs::TrackData& data);

    void setGameRobotStatus(const rm_msgs::GameRobotStatus& data);

    bool isRefereeOnline() const;

    void setRefereeOnline(bool online);

    bool hasBackCameraDetected() const;

    geometry_msgs::PointStamped getBackCameraDetection();

    void setBackCameraDetected(bool detected);

    [[nodiscard]]int getBackCameraDetectionId();

    void setBackCameraDetectionId(int id);

    bool hasEngineerMarked() const;

    void setEngineerMarked(bool marked);

    rm_msgs::TargetDetectionArray getFrontCameraDetection();

    rm_msgs::DbusData getDbusData() const;

    rm_msgs::GameStatus getGameStatus() const;

    rm_msgs::GameRobotHp getGameRobotHp() const;

    void setPowerHeatData(const rm_msgs::PowerHeatData &data);

    rm_msgs::PowerHeatData getPowerHeatData() const;

    rm_msgs::ClientMapSendData  getClientMapSendData() const;

    void clearClientMapUpdateState();

    bool isClientMapUpdate() const;

    rm_msgs::EventData getEventData() const;

    rm_msgs::BulletAllowance getBulletAllowance() const;

    rm_msgs::RobotsPositionData getRobotPositionData() const;

    nav_msgs::Path getGlobalPlannerPathData() const;

    bool isGlobalPlannerPathDataUpdate() const;

    nav_msgs::Odometry getOdomData() const;

    rm_msgs::RobotHurt getRobotHurtData() const;

    rm_msgs::Buff getBuffData() const;

    rm_msgs::RfidStatus getRfidStatus() const;

    rm_msgs::SentryInfo getSentryInfoData() const;

    rm_msgs::PowerManagementSampleAndStatusData getPowerManagementSampleAndStatusData_() const;

  private:
    void backCameraDetectionCallback(const rm_msgs::TargetDetectionArray::ConstPtr& data);

    void frontCameraDetectionCallback(const rm_msgs::TargetDetectionArray::ConstPtr& data);

    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data);

    void gimbalDesErrorCallback(
      const rm_msgs::GimbalDesError::ConstPtr& data); // data 是 rm_msgs::GimbalDesError 类型的 智能指针（ConstPtr）;

    void trackCallback(const rm_msgs::TrackData::ConstPtr& data);

    void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data);

    void robotHurtCallback(const rm_msgs::RobotHurt::ConstPtr& data);

    void robotBuffCallback(const rm_msgs::Buff::ConstPtr& data);

    void rfidStatuCallBack(const rm_msgs::RfidStatus::ConstPtr& data);

    void sentryCmdCallBack(const rm_msgs::SentryInfo::ConstPtr& data);

    void ShootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd::ConstPtr& data);

    void shootCommandCallback(const rm_msgs::ShootCmd::ConstPtr& data);

    void robotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data);

    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data);

    void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data);

    void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& data);

    void clientMapSendDataCallback(const rm_msgs::ClientMapSendData::ConstPtr& data);

    void radarToSentryCallback(const rm_msgs::RadarToSentry::ConstPtr& data);

    void eventDataCallback(const rm_msgs::EventData::ConstPtr& data);

    void bulletAllowanceCallback(const rm_msgs::BulletAllowance::ConstPtr& data);

    void robotsPositionCallback(const rm_msgs::RobotsPositionData::ConstPtr& data);

    void globalPlannerCallback(
      const nav_msgs::Path::ConstPtr&
      data); // 接受一个参数 const nav_msgs::Path::ConstPtr& data，该参数是一个指向 nav_msgs::Path 消息的常量指针;

    void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    tools::CmdTools& cmd_tools_;

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
    ros::Subscriber buff_sub_;
    ros::Subscriber sentry_info_sub_;
    ros::Subscriber rfid_statu_sub_;
    ros::Subscriber back_camera_detection_sub_;
    ros::Subscriber front_camera_detection_sub_;
    ros::Subscriber allow_shoot_sub_;
    ros::Subscriber shoot_command_sub_;
    ros::Subscriber radar_to_sentry_sub_;

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
    rm_msgs::Buff buff_;
    rm_msgs::RfidStatus rfid_statu_;
    rm_msgs::PowerHeatData power_heat_data_;
    rm_msgs::PowerManagementSampleAndStatusData power_management_sample_and_status_data_{};
    nav_msgs::Path goal_planner_;
    nav_msgs::Odometry odom_;
    rm_msgs::ShootCmd shoot_cmd_;
    rm_msgs::SentryInfo sentry_info_;
    rm_msgs::TargetDetectionArray front_camera_detection_info_;
    rm_msgs::RadarToSentry radar_to_sentry_info_;
    ros::Time last_map_data_update_{};
    std::atomic<bool> referee_is_online_{false};
    bool client_map_update_{false};
    bool goal_planner_update_{false};
    std::atomic<bool> has_back_camera_detected_{false};
    std::atomic<bool> has_engineer_marked_{false};
    geometry_msgs::PointStamped back_of_camera_;
    int back_camera_detection_id_ = 0;

    // Mutexes for thread-safe access
    mutable std::mutex front_camera_mutex_, dbus_mutex_, game_robot_status_mutex_, track_mutex_, game_status_mutex_, buff_data_mutex_,
                       game_robot_hp_mutex_, power_heat_data_mutex_, client_map_send_data_mutex_, odom_data_mutex_, robot_hurt_data_mutex_,
                       shoot_cmd_mutex_ , event_data_mutex_ , bullet_allowance_mutex_ , robot_position_mutex_ , global_planner_mutex_,
                        rfid_status_mutex_ , sentry_info_data_mutex_ , power_management_sample_and_status_data_mutex_ , back_camera_mutex_ , back_camera_detection_id_mutex_;
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
    };

    Publisher(ros::NodeHandle& bt_nh);

    Pubs* getPublishers();

  private:
    std::unique_ptr<Pubs> publishers_;
  };

}

#endif //NEW_BEHAVIOR_TREE_PERCEPTION_LAYER_H