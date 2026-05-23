#include "perception_layer.h"

namespace perception
{
  Subscriber::Subscriber(tools::CmdTools& cmd_tools, ros::NodeHandle& nh) : cmd_tools_(cmd_tools)
  {
    ros::NodeHandle subscriber_nh;
    dbus_sub_ = subscriber_nh.subscribe<rm_msgs::DbusData>("/rm_ecat_hw/dbus", 10, &Subscriber::dbusCallback, this);
    track_sub_ = subscriber_nh.subscribe<rm_msgs::TrackData>("/track", 10, &Subscriber::trackCallback, this);
    gimbal_des_error_sub_ = subscriber_nh.subscribe<rm_msgs::GimbalDesError>(
      "/controllers/gimbal_controller/error", 10, &Subscriber::gimbalDesErrorCallback,
      this); /*由于 gimbalDesErrorCallback 是 Subscriber 类的成员函数，而 ROS 需要一个 对象实例 来调用该函数，所以
                  this 代表当前 Subscriber 对象实例*/
    game_robot_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotStatus>(
      "/rm_referee/game_robot_status", 10, &Subscriber::gameRobotStatusCallback, this);
    power_heat_data_sub_ = subscriber_nh.subscribe<rm_msgs::PowerHeatData>("/rm_referee/power_heat_data", 10,
                                                                           &Subscriber::powerHeatDataCallback, this);
    capacity_sub_ = subscriber_nh.subscribe<rm_msgs::PowerManagementSampleAndStatusData>(
      "/rm_referee/power_management/sample_and_status", 10, &Subscriber::capacityDataCallback, this);
    game_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameStatus>("/rm_referee/game_status", 10,
                                                                    &Subscriber::gameStatusCallback, this);
    robot_hurt_sub_ = subscriber_nh.subscribe<rm_msgs::RobotHurt>("/rm_referee/robot_hurt_data", 10,
                                                                  &Subscriber::robotHurtCallback, this);
    robot_hp_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotHp>("/rm_referee/game_robot_hp", 10,
                                                                  &Subscriber::robotHpCallback, this);
    sentry_info_sub_ = subscriber_nh.subscribe<rm_msgs::SentryInfo>("/rm_referee/sentry_info", 10,
                                                                    &Subscriber::sentryCmdCallBack, this);
    client_map_send_data_sub_ = subscriber_nh.subscribe<rm_msgs::ClientMapSendData>(
      "/rm_referee/client_map_send_data", 10, &Subscriber::clientMapSendDataCallback, this);
    event_data_sub_ =
      subscriber_nh.subscribe<rm_msgs::EventData>("/rm_referee/event_data", 10, &Subscriber::eventDataCallback, this);
    bullet_allowance_sub_ = subscriber_nh.subscribe<rm_msgs::BulletAllowance>(
      "/rm_referee/bullet_allowance_data", 10, &Subscriber::bulletAllowanceCallback, this);
    global_planner_sub_ = subscriber_nh.subscribe<nav_msgs::Path>("/move_base_flex/GlobalPlanner/plan", 10,
                                                                  &Subscriber::globalPlannerCallback, this);
    robots_position_sub_ = subscriber_nh.subscribe<rm_msgs::RobotsPositionData>(
      "robot_position", 1, &Subscriber::robotsPositionCallback, this);
    buff_sub_ =
      subscriber_nh.subscribe<rm_msgs::Buff>("/rm_referee/robot_buff", 1, &Subscriber::robotBuffCallback, this);
    rfid_statu_sub_ = subscriber_nh.subscribe<rm_msgs::RfidStatus>("/rm_referee/rfid_status_data", 1,
                                                                   &Subscriber::rfidStatuCallBack, this);
    radar_to_sentry_sub_ = nh.subscribe<rm_msgs::RadarToSentry>("/rm_referee/radar_to_sentry", 1,
                                                                &Subscriber::radarToSentryCallback, this);
    allow_shoot_sub_ = subscriber_nh.subscribe<rm_msgs::ShootBeforehandCmd>(
      "/controllers/gimbal_controller/bullet_solver/shoot_beforehand_cmd", 10,
      &Subscriber::ShootBeforehandCmdCallback, this);
    shoot_command_sub_ = subscriber_nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                                    &Subscriber::shootCommandCallback, this);
    // Used to update referee data.
    goal_subscriber =
      nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, &Subscriber::rvizGoalCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &Subscriber::odomCallback, this);
    back_camera_detection_sub_ = nh.subscribe<rm_msgs::TargetDetectionArray>(
      "/detection_back", 10, &Subscriber::backCameraDetectionCallback, this);
    front_camera_detection_sub_ = nh.subscribe<rm_msgs::TargetDetectionArray>(
      "/detection_front", 10, &Subscriber::frontCameraDetectionCallback, this);
    game_robot_status_.remain_hp = 400;
  }

  void Subscriber::setNavigationTools(tools::NavigationTools* nav_tools) {
    navigation_tools_ = nav_tools;
  }

  rm_msgs::TrackData Subscriber::getTrackData() const
  {
    std::lock_guard<std::mutex> lock(track_mutex_);
    return track_data_;
  }

  rm_msgs::GameRobotStatus Subscriber::getGameRobotStatus() const
  {
    std::lock_guard<std::mutex> lock(game_robot_status_mutex_);
    return game_robot_status_;
  }

  rm_msgs::ShootCmd Subscriber::getShootCmd() const
  {
    std::lock_guard<std::mutex> lock(shoot_cmd_mutex_);
    return shoot_cmd_;
  }

  void Subscriber::setTrackData(const rm_msgs::TrackData& data)
  {
    std::lock_guard<std::mutex> lock(track_mutex_);
    track_data_ = data;
  }

  void Subscriber::setGameRobotStatus(const rm_msgs::GameRobotStatus& data)
  {
    std::lock_guard<std::mutex> lock(game_robot_status_mutex_);
    game_robot_status_ = data;
  }

  bool Subscriber::isRefereeOnline() const
  {
    return referee_is_online_.load(std::memory_order_acquire);
  }

  void Subscriber::setRefereeOnline(bool online)
  {
    referee_is_online_.store(online, std::memory_order_release);
  }

  bool Subscriber::hasBackCameraDetected() const
  {
    return has_back_camera_detected_.load(std::memory_order_acquire);
  }

  geometry_msgs::PointStamped Subscriber::getBackCameraDetection()
  {
    std::lock_guard<std::mutex> lock(back_camera_mutex_);
    return back_of_camera_;
  }

  void Subscriber::setBackCameraDetected(bool detected)
  {
    has_back_camera_detected_.store(detected, std::memory_order_release);
  }

  [[nodiscard]]int Subscriber::getBackCameraDetectionId()
  {
    std::lock_guard<std::mutex> lock(back_camera_detection_id_mutex_);
    return back_camera_detection_id_;
  }

  bool Subscriber::hasEngineerMarked() const
  {
    return has_engineer_marked_.load(std::memory_order_acquire);
  }

  void Subscriber::setEngineerMarked(bool marked)
  {
    has_engineer_marked_.store(marked, std::memory_order_release);
  }

  rm_msgs::TargetDetectionArray Subscriber::getFrontCameraDetection()
  {
    std::lock_guard<std::mutex> lock(front_camera_mutex_);
    return front_camera_detection_info_;
  }

  rm_msgs::DbusData Subscriber::getDbusData() const
  {
    std::lock_guard<std::mutex> lock(dbus_mutex_);
    return dbus_;
  }

  rm_msgs::GameStatus Subscriber::getGameStatus() const
  {
    std::lock_guard<std::mutex> lock(game_status_mutex_);
    return game_status_;
  }

  rm_msgs::GameRobotHp Subscriber::getGameRobotHp() const
  {
    std::lock_guard<std::mutex> lock(game_robot_hp_mutex_);
    return game_robot_hp_;
  }

  void Subscriber::setPowerHeatData(const rm_msgs::PowerHeatData& data)
  {
    std::lock_guard<std::mutex> lock(power_heat_data_mutex_);
    power_heat_data_ = data;
  }

  rm_msgs::PowerHeatData Subscriber::getPowerHeatData() const
  {
    std::lock_guard<std::mutex> lock(power_heat_data_mutex_);
    return power_heat_data_;
  }

  rm_msgs::ClientMapSendData Subscriber::getClientMapSendData() const
  {
    std::lock_guard<std::mutex> lock(client_map_send_data_mutex_);
    return client_map_send_data_;
  }

  void Subscriber::clearClientMapUpdateState()
  {
    client_map_update_ = false;
  }

  bool Subscriber::isClientMapUpdate() const
  {
    std::lock_guard<std::mutex> lock(client_map_send_data_mutex_);
    return client_map_update_;
  }

  rm_msgs::EventData Subscriber::getEventData() const
  {
    std::lock_guard<std::mutex> lock(event_data_mutex_);
    return event_data_;
  }

  rm_msgs::BulletAllowance Subscriber::getBulletAllowance() const
  {
    std::lock_guard<std::mutex> lock(bullet_allowance_mutex_);
    return bullet_allowance_;
  }

  rm_msgs::RobotsPositionData Subscriber::getRobotPositionData() const
  {
    std::lock_guard<std::mutex> lock(robot_position_mutex_);
    return robots_position_;
  }

  nav_msgs::Path Subscriber::getGlobalPlannerPathData() const
  {
    std::lock_guard<std::mutex> lock(global_planner_mutex_);
    return goal_planner_;
  }

  bool Subscriber::isGlobalPlannerPathDataUpdate() const
  {
    std::lock_guard<std::mutex> lock(global_planner_mutex_);
    return goal_planner_update_;
  }

  nav_msgs::Odometry Subscriber::getOdomData() const
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    return odom_;
  }

  rm_msgs::RobotHurt Subscriber::getRobotHurtData() const
  {
    std::lock_guard<std::mutex> lock(robot_hurt_data_mutex_);
    return robot_hurt_msgs_;
  }

  rm_msgs::Buff Subscriber::getBuffData() const
  {
    std::lock_guard<std::mutex> lock(buff_data_mutex_);
    return buff_;
  }

  rm_msgs::RfidStatus Subscriber::getRfidStatus() const
  {
    std::lock_guard<std::mutex> lock(rfid_status_mutex_);
    return rfid_statu_;
  }

  rm_msgs::SentryInfo Subscriber::getSentryInfoData() const
  {
    std::lock_guard<std::mutex> lock(sentry_info_data_mutex_);
    return sentry_info_;
  }

  void Subscriber::backCameraDetectionCallback(const rm_msgs::TargetDetectionArray::ConstPtr& data)
  {
    if (!data->detections.empty() && !hasBackCameraDetected())
    {
      setBackCameraDetected(true);
      std::lock_guard<std::mutex> lock(back_camera_mutex_);
      setBackCameraDetectionId(data->detections[0].id);
      back_of_camera_.header.frame_id = "back_camera_optical_frame";
      back_of_camera_.point.x = data->detections[0].pose.position.x;
      back_of_camera_.point.y = data->detections[0].pose.position.y;
      back_of_camera_.point.z = data->detections[0].pose.position.z;
    }
  }

  void Subscriber::frontCameraDetectionCallback(const rm_msgs::TargetDetectionArray::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(front_camera_mutex_);
    front_camera_detection_info_ = *data;
  }

  void Subscriber::dbusCallback(const rm_msgs::DbusData::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(dbus_mutex_);
    dbus_ = *data;
    cmd_tools_.getSenders()->chassis_command_sender_->updateRefereeStatus(isRefereeOnline());
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateRefereeStatus(isRefereeOnline());
    cmd_tools_.getSenders()->shooter_command_sender_->updateRefereeStatus(isRefereeOnline());
  }

  void Subscriber::gimbalDesErrorCallback(
    const rm_msgs::GimbalDesError::ConstPtr& data) // data 是 rm_msgs::GimbalDesError 类型的 智能指针（ConstPtr）
  {
    /*当 /controllers/gimbal_controller/error 话题发布 rm_msgs::GimbalDesError 消息时，它会被自动调用。它的作用是
          将云台的误差信息传递给 double_barrel_cmd_sender_，让其更新相应的参数。*/
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateGimbalDesError(
    // *data);  //*data 解引用 ConstPtr，获取实际的 rm_msgs::GimbalDesError 数据
    cmd_tools_.getSenders()->shooter_command_sender_->updateGimbalDesError(*data);
  }

  void Subscriber::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
  {
    setTrackData(*data);
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateTrackData(*data);
    cmd_tools_.getSenders()->shooter_command_sender_->updateTrackData(*data);
  }

  void Subscriber::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(game_status_mutex_);
    cmd_tools_.getSenders()->chassis_command_sender_->updateGameStatus(*data);
    game_status_ = *data;
  }

  void Subscriber::robotHurtCallback(const rm_msgs::RobotHurt::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(robot_hurt_data_mutex_);
    robot_hurt_msgs_ = *data;
  }

  void Subscriber::robotBuffCallback(const rm_msgs::Buff::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(buff_data_mutex_);
    buff_ = *data;
  }

  void Subscriber::rfidStatuCallBack(const rm_msgs::RfidStatus::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(rfid_status_mutex_);
    rfid_statu_ = *data;
  }

  void Subscriber::sentryCmdCallBack(const rm_msgs::SentryInfo::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(sentry_info_data_mutex_);
    sentry_info_ = *data;
  }

  void Subscriber::ShootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd::ConstPtr& data)
  {
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateShootBeforehandCmd(*data);
    cmd_tools_.getSenders()->shooter_command_sender_->updateShootBeforehandCmd(*data);
  }

  void Subscriber::shootCommandCallback(const rm_msgs::ShootCmd::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(shoot_cmd_mutex_);
    shoot_cmd_ = *data;
  }

  void Subscriber::robotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(game_robot_hp_mutex_);
    game_robot_hp_ = *data;
  }

  void Subscriber::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
  {
    setGameRobotStatus(*data);
    cmd_tools_.getSenders()->chassis_command_sender_->updateGameRobotStatus(*data);
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updateGameRobotStatus(*data);
    cmd_tools_.getSenders()->shooter_command_sender_->updateGameRobotStatus(*data);
  }

  void Subscriber::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
  {
    setRefereeOnline(ros::Time::now() - data->stamp < ros::Duration(0.3));
    cmd_tools_.getSenders()->chassis_command_sender_->updatePowerHeatData(*data);
    // cmd_tools_.union_cmd_sender_->double_barrel_cmd_sender_->updatePowerHeatData(*data);
    cmd_tools_.getSenders()->shooter_command_sender_->updatePowerHeatData(*data);
    setPowerHeatData(*data);
  }

  void Subscriber::capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& data)
  {
    cmd_tools_.getSenders()->chassis_command_sender_->updateCapacityData(*data);
    std::lock_guard<std::mutex> lock(power_management_sample_and_status_data_mutex_);
    power_management_sample_and_status_data_ = *data;
  }

  void Subscriber::clientMapSendDataCallback(const rm_msgs::ClientMapSendData::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(client_map_send_data_mutex_);
    if (*data != client_map_send_data_)
    {
      client_map_send_data_ = *data;
      client_map_update_ = true;
    }
  }

  void Subscriber::radarToSentryCallback(const rm_msgs::RadarToSentry::ConstPtr& data)
  {
    radar_to_sentry_info_ = *data;
    if (!hasEngineerMarked() && data->engineer_marked)
      setEngineerMarked(true);
  }

  void Subscriber::eventDataCallback(const rm_msgs::EventData::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(event_data_mutex_);
    event_data_ = *data;
  }

  void Subscriber::bulletAllowanceCallback(const rm_msgs::BulletAllowance::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(bullet_allowance_mutex_);
    bullet_allowance_ = *data;
  }

  void Subscriber::robotsPositionCallback(const rm_msgs::RobotsPositionData::ConstPtr& data)
  {
    std::lock_guard<std::mutex> lock(robot_position_mutex_);
    robots_position_ = *data;
  }

  void Subscriber::globalPlannerCallback(
    const nav_msgs::Path::ConstPtr&
    data) // 接受一个参数 const nav_msgs::Path::ConstPtr& data，该参数是一个指向 nav_msgs::Path 消息的常量指针
  {
    std::lock_guard<std::mutex> lock(global_planner_mutex_);
    goal_planner_ = *data;
    goal_planner_update_ = true;
  }

  void Subscriber::rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
    navigation_tools_->getMbfClient()->sendGoal(mbf_goal);
    //  Debug in rviz
  }

  void Subscriber::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    odom_ = *msg;
  }

  void Subscriber::setBackCameraDetectionId(int id)
  {
    std::lock_guard<std::mutex> lock(back_camera_detection_id_mutex_);
    back_camera_detection_id_ = id;
  }

  TfAccessor::TfAccessor(ros::NodeHandle &bt_nh , Subscriber &subscriber) : tf_listener_(tf_buffer_) , bt_nh_(bt_nh) , subscriber_(subscriber)
  {

  }

  geometry_msgs::TransformStamped TfAccessor::getTfTransform(const FrameId &target_frame,const FrameId &source_frame) const
  {
    ROS_ASSERT(target_frame != FrameId::TRACK);
    if (source_frame != FrameId::TRACK){
      return tf_buffer_.lookupTransform(frame_map.at(target_frame),
                                        frame_map.at(source_frame), ros::Time(0), ros::Duration(0.05));
    }else
    {
      return tf_buffer_.lookupTransform(frame_map.at(target_frame),subscriber_.getTrackData().header.frame_id,ros::Time(0),ros::Duration(0.05));
    }
  }

  Publisher::Publisher(ros::NodeHandle& bt_nh)
  {
    ros::NodeHandle root_nh;
    publishers_ = std::make_unique<Pubs>();

    publishers_->map_sentry_data_pub_ = root_nh.advertise<rm_msgs::MapSentryData>("/map_sentry_data", 10);
    publishers_->marker_pub_ = root_nh.advertise<visualization_msgs::Marker>("/radar_marker", 1);
    publishers_->aim_priority_pub_ = bt_nh.advertise<rm_msgs::PriorityArray>(
      "/armor_processor/priority/priority_arr", 1);
    publishers_->sentry_state_pub_ = bt_nh.advertise<std_msgs::String>("/custom_info", 1);
    publishers_->sentry_cmd_pub_ = bt_nh.advertise<rm_msgs::SentryCmd>("/sentry_cmd", 1);
    publishers_->conduct_point_pub_ = bt_nh.advertise<geometry_msgs::PoseStamped>("/conduct_point_in_map", 1);
    publishers_->attacking_target_pub_ = bt_nh.advertise<rm_msgs::SentryAttackingTarget>(
      "/sentry_target_to_referee", 1);
    publishers_->manual_to_referee_pub_ = bt_nh.advertise<rm_msgs::ManualToReferee>("/manual_to_referee", 1);
  }

  Publisher::Pubs* Publisher::getPublishers()
  {
    return publishers_.get();
  }
}
