//
// Created by root on 2026/5/10.
//
#include "common/tools.h"

namespace tools
{
  CmdTools::CmdTools(ros::NodeHandle& nh, BT::Blackboard& blackboard) : tf_listener_(tf_buffer_),
                                                                        blackboard_(blackboard)
  {
    senders_ = std::make_unique<Senders>();
    ros::NodeHandle chassis_nh(nh, "chassis");
    senders_->chassis_command_sender_ = std::make_unique<rm_common::ChassisCommandSender>(chassis_nh);
    ros::NodeHandle vel_nh(nh, "vel");
    senders_->vel_2d_command_sender_ = std::make_unique<rm_common::Vel2DCommandSender>(vel_nh);
    ros::NodeHandle base_gimbal_nh(nh, "base_gimbal");
    senders_->base_gimbal_command_sender_ = std::make_unique<rm_common::GimbalCommandSender>(base_gimbal_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    senders_->gimbal_command_sender_ = std::make_unique<rm_common::GimbalCommandSender>(gimbal_nh);
    ros::NodeHandle shooter_nh(nh, "switcher");
    senders_->shooter_command_sender_ = std::make_unique<rm_common::ShooterCommandSender>(shooter_nh);
    dClient_ = std::make_unique<dynamic_reconfigure::Client<global_planner::GlobalPlannerConfig>>("/move_base_flex/GlobalPlanner");

    ros::NodeHandle yaw_nh(nh, "yaw");
    yaw_nh.getParam("acc", yaw_acc_);
    ros::NodeHandle pitch_nh(nh, "pitch");
    pitch_nh.getParam("acc", pitch_acc_);
    ramp_yaw_ = new RampFilter<double>(yaw_acc_, 0.01);
    ramp_pitch_ = new RampFilter<double>(pitch_acc_, 0.01);
    if (!yaw_pid_.init(ros::NodeHandle(yaw_nh, "pid")))
      ROS_WARN("yaw pid has not define.");
  }

  CmdTools::Senders* CmdTools::getSenders() const
  {
    return senders_.get();
  }

  auto CmdTools::getDClient() const
  {
    return dClient_.get();
  }

  double CmdTools::getYawDirect() const
  {
    return yaw_direct_;
  }

  void CmdTools::setYawDirect(double yaw_direct)
  {
    yaw_direct_ = yaw_direct;
  }

  void CmdTools::yawPidCompute(const double angle)
  {
    double cmd = yaw_pid_.computeCommand(angle, ros::Duration(0.01));
    /*if (cmd > 0.5)
      cmd = std::copysign(0.5, cmd);
    if (cmd < -0.5)
      cmd = std::copysign(-0.5, cmd);*/
    yaw_direct_ = smoothlyYawOutput(cmd);
  }

  double CmdTools::smoothlyYawOutput(const double cmd)
  {
    ramp_yaw_->setAcc(yaw_acc_);
    ramp_yaw_->input(cmd);
    return ramp_yaw_->output();
  }

  double CmdTools::smoothlyPitchOutput(const double cmd)
  {
    ramp_pitch_->setAcc(pitch_acc_);
    ramp_pitch_->input(cmd);
    return ramp_pitch_->output();
  }

  void CmdTools::setGlobalPlannerParam(int lethal_cost, int neutral_cost)
  {
    if (last_lethal_cost_ != lethal_cost || last_neutral_cost_ != neutral_cost)
    {
      global_planner::GlobalPlannerConfig config;
      dClient_->getCurrentConfiguration(config);
      ROS_INFO_STREAM("cur neutral cost: " << config.neutral_cost << " target neutral_cost:" << neutral_cost
        << " cur lethal cost:" << config.lethal_cost
        << " target lethal cost:" << lethal_cost);
      config.neutral_cost = neutral_cost;
      config.lethal_cost = lethal_cost;
      last_neutral_cost_ = neutral_cost;
      last_lethal_cost_ = lethal_cost;
      dClient_->setConfiguration(config);
    }
  }

  void CmdTools::getGlobalPlannerDefaultConfig()
  {
    //    global_planner::GlobalPlannerConfig config;
    //    dClient_->getCurrentConfiguration(config);
    //    default_neutral_cost_ = config.neutral_cost;
    //    default_lethal_cost_ = config.lethal_cost;
    //    last_neutral_cost_ = config.neutral_cost;
    //    last_lethal_cost_ = config.lethal_cost;
    //    ROS_INFO_STREAM_THROTTLE(1.0, "default neutral cost:" << default_neutral_cost_
    //                                                          << "default lethal cost:" << default_lethal_cost_);
  }

  void CmdTools::sendStackGimbalCommand(ros::Time time)
  {
    senders_->gimbal_command_sender_->sendCommand(time);
    senders_->base_gimbal_command_sender_->sendCommand(time);
  }

  void CmdTools::setStackGimbalMode(int mode)
  {
    senders_->gimbal_command_sender_->setMode(mode);
    senders_->base_gimbal_command_sender_->setMode(mode);
  }

  void CmdTools::setStackGimbalRate(double scale_base_yaw, double scale_yaw, double scale_pitch)
  {
    senders_->gimbal_command_sender_->setRate(scale_yaw, scale_pitch);
    senders_->base_gimbal_command_sender_->setRate(scale_base_yaw, 0.0);
  }

  void CmdTools::setStackGimbalPoint()
  {
    //void
  }

  PlannerTools::PlannerTools(ros::NodeHandle &bt_nh) : ServiceCallerBase<rm_msgs::SetLimitVel>(bt_nh,"/set_limit_vel") , ServiceCallerBase<rm_msgs::EnableGyro>(bt_nh , "/enable_gyro")
  {

  }

  void PlannerTools::setLimitVelAndSlideWindow(const float & limit_vel , const float &slide_window)
  {
    SetLimitVelBase::service_.request.limit_vel = limit_vel;
    SetLimitVelBase::service_.request.slide_window = slide_window;
    SetLimitVelBase::callService();
  }

  double PlannerTools::getLimitVel()
  {
    return SetLimitVelBase::service_.response.current_limit_vel;
  }

  double PlannerTools::getSlideWindow()
  {
    return SetLimitVelBase::service_.response.current_slide_window;
  }

  void PlannerTools::setGyroSpeed(const float& gyro_speed)
  {
    EnableGyroBase::service_.request.gyro_speed = gyro_speed;
    EnableGyroBase::callService();
  }

  bool PlannerTools::isGyro()
  {
    return EnableGyroBase::service_.response.is_gyro;
  }

  tf2_ros::Buffer& CmdTools::getTfBuffer()
  {
    return tf_buffer_;
    //  std::pair<ros::Time, geometry_msgs::TransformStamped> check_obstacle_{};
  }

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name, BT::Blackboard& blackboard,
                                              int& last_patrol_position_index, bool sequential_patrol_enable,
                                              bool& is_complete)
  {
    auto all_zones = blackboard.get<std::unordered_map<
      std::string, std::vector<geometry_msgs::PoseStamped>>>("all_zones");

    std::vector<geometry_msgs::PoseStamped> points = all_zones[area_name];

    if (last_patrol_position_index == points.size() - 2) //当下标到了区域点最多的情况，为防止溢出，就将其赋值为-1
    {
      is_complete = true;
    }
    if (sequential_patrol_enable == true)
    {
      last_patrol_position_index = ((last_patrol_position_index + 1) % points.size());
      return points[last_patrol_position_index];
    }
    else
    {
      unsigned int rand_index = (rand() % points.size());
      return points[rand_index];
    }
  }

  bool isPointInPolygon(const geometry_msgs::TransformStamped& point,
                        const std::vector<geometry_msgs::PointStamped>& polygon)
  {
    int n = polygon.size();
    int count = 0;
    for (int i = 0; i < n; ++i)
    {
      if (point.transform.translation.x == polygon[i].point.x && point.transform.translation.y == polygon[i].point.y)
        return true;
      if (point.transform.translation.x == polygon[(i + 1) % n].point.x &&
        point.transform.translation.y == polygon[(i + 1) % n].point.y)
        return true;

      if ((point.transform.translation.y < polygon[i].point.y) !=
        (point.transform.translation.y < polygon[(i + 1) % n].point.y))
      {
        double x = (polygon[(i + 1) % n].point.x - polygon[i].point.x) *
          (point.transform.translation.y - polygon[i].point.y) /
          (polygon[(i + 1) % n].point.y - polygon[i].point.y) +
          polygon[i].point.x;
        if (x > point.transform.translation.x)
          count++;
        else if (x == point.transform.translation.x)
          return true;
      }
    }
    return count % 2 == 1;
  }

  std::string determinePolygonInWhich(const geometry_msgs::TransformStamped& point,
                                      std::unordered_map<std::string, std::vector<geometry_msgs::PointStamped>>
                                      pos_detection_polygons)
  {
    for (const auto& pair : pos_detection_polygons)
    {
      if (isPointInPolygon(point, pair.second))
        return pair.first;
    }
    return "unknown";
  }

  MiniMapTools::MiniMapTools(BT::Blackboard& blackboard, perception::Publisher& publisher,
                             perception::Subscriber& subscriber) : blackboard_(blackboard), publisher_(publisher),
                                                                   subscriber_(subscriber) //用于构造旋转矩阵
  {
    std::vector<double> minimap2map;
    if (!blackboard_.get<std::vector<double>>("minimap2map", minimap2map))
    {
      ROS_ERROR("BT can not access key name [minimap2map] , default value is zero");
      minimap2map.emplace_back(0);
      minimap2map.emplace_back(0);
      minimap2map.emplace_back(0);
    }
    minimap2world_.setOrigin(tf2::Vector3(minimap2map[0], minimap2map[1], 0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, minimap2map[2]);
    minimap2world_.setRotation(quaternion);
  }

  void MiniMapTools::pathTransform(const nav_msgs::Path& goal_path, rm_msgs::MapSentryData* map_sentry_data)
  // 首先括号里代表着要初始化的两个成员变量，函数实现了将路径
  // (goal_path) 转换为某种格式，并存储到 map_sentry_data
  {
    // 注意这里的map_sentry_data是一个指针，只有访问指针下面的成员采用->符号
    map_sentry_data->stamp =
      goal_path.header.stamp; // 将 goal_path 的时间戳 (goal_path.header.stamp) 赋值给 map_sentry_data 的 stamp 字段
    int sentry_intention ;
    if (!blackboard_.get<int>("sentry_intention", sentry_intention))
    {
      sentry_intention = static_cast<int>(types::SentryIntention::MoveToTheTargetPoint);//赋予默认值为3
      ROS_ERROR(
        "BT can not access key name [sentry_intention] , default value is types::ControlState::MoveToTheTargetPoint");
    }
    map_sentry_data->intention = sentry_intention; // 将当前的控制状态保存到目标数据的 intention（意图）字段中
    int num = 0; // num: 初始化路径点的计数器，标记当前处理的是路径中的第几个点；
    int step = ceil(goal_path.poses.size() / 10.0); // step: 计算路径的采样间隔，将路径点数等分为 10 段 ； ceil(...):
    // 使用 ceil 函数向上取整，确保步长为整数且至少为 1
    for (const auto& goal : goal_path.poses) // 遍历路径中的每一个点
    {
      if (step != 0)
      {
        if (num == 0)
          pathPointTransform(map_sentry_data, goal, num / step - 1, true);
        else if (num % step == 0)
          pathPointTransform(map_sentry_data, goal, num / step - 1, false);
      }
      num = num + 1; // 更新点的计数器，处理下一个路径点
    }
  }

  void MiniMapTools::pathPointTransform(rm_msgs::MapSentryData* map_sentry_data, const geometry_msgs::PoseStamped& goal,
                                        int num,
                                        bool is_start_point)
  {
    if (num >= 49)
      return;
    tf2::Transform minimap2path, world2path;
    world2path.setOrigin(tf2::Vector3(goal.pose.position.x, goal.pose.position.y, 0.0)); //设置原点偏移量
    world2path.setRotation(tf2::Quaternion(0, 0, 0, 1)); //设置原点偏移量
    minimap2path = minimap2world_ * world2path; //这里其实是变换矩阵相乘
    if (is_start_point)
    {
      map_sentry_data->start_position_x = minimap2path.getOrigin().x() * 10.0; //乘10是更改精度单位，米改成分米
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

  void MiniMapTools::targetPoseTransform(float sub_x, float sub_y, geometry_msgs::PoseStamped* target_pose)
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

  [[nodiscard]]geometry_msgs::PoseStamped MiniMapTools::getConductPoint()
  {
    geometry_msgs::PoseStamped target_pose;

    targetPoseTransform(subscriber_.getClientMapSendData().target_position_x,
                        subscriber_.getClientMapSendData().target_position_y, &target_pose);
    publisher_.getPublishers()->conduct_point_pub_.publish(target_pose);
    return target_pose;
  }

  NavigationTools::NavigationTools(BT::Blackboard& blackboard , perception::Subscriber &subscriber ,perception::TfAccessor &tf_viewer, CmdTools &cmd_tools , PlannerTools &planner_tools) : blackboard_(blackboard) , subscriber_(subscriber) ,tf_accessor_(tf_viewer), cmd_tools_(cmd_tools) , planner_tools_(planner_tools)
  {
    mbf_client_ = std::make_unique<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>("/move_base_flex/move_base", true);
    if (!blackboard_.get<double>("max_planning_period",max_planning_period_))
    {
      ROS_ERROR("BT can not access key name [max_planning_period] in NavigationTools , default value is 30.0");
      max_planning_period_ = 30.0;
    }
    if (!blackboard_.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones",all_zones))
    {
      ROS_ERROR("BT can not access key name [all_zones] , no default param");
    }
    if (!blackboard_.get<double>("chase_freq",chase_freq_))
    {
      ROS_ERROR("BT can not access key name [chase_freq] in NavigationTools , default value is 15.0");
      chase_freq_ = 15.0;
    }
    if (!blackboard.get<double>("chase_distance",chase_distance_))
    {
      ROS_ERROR("BT can not access key name [chase_distance] in NavigationTools , default value is 2.0");
    }
    if (!blackboard.get<double>("chase_tolerance",chase_tolerance_))
    {
      ROS_ERROR("BT can not access key name [chase_tolerance] in NavigationTools , default value is 0.5");
    }
  }

  void NavigationTools::reachGoalJudgement(const actionlib::SimpleClientGoalState& state , const mbf_msgs::MoveBaseResultConstPtr& result)
  {
    if (result->outcome == mbf_msgs::MoveBaseResult::SUCCESS)
    {
      patrol_state_ = PatrolState::REACHED;
      reach_time_ = ros::Time::now();
    }
    else
    {
      ROS_INFO_THROTTLE(0.5, "failed to reach goal");
      patrol_state_ = PatrolState::IDLE;
    }
  }

  void NavigationTools::patrol(const geometry_msgs::PoseStamped& point, double residence_time_at_point, bool is_conduct_mode , bool move_need_gyro , bool reached_need_gyro)
  {
    ros::Time time = ros::Time::now();
    if (checkMbfClientState())
    {
      if (patrol_state_ == PatrolState::IDLE || patrol_state_ == PatrolState::TIMEOUT)
      {
        int sentry_intention = is_conduct_mode ? static_cast<int>(types::SentryIntention::MoveToTheTargetPoint) : static_cast<int>(types::SentryIntention::DefendAtTheTargetPoint);
        blackboard_.set<int>("sentry_intention",sentry_intention);  //设置sentry intention，给minimap tool使用
        mbf_client_->cancelGoal();
        mbf_goal_.target_pose = point;
        mbf_goal_.direct_track = false;
        mbf_client_->sendGoal(mbf_goal_, [this](const actionlib::SimpleClientGoalState& state,
                                                             const mbf_msgs::MoveBaseResultConstPtr& result) {
            reachGoalJudgement(state, result);
          });
        if (move_need_gyro == true)
        {
          planner_tools_.setGyroSpeed(1.0);
        }else
        {
          planner_tools_.setGyroSpeed(0.0);
        }
        patrol_state_ = PatrolState::MOVING;
        ROS_INFO_STREAM_THROTTLE(0.5, "Present target point is: " << mbf_goal_.target_pose.pose.position.x << ","
                                                    << mbf_goal_.target_pose.pose.position.y
                                                    << ",patrol sequential index:" << patrol_sequential_index_);
        planning_start_time_ = ros::Time::now();
      }
      else
      {
        if (patrol_state_ == PatrolState::REACHED) //目标点信息发出去且到达目标，但是没有待够时间以重设has_determined_goal标志位的情况
        {
          if (reached_need_gyro == true)
          {
            cmd_tools_.getSenders()->vel_2d_command_sender_->setAngularZVel(0.2);
            cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(ros::Time::now());
          }else
          {
            cmd_tools_.getSenders()->vel_2d_command_sender_->setAngularZVel(0.0);
            cmd_tools_.getSenders()->vel_2d_command_sender_->sendCommand(ros::Time::now());
          }
          if (ros::Time::now() - reach_time_ > ros::Duration(residence_time_at_point))
          {
            if (is_conduct_mode)
              subscriber_.clearClientMapUpdateState();
            patrol_state_ = PatrolState::IDLE;
            ROS_INFO_THROTTLE(0.5, "Stay there long enough, change goal.");
          }
        }
        else //目标点信息发出去但是还没有到达目标的情况，可能出现一些状况导致无法到达，因此需要做超时检测
        {
          if (ros::Time::now() - planning_start_time_ > ros::Duration(max_planning_period_))
          {
            mbf_client_->cancelGoal();
            patrol_state_ = PatrolState::TIMEOUT;
            ROS_INFO_THROTTLE(0.5, "Planning timeout, change goal.");
          }
        }
      }
    }
    cmd_tools_.getSenders()->chassis_command_sender_->sendChassisCommand(time, true);
  }

  geometry_msgs::PoseStamped NavigationTools::getPatrolPoint(const std::string& patrol_area_name ,const bool sequential_patrol_enable)
  {
    std::vector<geometry_msgs::PoseStamped> points = all_zones[patrol_area_name];
    if (points.empty())
    {
      ROS_ERROR_THROTTLE(0.5, "Patrol area has no points: %s", patrol_area_name.c_str());
      geometry_msgs::PoseStamped fallback;
      fallback.header.frame_id = "map";
      return fallback;
    }
    if (sequential_patrol_enable) //顺序取点
    {
      if (last_patrol_area_name_ != patrol_area_name)
        patrol_sequential_index_ = -1;
      patrol_sequential_index_ = ((patrol_sequential_index_ + 1) % points.size());
      last_patrol_area_name_ = patrol_area_name;
      return points[patrol_sequential_index_];
    }
    else //随机取点
    {
      unsigned int rand_index = (rand() % points.size());
      last_patrol_area_name_ = patrol_area_name;
      return points[rand_index];
    }
  }

  actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* NavigationTools::getMbfClient() const
  {
    return mbf_client_.get();
  }

  bool NavigationTools::checkMbfClientState()
  {
    bool is_server_connect = getMbfClient()->waitForServer(ros::Duration(0.03));
    if (!is_server_connect)
    {
      ROS_WARN_THROTTLE(0.5, "action not connect");
      if (last_action_state_)
      {
        last_mbf_retry_time_ = ros::Time::now();
        resetMbfClient();
      }
      else if (ros::Time::now() - last_mbf_retry_time_ > ros::Duration(2.0))
      {
        resetMbfClient();
        last_mbf_retry_time_ = ros::Time::now();
      }
    }
    last_action_state_ = is_server_connect;
    return is_server_connect;
  }

  void NavigationTools::resetMbfClient()
  {
    mbf_client_.reset();
    mbf_client_ = std::make_unique<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>(
      "/move_base_flex/move_base", true);
  }

  void NavigationTools::resetPatrolState()
  {
    patrol_state_ = PatrolState::IDLE;
  }

  bool NavigationTools::chase()
  {
    ros::Time time = ros::Time::now();
    if (time - last_chase_time_ > ros::Duration(1.0 / chase_freq_))
    {
      last_chase_time_ = ros::Time::now();
      blackboard_.set<int>("sentry_intention",static_cast<int>(types::SentryIntention::AttackAtTheTargetPoint));
      geometry_msgs::PointStamped target_at_map;
      try
      {
        track_point_.point = subscriber_.getTrackData().position;
        geometry_msgs::TransformStamped transform_stamped =
            tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP, perception::TfAccessor::FrameId::TRACK);

        tf2::doTransform(track_point_, target_at_map, transform_stamped);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_THROTTLE(0.5, "Failed to transform point: %s", ex.what());
        return false;
      }
      if (std::hypot(target_at_map.point.x - last_target_at_map_.point.x , target_at_map.point.y - last_target_at_map_.point.y) < 0.1)
      {
        ROS_INFO("target at map move too short , skip!");
        return true;
      }
      last_target_at_map_ = target_at_map;
      service_processor::SearchEnablePoint srv;
      srv.request.target_pos = target_at_map;
      srv.request.robot_pos = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP,perception::TfAccessor::FrameId::BASE_LINK);
      srv.request.chase_distance = chase_distance_;
      srv.request.chase_tolerance = chase_tolerance_;

      if (service_client_.call(srv))
      {
        mbf_msgs::MoveBaseGoal mbf_goal;
        mbf_goal.target_pose = srv.response.move_point;
        mbf_goal.direct_track = !srv.response.is_block_on_line;
        if (checkMbfClientState())
          mbf_client_->sendGoal(mbf_goal);
      }
      else
      {
        ROS_WARN("chase service client can not get response");
        return false;
      }
      return true;
    }
    return true;
  }

  void NavigationTools::resetLastTargetAtMap()
  {
    last_target_at_map_.point.x = 0;
    last_target_at_map_.point.y = 0;
    last_target_at_map_.point.z = 0;
  }

  ControllerTools::ControllerTools(ros::NodeHandle &bt_nh) : bt_nh_(bt_nh)
  {
    bt_nh_.getParam("shooter_calibration",shooter_calibration_config_);
    controller_manager_ = std::make_unique<rm_common::ControllerManager>(bt_nh_);
    shooter_calibration_queue_ = std::make_unique<rm_common::CalibrationQueue>(shooter_calibration_config_,bt_nh_,*controller_manager_);

    XmlRpc::XmlRpcValue controllers_list;
    bt_nh_.getParam("controllers_list",controllers_list);
    ROS_ASSERT(controllers_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if (!controllers_list.hasMember("main_controllers"))
    {
      ROS_ERROR("ROS can not access key name [main_controllers] in ControllerTools");
      return;
    }

    //----------------------main controllers-------------------

    XmlRpc::XmlRpcValue  main_ctrls_xml = controllers_list["main_controllers"];
    ROS_ASSERT(main_ctrls_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < main_ctrls_xml.size(); ++i)
    {
      if (main_ctrls_xml[i].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        main_controllers_.push_back(static_cast<std::string>(main_ctrls_xml[i]));
      }
      else
      {
        ROS_ERROR("Element at index %d in main_controllers is not a string!", i);
      }
    }

    //--------------------calibration controllers------------------------

    XmlRpc::XmlRpcValue  calibration_ctrls_xml = controllers_list["calibration_controllers"];
    ROS_ASSERT(main_ctrls_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < calibration_ctrls_xml.size(); ++i)
    {
      if (calibration_ctrls_xml[i].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        calibration_controllers_.push_back(static_cast<std::string>(calibration_ctrls_xml[i]));
      }
      else
      {
        ROS_ERROR("Element at index %d in calibration_controllers is not a string!", i);
      }
    }
  }

  [[nodiscard]]rm_common::ControllerManager* ControllerTools::getControllerManager() const
  {
    return controller_manager_.get();
  }

  void ControllerTools::calibrate()
  {
    shooter_calibration_queue_->reset();
  }

  void ControllerTools::ControllerUpdate()
  {
    if (!controller_manager_)
    {
      return;
    }
    ros::Time time = ros::Time::now();
    // // gimbal_calibration - DISABLED: gimbal does not need calibration
    // // if (gimbal_calibration_)
    // //   gimbal_calibration_->update(time);
    if (shooter_calibration_queue_)
    {
      shooter_calibration_queue_->update(time);
    }
    controller_manager_->update();
  }

  void ControllerTools::startMainController()
  {
    for (const auto & controller : main_controllers_)
    {
      controller_manager_->startController(controller);
    }
  }

  void ControllerTools::stopMainController()
  {
    for (const auto & controller : main_controllers_)
    {
      controller_manager_->stopController(controller);
    }
  }

  void ControllerTools::stopCalibrationController()
  {
    for (const auto & controller : calibration_controllers_)
    {
      controller_manager_->stopController(controller);
    }
  }

  GimbalTools::GimbalTools(perception::TfAccessor& tf_accessor , CmdTools &cmd_tools , ros::NodeHandle &bt_nh) : tf_accessor_(tf_accessor) , cmd_tools_(cmd_tools) , bt_nh(bt_nh)
  {
    ros::NodeHandle pitch_nh = ros::NodeHandle(bt_nh,"pitch");
    if (!pitch_nh.getParam("max_pitch_angle",max_pitch_angle_))
    {
      ROS_ERROR("GimbalTools can not get param named [max_pitch_angle]");
    }
    if (!pitch_nh.getParam("min_pitch_angle",min_pitch_angle_))
    {
      ROS_ERROR("GimbalTools can not get param names [min_pitch_angle]");
    }
  }

  void GimbalTools::updatePitchStrafeDirect(double min_angel , double max_angle , double pitch_outside_vel , double pitch_inside_vel , double breach_threshold)
  {
    geometry_msgs::TransformStamped yaw2pitch_;
    try
    {
      yaw2pitch_ = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::YAW,perception::TfAccessor::FrameId::BASE_LINK);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(0.5, "%s", ex.what());
      return;
    }
    double roll_temp, pitch, yaw_temp;
    quatToRPY(yaw2pitch_.transform.rotation, roll_temp, pitch, yaw_temp);

    if (pitch >= max_angle)
    {
      pitch_direct_ = -pitch_inside_vel;
      if (pitch - max_angle >= breach_threshold)
        pitch_direct_ = -pitch_outside_vel;
    }
    else if (pitch <= min_angel)
    {
      pitch_direct_ = pitch_inside_vel;
      if (pitch - min_angel <= -breach_threshold)
        pitch_direct_ = pitch_outside_vel;
    }
  }

  void GimbalTools::setStackGimbalRate(double scale_yaw, double scale_pitch)
  {
    cmd_tools_.getSenders()->gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    cmd_tools_.getSenders()->gimbal_command_sender_->setRate(scale_yaw, scale_pitch);
    traj_pitch_ = cmd_tools_.getSenders()->gimbal_command_sender_->getMsg()->rate_pitch * 0.01 + traj_pitch_; //原pitch加上速度等于本次pitch
    if (traj_pitch_ > max_pitch_angle_)
      traj_pitch_ = max_pitch_angle_;
    if (traj_pitch_ < min_pitch_angle_)
      traj_pitch_ = min_pitch_angle_; //做保护
    cmd_tools_.getSenders()->gimbal_command_sender_->setTrajFrameId("base_yaw");//odom to base_yaw
    cmd_tools_.getSenders()->gimbal_command_sender_->setGimbalTraj(0.0, traj_pitch_);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setRate(scale_yaw, 0.);
    cmd_tools_.sendStackGimbalCommand(ros::Time::now());
  }

  void GimbalTools::setStackGimbalRate()
  {
    cmd_tools_.getSenders()->gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    cmd_tools_.getSenders()->gimbal_command_sender_->setRate(yaw_direct_, pitch_direct_);
    traj_pitch_ = cmd_tools_.getSenders()->gimbal_command_sender_->getMsg()->rate_pitch * 0.01 + traj_pitch_; //原pitch加上速度等于本次pitch
    if (traj_pitch_ > max_pitch_angle_)
      traj_pitch_ = max_pitch_angle_;
    if (traj_pitch_ < min_pitch_angle_)
      traj_pitch_ = min_pitch_angle_; //做保护
    cmd_tools_.getSenders()->gimbal_command_sender_->setTrajFrameId("base_yaw");//odom to base_yaw
    cmd_tools_.getSenders()->gimbal_command_sender_->setGimbalTraj(0.0, traj_pitch_);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setRate(yaw_direct_, 0.);
    cmd_tools_.sendStackGimbalCommand(ros::Time::now());
  }

  void GimbalTools::lidarTwist(double yaw_vel , double scan_range_circles)
  {
    geometry_msgs::TransformStamped map2yaw_;
    try
    {
      map2yaw_ = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP , perception::TfAccessor::FrameId::YAW);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(0.5, "%s", ex.what());
      return;
    }
    double yaw = yawFromQuat(map2yaw_.transform.rotation);

    if (circle_count_ <= 0)
      yaw_direct_ = cmd_tools_.smoothlyYawOutput(yaw_vel); //实际上就是限制加速度
    if (circle_count_ > scan_range_circles) //通过圈数计数器，使得其能够在0圈到指定圈数中来回正反转
      yaw_direct_ = cmd_tools_.smoothlyYawOutput(-yaw_vel);

    if (yaw_direct_ == yaw_vel || yaw_direct_ == -yaw_vel)
    {
      if (yaw - lidar_twist_last_yaw_ > M_PI) //处理计数器
      {
        circle_count_--;
      }
      else if (yaw - lidar_twist_last_yaw_ < -M_PI)
      {
        circle_count_++;
      }
      lidar_twist_last_yaw_ = yaw;
    }
  }

  void GimbalTools::setGimbalDirectPoint(geometry_msgs::PointStamped point_of_map)
  {
    geometry_msgs::PointStamped point_of_odom;
    geometry_msgs::TransformStamped odom2map;
    odom2map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::ODOM,perception::TfAccessor::FrameId::MAP);
    tf2::doTransform(point_of_map, point_of_odom, odom2map); //中文语义：将map坐标系的物体转换到odom下
    cmd_tools_.getSenders()->gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
    cmd_tools_.getSenders()->gimbal_command_sender_->setPoint(point_of_odom);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setPoint(point_of_odom);
    cmd_tools_.sendStackGimbalCommand(ros::Time::now());
  }

  void GimbalTools::setStackGimbalTrack()
  {
    cmd_tools_.getSenders()->gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    // double bullet_speed = union_cmd_sender_->double_barrel_cmd_sender_->getSpeed();
    double bullet_speed = cmd_tools_.getSenders()->shooter_command_sender_->getSpeed();
    cmd_tools_.getSenders()->gimbal_command_sender_->setBulletSpeed(bullet_speed);
    cmd_tools_.getSenders()->base_gimbal_command_sender_->setBulletSpeed(bullet_speed);
    cmd_tools_.sendStackGimbalCommand(ros::Time::now());
  }
}
