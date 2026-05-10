//
// Created by root on 2026/5/10.
//
#include "common/tools.h"

namespace tools
{
  EnableGyroServiceCaller::EnableGyroServiceCaller(ros::NodeHandle& nh) : ServiceCallerBase<rm_msgs::EnableGyro>(nh, "/enable_gyro")
  {
    service_.request.gyro_speed = 0.0;
    callService();
  }

  void EnableGyroServiceCaller::setGyro(double gyro_speed)
  {
    service_.request.gyro_speed = gyro_speed;
  }

  bool EnableGyroServiceCaller::isGyro()
  {
    return service_.response.is_gyro;
  }

  void EnableGyroServiceCaller::enable()
  {
    callService();
  }

  SetLimitVelServiceCaller::SetLimitVelServiceCaller(ros::NodeHandle& nh,const double init_limit_vel)
      : ServiceCallerBase<rm_msgs::SetLimitVel>(nh, "/set_limit_vel")
  {
    service_.request.limit_vel = init_limit_vel;
    callService();
  }

  void SetLimitVelServiceCaller::setLimitVel(const double& limit_vel)
  {
    service_.request.limit_vel = limit_vel;
    //    ROS_INFO("set planner's limit vel: %f", service_.request.limit_vel);
    callService();
  }

  void SetLimitVelServiceCaller::setSlideWindow(const double slide_window)
  {
    service_.request.slide_window = slide_window;
    callService();
  }

  double SetLimitVelServiceCaller::getLimitVel() const
  {
    return service_.response.current_limit_vel;
  }

  CmdTools::CmdTools(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
  {
    senders_ = std::make_unique<Senders>();
    ros::NodeHandle chassis_nh(nh, "chassis");
    senders_->chassis_command_sender_ = std::make_unique<rm_common::ChassisCommandSender>(chassis_nh);
    ros::NodeHandle vel_nh(nh, "vel");
    senders_->vel_2d_command_sender_ = std::make_unique<rm_common::Vel2DCommandSender>(vel_nh);
    ros::NodeHandle base_gimbal_nh(nh,"base_gimbal");
    senders_->base_gimbal_command_sender_ = std::make_unique<rm_common::GimbalCommandSender>(base_gimbal_nh);
    ros::NodeHandle gimbal_nh(nh,"gimbal");
    senders_->gimbal_command_sender_ = std::make_unique<rm_common::GimbalCommandSender>(gimbal_nh);
    ros::NodeHandle shooter_nh(nh,"shooter");
    senders_->shooter_command_sender_ = std::make_unique<rm_common::ShooterCommandSender>(shooter_nh);

    mbf_client_ =
        std::make_unique<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>("/move_base_flex/move_base", true);
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

  auto CmdTools::getMbfClient() const
  {
    return mbf_client_.get();
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

  tf2_ros::Buffer& CmdTools::getTfBuffer()
  {
    return tf_buffer_;
    //  std::pair<ros::Time, geometry_msgs::TransformStamped> check_obstacle_{};
  }

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name, BT::Blackboard& blackboard, int& last_patrol_position_index, bool sequential_patrol_enable, bool& is_complete)
  {
    auto all_zones=blackboard.get<std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>>>("all_zones");

    std::vector<geometry_msgs::PoseStamped> points = all_zones[area_name];

    if (last_patrol_position_index == points.size()-2) //当下标到了区域点最多的情况，为防止溢出，就将其赋值为-1
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

  bool isPointInPolygon(const geometry_msgs::TransformStamped& point, const std::vector<geometry_msgs::PointStamped>& polygon)
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

  std::string determinePolygonInWhich(const geometry_msgs::TransformStamped& point, std::unordered_map<std::string, std::vector<geometry_msgs::PointStamped>> pos_detection_polygons)
  {
    for (const auto& pair : pos_detection_polygons)
    {
      if (isPointInPolygon(point, pair.second))
        return pair.first;
    }
    return "unknown";
  }
}
