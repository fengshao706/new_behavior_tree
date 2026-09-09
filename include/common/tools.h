#ifndef NEW_BEHAVIOR_TREE_TOOLS_H
#define NEW_BEHAVIOR_TREE_TOOLS_H

//
// Created by spy on 2023/7/26.
//
#pragma once

#include "rm_common/decision/service_caller.h"
#include <math.h>
#include <perception_layer.h>
#include <rm_msgs/EnableGyro.h>
#include <rm_msgs/SetLimitVel.h>
#include <rm_msgs/PriorityArray.h>
#include <rm_msgs/MapSentryData.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TransformStamped.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/RecoveryAction.h>
#include <global_planner/GlobalPlannerConfig.h>
#include <dynamic_reconfigure/client.h>
#include <service_processor/SearchEnablePoint.h>
#include "types.h"
#include "rm_common/decision/command_sender.h"
#include "rm_common/filters/filters.h"
#include "rm_common/decision/service_caller.h"
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>
#include "behaviortree_cpp/blackboard.h"
#include <XmlRpcValue.h>
#include <common/types.h>
#include <rm_common/ori_tool.h>

#include "dx_track_switch_caller.h"
#include "common/patrol_retry_cooldown.h"

namespace perception
{
  class Publisher;
  class Subscriber;
  class TfAccessor;
}

namespace tools
{
  template <typename T>
  constexpr bool isBetween(const T& x, const T& min, const T& max)
  {
    return (x >= min && x < max);
  }

  class PlannerTools : public rm_common::ServiceCallerBase<rm_msgs::SetLimitVel>,
                       public rm_common::ServiceCallerBase<rm_msgs::EnableGyro>
  {
  public:
    PlannerTools(ros::NodeHandle& bt_nh);

    void setLimitVelAndSlideWindow(const float& limit_vel, const float& slide_window);

    double getLimitVel();

    double getSlideWindow();

    void setGyroSpeed(const float& gyro_speed);

    bool isGyro();

  protected:
    using SetLimitVelBase = rm_common::ServiceCallerBase<rm_msgs::SetLimitVel>;
    using EnableGyroBase = rm_common::ServiceCallerBase<rm_msgs::EnableGyro>;

  private:
  };

  class CmdTools
  {
  public:
    struct Senders
    {
      std::unique_ptr<rm_common::ChassisCommandSender> chassis_command_sender_;
      std::unique_ptr<rm_common::GimbalCommandSender> gimbal_command_sender_;
      std::unique_ptr<rm_common::ShooterCommandSender> shooter_command_sender_;
      std::unique_ptr<rm_common::Vel2DCommandSender> vel_2d_command_sender_;
    };

    CmdTools(ros::NodeHandle& nh, BT::Blackboard& blackboard);

    Senders* getSenders() const;

    auto getDClient() const;

    double getYawDirect() const;

    void setYawDirect(double yaw_direct);

    void yawPidCompute(const double angle);

    double smoothlyYawOutput(const double cmd);

    double smoothlyPitchOutput(const double cmd);

    void setGlobalPlannerParam(int lethal_cost, int neutral_cost);

    void getGlobalPlannerDefaultConfig();

    void sendStackGimbalCommand(ros::Time time);

    void setStackGimbalMode(int mode);

    void setStackGimbalRate(double scale_base_yaw, double scale_yaw, double scale_pitch);

    void setStackGimbalPoint();

    tf2_ros::Buffer& getTfBuffer();

  private:
    std::unique_ptr<Senders> senders_;
    std::unique_ptr<dynamic_reconfigure::Client<global_planner::GlobalPlannerConfig>> dClient_;

    control_toolbox::Pid yaw_pid_;
    // PID

    RampFilter<double>* ramp_yaw_{};
    RampFilter<double>* ramp_pitch_{};
    double pitch_acc_, yaw_acc_;
    double yaw_direct_{};

    int last_neutral_cost_, last_lethal_cost_, default_neutral_cost_, default_lethal_cost_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    BT::Blackboard& blackboard_;
    //  std::deque<std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>> goal_deque;
  };

  /**
   *@brief 该函数用于获取某个区域的用于导航的坐标点
   *@param area_name 所要获取的点所在的区域
   *@param blackboard 黑板
   *@param last_patrol_position_index 上一次的导航点，当使用队列模式的时候，该导航点下标每次会加一,你需要在外部维护一个这个变量
   *@param sequential_patrol_enable 是否使用队列模式，若为否，则使用随机模式
   *@param is_complete 所有的巡逻点是否已经全部巡逻完毕，该变量需要在外部进行维护并可读取它的值以做进一步分析
   * **/

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name, BT::Blackboard& blackboard,
                                              int& last_patrol_position_index, bool sequential_patrol_enable,
                                              bool& is_complete);

  class MiniMapTools
  {
  public:
    MiniMapTools(BT::Blackboard& blackboard, perception::Publisher& publisher, perception::Subscriber& subscriber);

    /**@brief 将导航生成的规划路径转换为通过常规链路向对应的操作手选手端发送路径的坐标数据
     *@param goal_path 导航生成的路径点
     *@param map_sentry_data 用于发送的数据
     * **/
    void pathTransform(const nav_msgs::Path& goal_path, rm_msgs::MapSentryData* map_sentry_data);

    void pathPointTransform(rm_msgs::MapSentryData* map_sentry_data, const geometry_msgs::PoseStamped& goal, int num,
                            bool is_start_point);

    /** @brief 用于将相对于小地图坐标系的点转换为相对于世界坐标系的点
     * @param sub_x 相对于小地图坐标系的点的x坐标
     * @param sub_y 相对于小地图坐标系的点的y坐标
     * @param target_pose 该函数计算出来的最终结果将存储在这个变量中
     *  **/
    void targetPoseTransform(float sub_x, float sub_y, geometry_msgs::PoseStamped* target_pose);

    /**@brief 用于从订阅者中获取操作手指引的目标导航点
     * **/
    [[nodiscard]] geometry_msgs::PoseStamped getConductPoint();

  private:
    BT::Blackboard& blackboard_;
    tf2::Transform minimap2world_;
    double last_point_x_ = 0;
    double last_point_y_ = 0;
    perception::Publisher& publisher_;
    perception::Subscriber& subscriber_;
  };

  class NavigationTools
  {
  public:
    enum class PatrolState
    {
      IDLE,
      MOVING,
      REACHED,
      TIMEOUT,
    };

    NavigationTools(BT::Blackboard& blackboard, perception::Subscriber& subscriber, perception::TfAccessor& tf_viewer,
                    CmdTools& cmd_tools, PlannerTools& planner_tools);

    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* getMbfClient() const;

    /**@brief 用于点位巡航，通过mbf实现
     *@param point 导航点，常通过getPatrolPoint获得
     *@param residence_time_at_point 在单个点中的停留时间
     *@param is_conduct_mode 是否为云台手指引的点位，该参数影响client map update状态的维护
     *@param move_need_gyro 导航运动到指定目标的过程中是否需要开启小陀螺
     *@param reached_need_gyro 哨兵到达目标点并停留的时候是否需要开启小陀螺
     * **/
    void patrol(const geometry_msgs::PoseStamped& point, double residence_time_at_point, bool is_conduct_mode,
                bool move_need_gyro, bool reached_need_gyro);

    /**@brief 用于从all_zones中获取指定patrol_area_name中的巡航点，该函数维护patrol_sequential_index_，
     *通过更改index的方式实现多点巡航
     *@param patrol_area_name 巡航区域名字
     *@param sequential_patrol_enable 是否开启顺序巡航，若为false，则为随机点位巡航
     * **/
    geometry_msgs::PoseStamped getPatrolPoint(const std::string& patrol_area_name, const bool sequential_patrol_enable);

    /**@brief 用于在打断原patrol的时候重置参数
     * **/
    void resetPatrolState();

    bool isPointInPolygon(const geometry_msgs::Point& point,
                          const std::vector<geometry_msgs::PointStamped>& polygon);

    [[nodiscard]] std::string determinePolygonInWhich(const geometry_msgs::Point& point);

    /**@brief 用于获取Track所标记的目标在地图上面的位置，并给mbf发送该位置坐标用于追击敌人
     * **/
    bool chase();

    /**@brief 用于在退出追击逻辑的时候重置记录的目标在地图上面的位置
     * **/
    void resetLastTargetAtMap();

  private:
    bool checkMbfClientState();

    void resetMbfClient();

    void reachGoalJudgement(const actionlib::SimpleClientGoalState& state,
                            const mbf_msgs::MoveBaseResultConstPtr& result);


    BT::Blackboard& blackboard_;
    perception::Subscriber& subscriber_;
    perception::TfAccessor& tf_accessor_;
    std::unique_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> mbf_client_;

    ros::Time last_mbf_retry_time_;
    bool last_action_state_ = false;
    mbf_msgs::MoveBaseGoal mbf_goal_;
    ros::Time planning_start_time_; //用于超时检测，判断距离send goal过去了多长时间
    ros::Time reach_time_; //用于检查在一个点中停留的时间，超时将会切换到下一个点
    ros::Time last_chase_time_;
    double max_planning_period_; //规划器超时时间
    double chase_freq_;
    double chase_distance_ = 2.0;
    double chase_tolerance_ = 0.5;
    CmdTools& cmd_tools_;
    int patrol_sequential_index_ = -1;
    std::string last_patrol_area_name_{};
    PatrolState patrol_state_ = PatrolState::IDLE;
    bool has_determined_goal_ = false; //本轮巡航目标是否已定（锁定 index，防止每 tick 重复累加导致跳点/重复点）
    std::unordered_map<std::string, std::vector<geometry_msgs::PoseStamped>> all_zones;
    std::unordered_map<std::string, std::vector<geometry_msgs::PointStamped>> pos_detection_polygons;
    geometry_msgs::PointStamped track_point_;
    geometry_msgs::PointStamped last_target_at_map_;
    ros::ServiceClient service_client_;
    PlannerTools& planner_tools_;
  };

  class ControllerTools
  {
  public:
    ControllerTools(ros::NodeHandle& bt_nh);

    [[nodiscard]] rm_common::ControllerManager* getControllerManager() const;

    void calibrate();

    /** @brief 强制终止校准并恢复全部主控制器（停 calibration controllers + startMainController 兜底）
     * **/
    void stopCalibration();

    /** @brief 用于刷新控制器启停管理器，该函数需要在主循环线程中调用
     * **/
    void ControllerUpdate();

    /**@brief 用于从参数文件中列出来的state_controllers中启动对应的控制器
     * **/
    void startStateController();

    /**@brief 用于从参数文件中列出来的state_controllers中停止对应的控制器
     * **/
    void stopStateController();

    /**@brief 用于从参数文件中列出来的main_controllers中启动对应的控制器
     * **/
    void startMainController();

    /**@brief 用于从参数文件中列出来的main_controllers中停止对应的控制器
     * **/
    void stopMainController();

    /**@brief 用于从参数文件中列出来的calibration_controllers中停止对应的控制器
     * **/
    void stopCalibrationController();

  private:
    ros::NodeHandle& bt_nh_;
    std::unique_ptr<rm_common::ControllerManager> controller_manager_;
    XmlRpc::XmlRpcValue shooter_calibration_config_;
    std::unique_ptr<rm_common::CalibrationQueue> shooter_calibration_queue_;
    std::vector<std::string> main_controllers_;
    std::vector<std::string> state_controllers_;
    std::vector<std::string> calibration_controllers_;
  };

  class GimbalTools
  {
  public:
    GimbalTools(perception::TfAccessor& tf_accessor, CmdTools& cmd_tools, ros::NodeHandle& bt_nh);

    /**@brief 更新pitch轴扫描的pitch指向
     *@param min_angel pitch轴指向的最小角度
     *@param max_angle pitch轴指向的最大角度
     *@param pitch_outside_vel 严重超出给定pitch最大最小角度的pitch回复速度
     *@param pitch_inside_vel 未超出pitch最大最小角度的pitch回复速度
     *@param breach_threshold 判定pitch严重超出给定pitch最大最小角度的阈值
     *
     * **/
    void updatePitchStrafeDirect(double min_angel, double max_angle, double pitch_outside_vel, double pitch_inside_vel,
                                 double breach_threshold);

    /**@brief 将期望的机器人云台yaw轴速度和pitch轴速度信息填充进gimbal_cmd里面，供后续发布，小yaw使用traj模式，大yaw使用rate模式
     *@param scale_yaw yaw轴运动速度相对于最大速度的比率
     *@param scale_pitch pitch轴运动速度相对于最大速度的比率
     * **/
    void setStackGimbalRate(double scale_yaw, double scale_pitch);

    /**@brief 将期望的机器人云台yaw轴速度和pitch轴速度信息填充进gimbal_cmd里面，供后续发布，小yaw使用traj模式，大yaw使用rate模式，
     *该函数直接使用成员变量中存储的direct，因此在调用它时，请先更新成员变量中的direct
     * **/
    void setStackGimbalRate();

    /**@brief 让机器人的云台在一定的范围内来回旋转，针对于yaw轴
     *@param yaw_vel yaw轴旋转的速度
     *@param scan_range_circles yaw轴将在该圈数限制下来回运动
     * **/
    void lidarTwist(double yaw_vel, int scan_range_circles);

    /**@brief 该函数用于让云台指向map坐标系下的某个点
     *@param point_of_map map坐标系下的点坐标
     * **/
    void setGimbalDirectPoint(geometry_msgs::PointStamped point_of_map);

    /**@brief 用于给云台设置为track模式，并填充cmdGimbal中的bullet_speed字段
     * **/
    void setStackGimbalTrack();

    /** @brief 将轨迹积分初值重置为云台当前角度,防止进入扫描/自动时产生阶跃
     * 在 StatefulActionNode 的 onStart 中调用,使首帧期望=当前位置  **/

    void resetTrajToCurrent();

  private:
    perception::TfAccessor& tf_accessor_;
    CmdTools& cmd_tools_;
    double pitch_direct_{};
    double yaw_direct_{};
    double traj_pitch_{};
    double traj_yaw_{};
    double max_pitch_angle_{};
    double min_pitch_angle_{};
    int circle_count_{};
    double lidar_twist_last_yaw_{};
    ros::NodeHandle& bt_nh;
    ros::Time last_update_time_{};
  };

  class AutoAimTools
  {
  public:
    enum class Result
    {
      Ready,
      Switching,
      Failed
    };

    // 参与目标切换的四种视觉服务。
    enum class Service
    {
      Detection, // 识别/处理器
      Exposure, // 曝光
      Forecast, // 预测
      Buff // 打符
    };

    AutoAimTools(ros::NodeHandle &bt_nh , CmdTools &cmd_tools,
                 auto_aim::DxTrackSwitchCaller &dx_track_switch_caller);

    /**@brief 将从/track 话题中读取到的原始数据归一化到连续的语义
    *@details 用 outpost_id 是否为 0 来区分"前哨"(!=0) 和 "打符"(==0)，避免和哨兵 id 冲突：
              outpost_id != 0 -> 置为内部前哨 6（前哨和 outpost_id 都改成 6）；
              否则 id==6（旧式哨兵）-> 置为内部哨兵 7；
              其余 id 原样保留。
     *@param input /track话题中的原始数据
     *@param legacy_dx_track_ids 判定是否使用原始数据，为false时数据直接原样输出
     * **/
    rm_msgs::TrackData normalizeTrackData(const rm_msgs::TrackData& input, const bool legacy_dx_track_ids);

    /**@brief 转换单个id
     *@details 12 -> 6(前哨)，6 -> 7(哨兵)，其余原样
     *@param id 输入的原始id
     *@param legacy_dx_track_ids 判定是否使用原始数据，为false时数据直接原样输出
     * **/
    uint8_t normalizeDetectionId(const uint8_t id, const bool legacy_dx_track_ids);

    /**@brief 对一整个数组进行id转换
     *@param input 待转换的detection array
     *@param legacy_dx_track_ids 判定是否使用原始数据，为false时数据直接原样输出
     * **/
    rm_msgs::TargetDetectionArray normalizeDetectionArray(const rm_msgs::TargetDetectionArray& input,
                                                          const bool legacy_dx_track_ids);

    /**@brief 服务层协议限制：除 Forecast 外，切换服务时一律把大符映射成小符，
             只有 Forecast 服务才能真正区分大小符
     * **/
    static uint8_t serviceTarget(const Service service, const uint8_t target)
    {
      if (target == rm_msgs::StatusChangeRequest::BIG_BUFF && service != Service::Forecast)
        return rm_msgs::StatusChangeRequest::SMALL_BUFF;
      return target;
    }

    /**@brief 获取当前所处的服务名
     * **/
    const char* getCurrentServiceName() const
    {
      if (!transition_active_)
        return "ready";
      return serviceName(serviceOrder(transition_target_)[step_index_]);
    }

    /**@brief 进入打符窗口的公共入口
    *@details 先把射速压到 HeatLimit::MINIMAL，
             若上一轮 buff 链已失败则直接退回装甲窗口并返回 Failed
     @param target 目标 (SMALL_BUFF or BIG_BUFF)
     @param now 当前时刻
     * **/
    Result enterBuffWindow(const uint8_t target, const ros::Time& now)
    {
      if (!shooter_limited_)
      {
        last_shoot_frequency_ =
          cmd_tools_.getSenders()->shooter_command_sender_->getShootFrequency();
        cmd_tools_.getSenders()->shooter_command_sender_->setShootFrequency(
          rm_common::HeatLimit::MINIMAL);
        shooter_limited_ = true;
      }
      if (buff_entry_failed_)
      {
        requestTarget(rm_msgs::StatusChangeRequest::ARMOR, now);
        return Result::Failed;
      }
      return requestTarget(target, now);
    }

    /**@brief 进入装甲识别模式
     * **/
    Result restoreArmorWindow(const ros::Time& now = ros::Time::now())
    {
      const Result result = requestTarget(rm_msgs::StatusChangeRequest::ARMOR, now);
      if (result == Result::Ready)
      {
        if (shooter_limited_)
        {
          cmd_tools_.getSenders()->shooter_command_sender_->setShootFrequency(
            last_shoot_frequency_);
          shooter_limited_ = false;
        }
        buff_entry_failed_ = false;
      }
      return result;
    }

    /**@brief 切换前向相机只看前哨
     * **/
    void syncOutpostAimArmorTarget(const bool enabled)
    {
      syncBuildingAimArmorTarget(enabled ? std::vector<int8_t>{ 6 } :
                                           std::vector<int8_t>{ 1, 2, 3, 4, 5, 6, 7 },
                                 enabled ? "OUTPOST_ONLY" : "ROBOTS_AND_OUTPOST_WITHOUT_BASE");
    }

    /**@brief 切换前向相机只看基地
     * **/
    void syncBaseAimArmorTarget()
    {
      syncBuildingAimArmorTarget({ 8 }, "BASE_ONLY");
    }

  private:
    /**@brief 服务枚举 -> 人类可读名（ROS_INFO 用）
   *@param service 服务
   * **/
    static const char* serviceName(const Service service)
    {
      switch (service)
      {
      case Service::Detection:
        return "detection";
      case Service::Exposure:
        return "exposure";
      case Service::Forecast:
        return "forecast";
      case Service::Buff:
        return "buff";
      }
      return "unknown";
    }

    /**@brief 依据目标类型返回四服务的调用顺序
      @param target 目标id
    @return 打符目标 -> Detection/Exposure/Buff/Forecast；
            非打符 -> Buff/Detection/Forecast/Exposure
      * **/
    static std::array<Service, 4> serviceOrder(const uint8_t target)
    {
      if (isBuffTarget(target))
        return {Service::Detection, Service::Exposure, Service::Buff, Service::Forecast};
      return {Service::Buff, Service::Detection, Service::Forecast, Service::Exposure};
    }

    /**@brief 判定目标是否为打符目标
      *@param target 待判定的目标
      * **/
    static bool isBuffTarget(const uint8_t target)
    {
      return target == rm_msgs::StatusChangeRequest::SMALL_BUFF || target == rm_msgs::StatusChangeRequest::BIG_BUFF;
    }

    /**@brief 把"对某服务的操作"分发到实际对应的服务 caller 上（识别/曝光/预测/打符）
     *@param service 服务
     * **/
    template <typename Callback>
    void withService(const Service service, Callback callback)
    {
      switch (service)
      {
      case Service::Detection:
        callback(&dx_track_switch_caller_);
        break;
      case Service::Exposure:
        callback(switch_exposure_srv_.get());
        break;
      case Service::Forecast:
        callback(switch_buff_type_srv_.get());
        break;
      case Service::Buff:
        callback(switch_buff_srv_.get());
        break;
      }
    }

    /**@brief 启动一次新的目标切换
     *@details 把目标态设为目标、重置各种中间变量、步进从第 0 步开始
     * **/
    void startTransition(const uint8_t target, const ros::Time& now)
    {
      desired_target_ = target;
      transition_target_ = target;
      transition_active_ = true;
      call_in_flight_ = false;
      restart_after_call_ = false;
      step_index_ = 0;
      attempt_count_ = 0;
      retry_not_before_ = now;
    }

    /**@brief 设置相机所要识别的目标
     * **/
    void syncBuildingAimArmorTarget(const std::vector<int8_t>& front_target_ids, const char* mode_name)
    {
      if (transition_active_ || effective_target_ != rm_msgs::StatusChangeRequest::ARMOR)
        return;
      const uint8_t desired_target = rm_msgs::StatusChangeRequest::ARMOR;
      // 为单个相机服务下发放置目标集合（ARMOR + 指定 armor target），仅在未在调用且
      // 参数不一致时才真正发出，返回是否真的更新了。
      const auto sync = [desired_target](auto* caller, const uint8_t desired_armor_target)
      {
        if (caller == nullptr || caller->isCalling() ||
          (caller->getTarget() == desired_target && caller->getArmorTarget() == desired_armor_target))
        {
          return false;
        }
        caller->setTargetType(desired_target);
        caller->setArmorTargetType(desired_armor_target);
        caller->callService(); //给dx_track设置
        return true;
      };

      // 只有前向 dx_track 链能用于瞄前哨/基地，其分组协议值由 DxTrackSwitchCaller 收窄为
      // ID 6(前哨) 或 ID 8(基地)。侧相机始终识别机器人装甲，绝不能开"基地分组目标"。
      // 在专门的"仅前哨"窗口之外，不要把前哨从 RoundSearchEnemy 的搜索里藏掉：
      // 前向 dx_track 收到的 ID 为 1..7（不含基地 8）。
      // 目标集合只含 {6} 或 {8} 视为"只瞄建筑"，此时 front 用 ARMOR_OUTPOST_BASE，
      // 否则用 ARMOR_ALL（含机器人+建筑）。
      const bool building_only = front_target_ids == std::vector<int8_t>{6} ||
        front_target_ids == std::vector<int8_t>{8};
      const uint8_t front_armor_target = building_only
                                           ? rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE
                                           : rm_msgs::StatusChangeRequest::ARMOR_ALL;
      const uint8_t side_armor_target = rm_msgs::StatusChangeRequest::ARMOR_WITHOUT_OUTPOST_BASE;
      auto* front = &dx_track_switch_caller_;
      const bool front_ids_changed = front != nullptr && front->setExplicitArmorTargetIds(front_target_ids);
      bool front_updated = false;
      if (front != nullptr && !front->isCalling() &&
        (front_ids_changed || front->getTarget() != desired_target || front->getArmorTarget() != front_armor_target))
      {
        front->setTargetType(desired_target);
        front->setArmorTargetType(front_armor_target);
        front->callService();
        front_updated = true;
      }
      const bool updated = front_updated || sync(switch_detection_left_srv_.get(), side_armor_target) ||
        sync(switch_detection_right_srv_.get(), side_armor_target);
      if (updated)
        ROS_INFO("[Vision] front armor target -> %s", mode_name);
    }

    /**@brief 核心状态机，每次调用推进一次目标切换
     *@param target 目标id
     *@param now 当前时刻
     * **/
    Result requestTarget(const uint8_t target, const ros::Time& now)
    {
      if (target != desired_target_) //传入的目标和存储的desired_target不一致的情况
      {
        desired_target_ = target;
        if (call_in_flight_)
          restart_after_call_ = true;
        else
          startTransition(target, now);
      }
      else if (!transition_active_ && effective_target_ != target) //目标一致但是没有正在活动的切换任务并且已生效的目标不为传入的目标的情况
        startTransition(target, now);

      if (!transition_active_)
        return effective_target_ == target ? Result::Ready : Result::Switching;

      const auto order = serviceOrder(transition_target_);
      const Service service = order[step_index_];
      if (call_in_flight_)
      {
        bool still_calling = false;
        bool succeeded = false;
        withService(service, [&](auto* caller)
        {
          still_calling = caller->isCalling();
          if (!still_calling)
            succeeded = caller->getIsSwitch();
        });
        if (still_calling)
          return Result::Switching;

        call_in_flight_ = false;
        if (restart_after_call_)
        {
          startTransition(desired_target_, now);
          return Result::Switching;
        }
        if (succeeded) //成功获取到返回结果
        {
          attempt_count_ = 0;
          if (++step_index_ == order.size()) //全部服务请求完毕，并在这里执行step_index_的递增
          {
            effective_target_ = transition_target_;
            transition_active_ = false;
            const char* target_name = transition_target_ == rm_msgs::StatusChangeRequest::SMALL_BUFF
                                        ? "SMALL_BUFF"
                                        : transition_target_ == rm_msgs::StatusChangeRequest::BIG_BUFF
                                        ? "BIG_BUFF"
                                        : "ARMOR";
            ROS_INFO("[Vision] four-service target switch completed: %s", target_name);
            return Result::Ready;
          }
        }
        else //还未获得返回结果
        {
          ++attempt_count_;
          int fail_limit = 10;
          withService(service, [&](auto* caller) { fail_limit = caller->getFailLimit(); });
          if (fail_limit <= 0)
            fail_limit = 10;
          if (attempt_count_ >= fail_limit) //超过失败次数
          {
            if (isBuffTarget(transition_target_))
            {
              ROS_ERROR("[Vision] Buff chain switch failed; restoring Armor chain.");
              buff_entry_failed_ = true;
              effective_target_ = kUnknownTarget;
              startTransition(rm_msgs::StatusChangeRequest::ARMOR, now);
              return Result::Failed;
            }
            ROS_ERROR_THROTTLE(2.0, "[Vision] Armor chain restore still failing; retrying.");
            attempt_count_ = 0;
          }
          retry_not_before_ = now + ros::Duration(retry_interval_sec_);
          return Result::Switching;
        }
      }

      if (now < retry_not_before_)
        return Result::Switching;

      const Service next_service = serviceOrder(transition_target_)[step_index_];
      bool busy = false;
      withService(next_service, [&](auto* caller) { busy = caller->isCalling(); });
      if (busy)
        return Result::Switching;

      effective_target_ = kUnknownTarget;
      withService(next_service, [&](auto* caller)
      {
        caller->setTargetType(serviceTarget(next_service, transition_target_));
        caller->resetSwitchResult();
        caller->callService();
      });
      call_in_flight_ = true;
      return Result::Switching;
    }

    const uint8_t kInternalOutpostId = 6; // 行为树内部：前哨 id
    const uint8_t kInternalSentryId = 7; // 行为树内部：哨兵 id
    const uint8_t kLegacyDxTrackSentryId = 6; // 旧版 dx_track：哨兵上报 id（和内部前哨 6 冲突）
    const uint8_t kLegacyDxTrackOutpostId = 12; // 旧版 dx_track：前哨上报 id（打符也用它）

    static constexpr uint8_t kUnknownTarget = std::numeric_limits<uint8_t>::max();
    uint8_t desired_target_{rm_msgs::StatusChangeRequest::ARMOR}; // 期望的目标（外部请求）
    uint8_t transition_target_{rm_msgs::StatusChangeRequest::ARMOR}; // 正在切换的目标
    uint8_t effective_target_{rm_msgs::StatusChangeRequest::ARMOR}; // 已生效/确认的目标
    uint8_t last_shoot_frequency_{}; // 打符前记录的射频，退出时还原
    bool shooter_limited_{false}; // 是否已把射频压到 MINIMAL
    bool transition_active_{false}; // 是否有切换在推进
    bool call_in_flight_{false}; // 是否有一次服务调用在飞行中（未返回结果）
    bool restart_after_call_{false}; // 飞行中来了新目标，等本次调用结束后重启切换
    bool buff_entry_failed_{false}; // 上一次 buff 链切换失败（下次进入直接退回装甲）
    std::size_t step_index_{0}; // 当前推进到第几个服务（0..3）
    int attempt_count_{0}; // 当前服务失败重试计数
    double retry_interval_sec_{0.2}; // 失败重试间隔
    ros::Time retry_not_before_; // 在此时刻之前不做下一次重试

    std::unique_ptr<rm_common::SwitchDetectionCaller> switch_detection_left_srv_;
    std::unique_ptr<rm_common::SwitchDetectionCaller> switch_detection_right_srv_;
    std::unique_ptr<rm_common::SwitchDetectionCaller> switch_buff_srv_;
    std::unique_ptr<rm_common::SwitchDetectionCaller> switch_buff_type_srv_;
    std::unique_ptr<rm_common::SwitchDetectionCaller> switch_exposure_srv_;
    auto_aim::DxTrackSwitchCaller &dx_track_switch_caller_;

    ros::NodeHandle &bt_nh_;
    CmdTools &cmd_tools_;
  };
}


#endif //NEW_BEHAVIOR_TREE_TOOLS_H
