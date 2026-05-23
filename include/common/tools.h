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

namespace perception
{
  class Publisher;
  class Subscriber;
  class TfAccessor;
}

namespace tools
{
  class EnableGyroServiceCaller : public rm_common::ServiceCallerBase<rm_msgs::EnableGyro>
  {
  public:
    explicit EnableGyroServiceCaller(ros::NodeHandle& nh);

    void setGyro(double gyro_speed);

    bool isGyro();

    void enable();
  };

  class SetLimitVelServiceCaller : public rm_common::ServiceCallerBase<rm_msgs::SetLimitVel>
  {
  public:
    explicit SetLimitVelServiceCaller(ros::NodeHandle& nh,const double init_limit_vel);

    void setLimitVel(const double& limit_vel);

    void setSlideWindow(const double slide_window);

    double getLimitVel() const;
  };

  class CmdTools
  {
  public:
    struct Senders
    {
      std::unique_ptr<rm_common::ChassisCommandSender> chassis_command_sender_;
      std::unique_ptr<rm_common::GimbalCommandSender> gimbal_command_sender_;
      std::unique_ptr<rm_common::GimbalCommandSender> base_gimbal_command_sender_;
      std::unique_ptr<rm_common::ShooterCommandSender> shooter_command_sender_;
      std::unique_ptr<rm_common::Vel2DCommandSender> vel_2d_command_sender_;
    };

    CmdTools(ros::NodeHandle& nh , BT::Blackboard &blackboard);

    Senders * getSenders() const;

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
    BT::Blackboard &blackboard_;
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

  geometry_msgs::PoseStamped getZonesPosition(const std::string& area_name,BT::Blackboard &blackboard,int &last_patrol_position_index , bool sequential_patrol_enable , bool & is_complete);

  bool isPointInPolygon(const geometry_msgs::TransformStamped& point,
                        const std::vector<geometry_msgs::PointStamped>& polygon);

  std::string determinePolygonInWhich(const geometry_msgs::TransformStamped& point , std::unordered_map<std::string,std::vector<geometry_msgs::PointStamped>> pos_detection_polygons);

  class MiniMapTools
  {
  public:
    MiniMapTools(BT::Blackboard &blackboard , perception::Publisher &publisher , perception::Subscriber &subscriber);

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
    [[nodiscard]]geometry_msgs::PoseStamped getConductPoint();

  private:
    BT::Blackboard & blackboard_;
    tf2::Transform minimap2world_;
    double last_point_x_ = 0;
    double last_point_y_ = 0;
    perception::Publisher &publisher_;
    perception::Subscriber &subscriber_;
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

    NavigationTools(BT::Blackboard &blackboard , perception::Subscriber &subscriber ,perception::TfAccessor &tf_viewer, CmdTools &cmd_tools);

    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* getMbfClient() const;

    /**@brief 用于点位巡航，通过mbf实现
     *@param point 导航点，常通过getPatrolPoint获得
     *@param residence_time_at_point 在单个点中的停留时间
     *@param is_conduct_mode 是否为云台手指引的点位，该参数影响client map update状态的维护
     * **/
    void patrol(const geometry_msgs::PoseStamped& point, double residence_time_at_point, bool is_conduct_mode);

    /**@brief 用于从all_zones中获取指定patrol_area_name中的巡航点，该函数维护patrol_sequential_index_，
     *通过更改index的方式实现多点巡航
     *@param patrol_area_name 巡航区域名字
     *@param sequential_patrol_enable 是否开启顺序巡航，若为false，则为随机点位巡航
     * **/
    geometry_msgs::PoseStamped getPatrolPoint(const std::string& patrol_area_name ,const bool sequential_patrol_enable);

    /**@brief 用于在打断原patrol的时候重置参数
     * **/
    void resetPatrolState();

    /**@brief 用于获取Track所标记的目标在地图上面的位置，并给mbf发送该位置坐标用于追击敌人
     * **/
    bool chase();

    /**@brief 用于在退出追击逻辑的时候重置记录的目标在地图上面的位置
     * **/
    void resetLastTargetAtMap();

  private:
    bool checkMbfClientState();

    void resetMbfClient();

    void reachGoalJudgement(const actionlib::SimpleClientGoalState& state , const mbf_msgs::MoveBaseResultConstPtr& result);


    BT::Blackboard &blackboard_;
    perception::Subscriber &subscriber_;
    perception::TfAccessor &tf_accessor_;
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
    CmdTools &cmd_tools_;
    int patrol_sequential_index_ = -1;
    std::string last_patrol_area_name_{};
    PatrolState patrol_state_ = PatrolState::IDLE;
    std::unordered_map<std::string,std::vector<geometry_msgs::PoseStamped>> all_zones;
    geometry_msgs::PointStamped track_point_;
    geometry_msgs::PointStamped last_target_at_map_;
    ros::ServiceClient service_client_;
  };

  class ControllerTools
  {
  public:
    ControllerTools(ros::NodeHandle &bt_nh);

    [[nodiscard]]rm_common::ControllerManager * getControllerManager() const;

    void calibrate();

    /** @brief 用于刷新控制器启停管理器，该函数需要在主循环线程中调用
     * **/
    void ControllerUpdate();

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
    ros::NodeHandle &bt_nh_;
    std::unique_ptr<rm_common::ControllerManager> controller_manager_;
    XmlRpc::XmlRpcValue shooter_calibration_config_;
    std::unique_ptr<rm_common::CalibrationQueue> shooter_calibration_queue_;
    std::vector<std::string> main_controllers_;
    std::vector<std::string> calibration_controllers_;
  };

  class GimbalTools
  {
  public:
    GimbalTools(perception::TfAccessor &tf_accessor , CmdTools &cmd_tools , ros::NodeHandle &bt_nh);

    /**@brief 更新pitch轴扫描的pitch指向
     *@param min_angel pitch轴指向的最小角度
     *@param max_angle pitch轴指向的最大角度
     *@param pitch_outside_vel 严重超出给定pitch最大最小角度的pitch回复速度
     *@param pitch_inside_vel 未超出pitch最大最小角度的pitch回复速度
     *@param breach_threshold 判定pitch严重超出给定pitch最大最小角度的阈值
     *
     * **/
    void updatePitchStrafeDirect(double min_angel , double max_angle , double pitch_outside_vel , double pitch_inside_vel, double breach_threshold);

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
    void lidarTwist(double yaw_vel , double scan_range_circles);

    /**@brief 该函数用于让云台指向map坐标系下的某个点
     *@param point_of_map map坐标系下的点坐标
     * **/
    void setGimbalDirectPoint(geometry_msgs::PointStamped point_of_map);

    /**@brief 用于给云台设置为track模式，并填充cmdGimbal中的bullet_speed字段
     * **/
    void setStackGimbalTrack();

  private:
    perception::TfAccessor &tf_accessor_;
    CmdTools &cmd_tools_;
    double pitch_direct_{};
    double yaw_direct_{};
    double traj_pitch_{};
    double max_pitch_angle_{};
    double min_pitch_angle_{};
    double circle_count_{};
    double lidar_twist_last_yaw_{};
    ros::NodeHandle &bt_nh;
  };
}


#endif //NEW_BEHAVIOR_TREE_TOOLS_H