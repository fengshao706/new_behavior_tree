#include "common/posture_manager.h"

namespace posture
{
  PostureManager::PostureManager(ros::NodeHandle& nh , BT::Blackboard &blackboard) : blackboard_(blackboard)
  {
    ros::NodeHandle posture_nh(nh, "posture_manager");
    posture_nh.param("switch_cooldown_sec", switch_cooldown_sec_, 5.0);
    posture_nh.param("posture_decay_threshold_sec", posture_decay_threshold_sec_, 180.0);
    posture_nh.param("track_enemy_attack_delay_sec", track_enemy_attack_delay_sec_, 0.0);
    posture_nh.param("track_enemy_attack_hold_after_exit_sec", track_enemy_attack_hold_after_exit_sec_, 5.0);
  }

  void PostureManager::makeEffect()
  {
    if (posture_state_.current == PostureMode::Move)
    {
      posture_effect_.power_limit_ratio = posture_state_.decayed[0] ? 1.2 : 1.5;
      posture_effect_.vulnerability_ratio = 1.25;
      posture_effect_.heat_cooldown_ratio = 1.0 / 3.0;
    }else if (posture_state_.current == PostureMode::Attack)
    {
      posture_effect_.power_limit_ratio = 0.5;
      posture_effect_.vulnerability_ratio = 1.25;
      posture_effect_.heat_cooldown_ratio = posture_state_.decayed[1] ? 2.0 : 3.0;
    }else if (posture_state_.current == PostureMode::Defense)
    {
      posture_effect_.power_limit_ratio = 0.5;
      posture_effect_.defense_ratio = posture_state_.decayed[2] ? 1.25 : 1.5;
      posture_effect_.heat_cooldown_ratio = 1.0 / 3.0;
    }
  }

  bool isRoadArea(const std::string& area)
  {
    return area == "road" || area == "rolling_road_area";
  }

  void PostureManager::update(const PostureContext& ctx)
  {
    const ros::Time now = ros::Time::now();

    if (posture_state_.last_switch_time.isZero())
      posture_state_.last_switch_time = now;
    if (posture_state_.time_entered_current_mode_.isZero())
      posture_state_.time_entered_current_mode_ = now;

    if (blackboard_.get<int>("gimbal_mode",gimbal_mode_) == false)
    {
      ROS_ERROR("Posture manager can not access key name [gimbal mode] , default value is 0");
      gimbal_mode_ = 0;
    }
    if (blackboard_.get<int>("chassis_mode",chassis_mode_) == false)
    {
      ROS_ERROR("Posture manager can not access key name [chassis mode] , default value is 0");
      chassis_mode_ = 0;
    }

    const bool track_enemy_active = gimbal_mode_ == static_cast<int>(types::GimbalMode::TrackEnemy);
    if (track_enemy_active)
    {
      if (posture_state_.track_enemy_enter_time.isZero())
        posture_state_.track_enemy_enter_time = now;
    }
    else //未跟踪到敌人的情况
    {
      if (posture_state_.was_track_enemy_active && posture_state_.current == PostureMode::Attack)
      {
        posture_state_.track_enemy_attack_hold_until_time = now + ros::Duration(track_enemy_attack_hold_after_exit_sec_); //记录attack模式需要何时退出
      }
      posture_state_.track_enemy_enter_time = ros::Time(0);
    }
    posture_state_.was_track_enemy_active = track_enemy_active;

    posture_state_.desired = decideDesired(ctx); //TODO : 需要进一步检查

    // 用独立计时器累积当前姿态持续时间，不随冷却时钟重置
    const double elapsed = (now - posture_state_.time_entered_current_mode_).toSec();
    posture_state_.accumulated_sec[postureIndex(posture_state_.current)] = elapsed;
    posture_state_.decayed[postureIndex(posture_state_.current)] = elapsed >= posture_decay_threshold_sec_;

    posture_state_.cooldown_active = !canSwitch(posture_state_.current, posture_state_.desired);

    if (!posture_state_.force_locked && posture_state_.current != posture_state_.desired && !posture_state_.cooldown_active)
    {
      posture_state_.last_committed = posture_state_.current;
      posture_state_.current = posture_state_.desired;
      posture_state_.last_switch_time = now;
      posture_state_.time_entered_current_mode_ = now;  //在这里重置进入某个状态的定时器
      posture_state_.cooldown_active = false;
    }
    makeEffect();
    fillSentryCmd();
  }

  PostureMode PostureManager::decideDesired(const PostureContext& ctx) const
  {
    if (!ctx.in_battle || ctx.is_dead)
      return PostureMode::Move;

    if (ctx.operator_request_valid)
      return ctx.operator_requested;

    if (chassis_mode_ == static_cast<int>(types::ChassisMode::GotoHpReturnArea))
      return PostureMode::Move;
    if (gimbal_mode_ == static_cast<int>(types::GimbalMode::TrackEnemy))
    {
      if (posture_state_.current == PostureMode::Attack)
        return PostureMode::Attack;
      const auto& enter_time = posture_state_.track_enemy_enter_time;
      if (enter_time.isZero() || (ros::Time::now() - enter_time).toSec() < track_enemy_attack_delay_sec_)
        return PostureMode::Move;
      return PostureMode::Attack;
    }
    if (ctx.present_time < 20.0 || ctx.is_in_road_area) //开局和在崎岖路段的时候需要更大的底盘功率
      return PostureMode::Move;
    return PostureMode::Defense;
  }

  bool PostureManager::canSwitch(const PostureMode from, const PostureMode to) const
  {
    if (from == to)
      return true;
    if (from == PostureMode::Attack && to != PostureMode::Attack)//从attack转到其他姿态
    {
      const auto& hold_until = posture_state_.track_enemy_attack_hold_until_time;
      if (!hold_until.isZero() && ros::Time::now() < hold_until)
        return false;
    }
    if (posture_state_.force_locked)
      return false;
    if (chassis_mode_ == static_cast<int>(types::ChassisMode::GotoHpReturnArea) && to == PostureMode::Move)
      return true;
    if (posture_state_.last_switch_time.isZero())
      return true;
    return (ros::Time::now() - posture_state_.last_switch_time).toSec() >= switch_cooldown_sec_;
  }

  void PostureManager::fillSentryCmd() const
  {
    // 2026 协议：1=Attack, 2=Defense, 3=Move
    // PostureMode: Move=0, Attack=1, Defense=2
    static constexpr uint8_t kProtocolValue[] = {3, 1, 2};
    info.sentry_cmd_.posture_cmd = kProtocolValue[postureIndex(info.posture_state_.current)];
  }
} // namespace posture
