#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>

namespace chase_policy
{
  struct ChaseRestrictedZoneConfig
  {
    std::string name;
    bool is_target_area;
    int begin_time;
    int end_time;
  };

  struct ChaseGateInput
  {
    bool enabled{false}; // 追击总开关（配置项 enable_chase）
    bool track_enemy{false}; // 云台是否正处于锁敌跟踪模式（GimbalMode::TrackEnemy）
    bool current_mode_is_chase{false}; // 底盘当前是否已是追击模式（last_chassis_mode_ == Chase）
    bool goal_active{false}; // 底层导航 move_base_flex 是否还有 ACTIVE / PENDING 的目标
    double last_track_time{0.0}; // 上一帧跟踪的时间
    double max_continuation_sec{0.0}; // 失跟踪后允许继续追击的时间窗（配置项 chasing_max_for_time，默认 10s）

    std::string target_area_name;
    std::string staying_area_name;
    double present_game_time{0.0};
    bool ally_in_target_area{false}; // 敌方目标所在区域内有至少一个存活的我方机器人（Hero/工程/步兵，
    // 需 HP>0 且位置落在同一区域）——追击需友军同区呼应
    bool target_attackable{false}; // 目标当前可攻击（在 target_fresh 基础上叠加场地硬禁/生命状态判定）
  };

  /**@brief 根据传入的状态结构体，判断是否可以追击
   *@param chase_gate_input 状态结构体
   *@param restricted_zone_configs 限制追击区域配置
   *@return true为追，false为不追
   * **/
  [[nodiscard]] inline bool chaseGateAllows(const ChaseGateInput& chase_gate_input ,const std::vector<ChaseRestrictedZoneConfig> &restricted_zone_configs)
  {
    const auto in_restriction_time_window = [](const double present_game_time, const ChaseRestrictedZoneConfig& config)
    {
      return present_game_time > config.begin_time && present_game_time < config.end_time;
    };

    bool target_area_allowed{true};
    for (const auto& config : restricted_zone_configs)
    {
      if (config.is_target_area && config.name == chase_gate_input.target_area_name)
      {
        target_area_allowed = !in_restriction_time_window(chase_gate_input.present_game_time, config);
        break;
      }
    }

    bool staying_area_allowed{true};
    for (const auto& config : restricted_zone_configs)
    {
      if (!config.is_target_area && config.name == chase_gate_input.staying_area_name)
      {
        staying_area_allowed = !in_restriction_time_window(chase_gate_input.present_game_time, config);
        break;
      }
    }

    if (!chase_gate_input.enabled) return false;
    if (!chase_gate_input.track_enemy &&
      !(chase_gate_input.current_mode_is_chase && chase_gate_input.goal_active &&
        ros::Time::now().toSec() - chase_gate_input.last_track_time >= 0.0 &&
        ros::Time::now().toSec() - chase_gate_input.last_track_time <= chase_gate_input.max_continuation_sec))
      return false;
    return target_area_allowed && staying_area_allowed &&
      chase_gate_input.target_attackable && chase_gate_input.ally_in_target_area;
  }
} // namespace chase_policy
