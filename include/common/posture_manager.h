#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp/blackboard.h>
#include "types.h"

namespace posture
{
  enum class PostureMode : uint8_t
  {
    Move = 0,
    Attack = 1,
    Defense = 2
  };

  enum class TacticalIntent : uint8_t
  {
    Transit,
    Engage,
    Hold,
    Recover,
    DefendBase,
    AttackObjective
  };

  struct PostureContext  //用于描述机器人所处的状态
  {
    bool in_battle{ false };
    bool is_dead{ false };
    double present_time {0};
    bool is_in_road_area {false};
    bool operator_request_valid{ false };
    PostureMode operator_requested{ PostureMode::Move };
    TacticalIntent tactical_intent{ TacticalIntent::Transit };
    int remain_hp{ 0 };
    int max_hp{ 0 };
    int bullet_17{ 0 };
    bool low_hp{ false };
  };

  struct PostureState  //姿态状态机
  {
    // 2026 姿态策略状态：current 为已提交姿态，desired 为本 tick 计算结果，cooldown/force 用于抑制频繁切换。
    PostureMode current{ PostureMode::Move }; //当前所处姿态
    PostureMode desired{ PostureMode::Move }; //期望姿态
    PostureMode last_committed{ PostureMode::Move }; //上次切换前的姿态
    ros::Time last_switch_time{ 0.0 }; //上次切换时间
    ros::Time time_entered_current_mode_{ 0.0 }; //进入当前体姿态的shike 时刻
    ros::Time track_enemy_enter_time{ 0.0 }; //追踪到敌人的时间
    ros::Time track_enemy_attack_hold_until_time{ 0.0 }; //退出追踪敌人的时候attack姿退出attack姿态的时间戳时刻
    std::array<double, 3> accumulated_sec{ { 0.0, 0.0, 0.0 } }; //各姿态的累计持续时间
    std::array<bool, 3> decayed{ { false, false, false } }; //各姿态是否已经衰减
    bool cooldown_active{ false }; //是否在切换冷却时间中
    bool force_locked{ false }; //是否被强制锁定
    bool was_track_enemy_active{ false };  //上一帧是否是tr
  };

  struct PostureEffect  //姿态影响
  {
    // 姿态对热量、底盘功率和受击倍率的策略系数；实际裁判协议编码在 PostureManager 内完成。
    double heat_cooldown_ratio{ 1.0 };
    double power_limit_ratio{ 1.0 };
    double defense_ratio{ 1.0 };
    double vulnerability_ratio{ 1.0 };
  };

  class PostureManager
  {
  public:
    explicit PostureManager(ros::NodeHandle& nh , BT::Blackboard &blackboard);

    /**@brief 根据当前姿态计算出所处姿态对于机器人性能的影响
     * **/
    void makeEffect();

    void update(const PostureContext& ctx);
    [[nodiscard]] PostureMode decideDesired(const PostureContext& ctx) const;
    [[nodiscard]] bool canSwitch(PostureMode from, PostureMode to) const;
    void fillSentryCmd() const;

  private:
    static constexpr std::size_t postureIndex(const PostureMode mode)
    {
      return static_cast<std::size_t>(mode);
    }

    PostureEffect posture_effect_;
    PostureState posture_state_;
    double switch_cooldown_sec_{ 5.0 };
    double posture_decay_threshold_sec_{ 180.0 };
    double track_enemy_attack_delay_sec_{ 0.0 }; //跟踪到替人敌人后切attack模式的延迟时间
    double track_enemy_attack_hold_after_exit_sec_{ 5.0 };
    BT::Blackboard &blackboard_;
    int gimbal_mode_{};
    int chassis_mode_{};
  };
}