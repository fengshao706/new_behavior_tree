//
// Created by root on 2026/8/25.
//

#ifndef NEW_BEHAVIOR_TREE_INVINCIBLE_DETECTION_H
#define NEW_BEHAVIOR_TREE_INVINCIBLE_DETECTION_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <rm_msgs/TrackData.h>
#include <rm_msgs/RadarWirelessEnemyRobotPos.h>

#include "perception_layer.h"
#include "common/tools.h"

namespace invincible_detection
{
  enum class EnemyInvincibleState
  {
    UNKNOWN, // 尚无任何确认信息（雷达 HP 一帧都还没来，或 index 越界）
    ALIVE, // 已确认存活（连续 confirm_samples 帧 HP>0）
    DEAD, // 已确认阵亡（连续 confirm_samples 帧 HP<=0）
    REVIVE_INVINCIBLE, // 检测到 ALIVE->DEAD->ALIVE 完整序列，进入复活无敌期（打了不扣血）
    REGION_INVINCIBLE // 目标在敌方补给区 / 工程在交换区，规则上不可攻击
  };

  /**@brief 无敌状态枚举值转字符串，供日志输出使用
   * **/
  inline const char* enemyAttackabilityReasonToString(const EnemyInvincibleState reason)
  {
    switch (reason)
    {
    case EnemyInvincibleState::ALIVE:
      return "alive";
    case EnemyInvincibleState::UNKNOWN:
      return "unknown";
    case EnemyInvincibleState::DEAD:
      return "dead";
    case EnemyInvincibleState::REVIVE_INVINCIBLE:
      return "revive_invincible";
    case EnemyInvincibleState::REGION_INVINCIBLE:
      return "region_invincible";
    default:
      return "unknown";
    }
  }

  struct EnemyLifeSnapshot
  {
    EnemyInvincibleState state{EnemyInvincibleState::UNKNOWN}; // 去抖后的生命状态
    ros::Time revive_invincible_until; // 复活无敌截止时刻（未复活时为零）
    int hp{0}; // 最近一帧 HP（已下限钳到 0）
  };

  class EnemyInvincibilityManager
  {
  public:
    struct Config
    {
      double source_timeout_sec{1.0}; // 相邻 HP 帧允许的最大间隔；超过视为一次长中断，重开一轮去抖
      int confirm_samples{2}; // 同一方向连续 N 帧才确认状态切换（去抖窗口宽度）
      double normal_revive_invincible_sec{10.0}; // 普通机器人复活无敌时长
      double sentry_revive_invincible_sec{60.0}; // 敌方哨兵复活无敌时长
    };

    /**@brief 构造：注入区域判定、坐标转换与数据源。
     *@param navigation_tools 区域判定（determinePolygonInWhich）
     *@param mini_map_tools 坐标转换（小地图系 → map 系）
     *@param tf_accessor 坐标转换（track 相机系 → map 系）
     *@param subscriber 订阅器（track / 雷达位置消息源）
     *@param robot_color 己方颜色（"red"/"blue"），用于命中交换区区域名 "{color}_engineer_invincible_area"
     *@param robot_count 机器人下标槽位数（默认 8，兼容原 HP 链）
     * **/
    explicit EnemyInvincibilityManager(tools::NavigationTools& navigation_tools, tools::MiniMapTools& mini_map_tools,
                                       perception::TfAccessor& tf_accessor,
                                       perception::Subscriber& subscriber, std::string robot_color,
                                       const std::size_t robot_count = 8)
      : states_(robot_count), navigation_tools_(navigation_tools), mini_map_tools_(mini_map_tools),
        tf_accessor_(tf_accessor), subscriber_(subscriber), robot_color_(std::move(robot_color))
    {
    }

    /**@brief 将存储RobotState的vector容器内的所有数据清空并重新初始化
     * **/
    void reset()
    {
      states_.assign(states_.size(), RobotState{});
    }

    /**@brief 对一个机器人的当前状态（存活，死亡，复活后无敌）做观测和记录，该函数应在雷达帧到来时被调用
     *@param robot_index 机器人下标序号
     *@param hp 机器人当前血量
     *@param is_sentry 是否是哨兵
     *@param receive_stamp 接收到消息时的时间戳
     *@param now 当前时刻
     * **/
    void lifeObserve(const std::size_t robot_index, const int hp, const bool is_sentry, const ros::Time& receive_stamp,
                     const ros::Time& now)
    {
      // 越界 / 无时间戳的帧直接丢弃。
      if (robot_index >= states_.size() || receive_stamp.isZero())
        return;

      RobotState& state = states_[robot_index];
      // 单调性检查：重复帧 / 乱序帧 / 时间回跳一律丢弃，保证一帧最多消费一次。
      if (!state.last_receive_stamp.isZero() && receive_stamp <= state.last_receive_stamp)
        return;

      // 距上一帧超过 source_timeout_sec：视为一次长中断，重开一轮去抖。
      // 注意只清"候选"，不清"已确认的 life_state"——断流期间的确认结果被刻意保留。
      if (!state.last_receive_stamp.isZero() &&
        receive_stamp - state.last_receive_stamp > ros::Duration(config_.source_timeout_sec))
      {
        // 重开一轮去抖：不丢弃已确认状态；断流后第一帧独苗不应翻转 ALIVE/DEAD，
        // 但连续两帧新鲜数据仍足以确认一次真实的状态切换。
        state.has_candidate = false;
        state.candidate_count = 0;
      }

      state.last_receive_stamp = receive_stamp;
      state.hp = std::max(0, hp); // 血量下限钳制，负血量一律按 0 处理
      const bool alive_sample = hp > 0;

      // 候选计数：当前帧存活方向与候选一致就累加，方向反转则重新起候选。
      // "候选" = 未经确认的潜在线生/死切换；攒够 confirm_samples 帧才落地为确认状态。
      if (!state.has_candidate || state.candidate_alive != alive_sample)
      {
        state.has_candidate = true;
        state.candidate_alive = alive_sample;
        state.candidate_count = 1;
      }
      else
      {
        ++state.candidate_count;
      }

      // 帧数未攒够，暂不改变已确认状态。
      if (state.candidate_count < config_.confirm_samples)
        return;

      // ---- 以下为"确认"处理：候选已攒够，采纳当前帧方向 ----
      if (alive_sample)
      {
        // 复活判定：此前已确认阵亡(DEAD) 且 死前确认过存活(has_been_alive)。
        // 只有完整走完 ALIVE -> DEAD -> ALIVE 才可能是真正复活，进入无敌期。
        if (state.life_state == EnemyInvincibleState::DEAD && state.has_been_alive == true)
        {
          state.life_state = EnemyInvincibleState::REVIVE_INVINCIBLE;
          const double duration = is_sentry
                                    ? config_.sentry_revive_invincible_sec
                                    : config_.normal_revive_invincible_sec;
          state.revive_invincible_until = now + ros::Duration(duration);
        }
        else if (state.life_state != EnemyInvincibleState::REVIVE_INVINCIBLE || now >= state.revive_invincible_until)
        {
          state.life_state = EnemyInvincibleState::ALIVE;
          state.has_been_alive = true;
        }
      }
      else
      {
        state.life_state = EnemyInvincibleState::DEAD;
      }
    }

    /**@brief 向外提供查询机器人状态（存活，死亡，复活后无敌，区域无敌）的接口
     *@param robot_index 机器人下标 / 角色序号（1=英雄 2=工程 3=步兵三 4=步兵四 5=无人机 6=哨兵）
     *@param now 当前时刻
     * **/
    [[nodiscard]] EnemyLifeSnapshot snapshot(const std::size_t robot_index, const ros::Time& now)
    {
      updateEnemyPositions(); // 每次查询前先自刷坐标槽（track / 雷达竞争结果）
      const std::string enemy_color = (robot_color_ == "red") ? "blue" : "red";

      // 仅 1~6 是合法角色序号，其余返回全未知。
      if (robot_index < 1 || robot_index > 6)
        return {};

      // ---- 生命状态：从 HP 链 states_ 取（RobotState，与位置字段互不合并）----
      EnemyLifeSnapshot result;
      if (robot_index < states_.size())
      {
        const RobotState& state = states_[robot_index];
        // 一帧都没收到过 -> 未知。
        if (!state.last_receive_stamp.isZero())
        {
          result.state = state.life_state;
          // 复活无敌期已到点：快照层面回落到 ALIVE（放行开火）
          if (result.state == EnemyInvincibleState::REVIVE_INVINCIBLE && now >= state.revive_invincible_until)
            result.state = EnemyInvincibleState::ALIVE;
          result.revive_invincible_until = state.revive_invincible_until;
          result.hp = state.hp;
        }
      }

      // ---- 位置无敌性：对该角色坐标做逐目标判定 ----
      EnemyPos enemy_pos;
      switch (robot_index)
      {
      case 1: enemy_pos = hero_;
        break;
      case 2: enemy_pos = engineer_;
        break;
      case 3: enemy_pos = standard3_;
        break;
      case 4: enemy_pos = standard4_;
        break;
      case 5: enemy_pos = aerial_;
        break;
      default: enemy_pos = sentry_; // 6 及越界兜底（snapshot 已拦 1~6）
      }

      // 敌方补给区：任何角色落入 enemy_supply_area -> REGION_INVINCIBLE。
      const std::string supply_area = enemy_color + "_supply_area";
      if (enemy_pos.valid && navigation_tools_.determinePolygonInWhich(enemy_pos.pos) == supply_area)
        result.state = EnemyInvincibleState::REGION_INVINCIBLE;

        // 工程交换区：仅工程机器人在 enemy_engineer_invincible_area -> REGION_INVINCIBLE。
      else if (robot_index == 2 && enemy_pos.valid)
      {
        if (navigation_tools_.determinePolygonInWhich(enemy_pos.pos) == enemy_color + "_engineer_invincible_area")
        {
          result.state = EnemyInvincibleState::REGION_INVINCIBLE;
        }
      }

      return result;
    }

    /**@brief 更新每个敌方机器人的 map 坐标（需要在每个决策周期、进入追击门判定的节点前调用一遍）
     * **/
    void updateEnemyPositions()
    {
      const auto& track_ret = subscriber_.msgGetter<rm_msgs::TrackData>(perception::Subscriber::TopicId::TRACK_DATA);
      const auto& radar_ret = subscriber_.msgGetter<rm_msgs::RadarWirelessEnemyRobotPos>(
        perception::Subscriber::TopicId::RADAR_WIRELESS_ENEMY_ROBOT_POS);
      const rm_msgs::RadarWirelessEnemyRobotPos& msg = radar_ret.message;

      // ---- track 更新：仅真正锁敌且 id 合法(1~6)时参与竞争 ----
      if (track_ret.message.tracking && track_ret.message.id >= 1 && track_ret.message.id <= 6)
      {
        const auto track_to_map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP,
                                                              perception::TfAccessor::FrameId::TRACK);

        // id→角色映射（1=hero 2=engineer 3=infantry_3 4=infantry_4 5=aerial 6=sentry）。
        // 每条先看时间戳是否更新再写
        if (track_ret.message.id == 1 && (!hero_.valid || track_ret.stamp > hero_.stamp))
        {
          hero_.pos.x = track_to_map.transform.translation.x;
          hero_.pos.y = track_to_map.transform.translation.y;
          hero_.pos.z = 0.0;
          hero_.stamp = track_ret.stamp;
          hero_.valid = true;
        }
        else if (track_ret.message.id == 2 && (!engineer_.valid || track_ret.stamp > engineer_.stamp))
        {
          engineer_.pos.x = track_to_map.transform.translation.x;
          engineer_.pos.y = track_to_map.transform.translation.y;
          engineer_.pos.z = 0.0;
          engineer_.stamp = track_ret.stamp;
          engineer_.valid = true;
        }
        else if (track_ret.message.id == 3 && (!standard3_.valid || track_ret.stamp > standard3_.stamp))
        {
          standard3_.pos.x = track_to_map.transform.translation.x;
          standard3_.pos.y = track_to_map.transform.translation.y;
          standard3_.pos.z = 0.0;
          standard3_.stamp = track_ret.stamp;
          standard3_.valid = true;
        }
        else if (track_ret.message.id == 4 && (!standard4_.valid || track_ret.stamp > standard4_.stamp))
        {
          standard4_.pos.x = track_to_map.transform.translation.x;
          standard4_.pos.y = track_to_map.transform.translation.y;
          standard4_.pos.z = 0.0;
          standard4_.stamp = track_ret.stamp;
          standard4_.valid = true;
        }
        else if (track_ret.message.id == 5 && (!aerial_.valid || track_ret.stamp > aerial_.stamp))
        {
          aerial_.pos.x = track_to_map.transform.translation.x;
          aerial_.pos.y = track_to_map.transform.translation.y;
          aerial_.pos.z = 0.0;
          aerial_.stamp = track_ret.stamp;
          aerial_.valid = true;
        }
        else if (track_ret.message.id == 6 && (!sentry_.valid || track_ret.stamp > sentry_.stamp))
        {
          sentry_.pos.x = track_to_map.transform.translation.x;
          sentry_.pos.y = track_to_map.transform.translation.y;
          sentry_.pos.z = 0.0;
          sentry_.stamp = track_ret.stamp;
          sentry_.valid = true;
        }
      }

      // ---- 雷达更新：逐角色平铺，与 track 共享同一把锁；原始厘米需 ×0.01 转米 ----
      if (msg.hero_position_x || msg.hero_position_y)
      {
        if (!hero_.valid || radar_ret.stamp > hero_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.hero_position_x * 0.01f, msg.hero_position_y * 0.01f, &map_pose);
          hero_.pos = map_pose.pose.position;
          hero_.stamp = radar_ret.stamp;
          hero_.valid = true;
        }
      }
      if (msg.engineer_position_x || msg.engineer_position_y)
      {
        if (!engineer_.valid || radar_ret.stamp > engineer_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.engineer_position_x * 0.01f, msg.engineer_position_y * 0.01f,
                                              &map_pose);
          engineer_.pos = map_pose.pose.position;
          engineer_.stamp = radar_ret.stamp;
          engineer_.valid = true;
        }
      }
      if (msg.infantry_3_position_x || msg.infantry_3_position_y)
      {
        if (!standard3_.valid || radar_ret.stamp > standard3_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.infantry_3_position_x * 0.01f, msg.infantry_3_position_y * 0.01f,
                                              &map_pose);
          standard3_.pos = map_pose.pose.position;
          standard3_.stamp = radar_ret.stamp;
          standard3_.valid = true;
        }
      }
      if (msg.infantry_4_position_x || msg.infantry_4_position_y)
      {
        if (!standard4_.valid || radar_ret.stamp > standard4_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.infantry_4_position_x * 0.01f, msg.infantry_4_position_y * 0.01f,
                                              &map_pose);
          standard4_.pos = map_pose.pose.position;
          standard4_.stamp = radar_ret.stamp;
          standard4_.valid = true;
        }
      }
      if (msg.aerial_position_x || msg.aerial_position_y)
      {
        if (!aerial_.valid || radar_ret.stamp > aerial_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.aerial_position_x * 0.01f, msg.aerial_position_y * 0.01f, &map_pose);
          aerial_.pos = map_pose.pose.position;
          aerial_.stamp = radar_ret.stamp;
          aerial_.valid = true;
        }
      }
      if (msg.sentry_position_x || msg.sentry_position_y)
      {
        if (!sentry_.valid || radar_ret.stamp > sentry_.stamp)
        {
          geometry_msgs::PoseStamped map_pose;
          mini_map_tools_.targetPoseTransform(msg.sentry_position_x * 0.01f, msg.sentry_position_y * 0.01f, &map_pose);
          sentry_.pos = map_pose.pose.position;
          sentry_.stamp = radar_ret.stamp;
          sentry_.valid = true;
        }
      }
    }

  private:
    struct RobotState
    {
      EnemyInvincibleState life_state{EnemyInvincibleState::UNKNOWN}; // 已确认的生命状态
      ros::Time revive_invincible_until; // 复活无敌截止时刻
      ros::Time last_receive_stamp; // 最近一帧接收时间，用于单调性与断流判断
      int hp{0}; // 最近一帧血量（已钳到 >=0）
      int candidate_count{0}; // 候选方向上已连续出现的帧数
      bool candidate_alive{false}; // 候选方向：true=存活候选，false=阵亡候选
      bool has_candidate{false}; // 是否已有进行中的候选
      bool has_been_alive{false}; // 复活判定的扳机保险：本纪元确认过存活才置位
    };

    struct EnemyPos // 单个敌方角色的敌位槽
    {
      geometry_msgs::Point pos; // map 米坐标
      ros::Time stamp; // 最近一次更新（接收时刻，两路统一）
      bool valid{false}; // 是否已有有效坐标
    };

    Config config_;
    std::vector<RobotState> states_;

    tools::NavigationTools& navigation_tools_; // 区域判定
    tools::MiniMapTools& mini_map_tools_; // 小地图系 → map 系坐标转换
    perception::TfAccessor& tf_accessor_; // track 相机系 → map 系坐标转换
    perception::Subscriber& subscriber_; // track / 雷达位置消息源
    std::string robot_color_; // 己方颜色，命中交换区区域名

    // 各角色敌位槽（雷达、track 竞争后都写这里）。
    EnemyPos hero_;
    EnemyPos engineer_;
    EnemyPos standard3_;
    EnemyPos standard4_;
    EnemyPos aerial_;
    EnemyPos sentry_;
  };
} // namespace invincible_detection

#endif //NEW_BEHAVIOR_TREE_INVINCIBLE_DETECTION_H
