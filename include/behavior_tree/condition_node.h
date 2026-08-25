//
// Created by root on 2026/3/19.
//

#ifndef NEW_BEHAVIOR_TREE_CONDITION_NODE_H
#define NEW_BEHAVIOR_TREE_CONDITION_NODE_H

#include <behaviortree_cpp/condition_node.h>
#include <fstream>
#include "common/types.h"

#include "behaviortree_cpp/action_node.h"
#include "common/tools.h"

namespace condition_node
{
  class IsRefereeOnline : public BT::ConditionNode
  {
  public:
    IsRefereeOnline(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      ros::Time heat_data_stamp = subscriber_.msgGetter<rm_msgs::PowerHeatData>(perception::Subscriber::TopicId::POWER_HEAT_DATA).stamp;

      bool is_online = ros::Time::now() - heat_data_stamp < ros::Duration(0.3); //时间差小于0.3的时候视为referee online
      BT::NodeStatus status = is_online == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsGameInBattle : public BT::ConditionNode
  {
  public:
    IsGameInBattle(const std::string& name, const BT::NodeConfig& config,
                   perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool is_in_battle = subscriber_.msgGetter<rm_msgs::GameStatus>(perception::Subscriber::TopicId::GAME_STATUS).message.game_progress == rm_msgs::GameStatus::IN_BATTLE;
      BT::NodeStatus status = is_in_battle == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsClientMapUpdate : public BT::ConditionNode
  {
  public:
    IsClientMapUpdate(const std::string& name, const BT::NodeConfig& config,
                      perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::OutputPort<int>("chassis_mode")};
    }

    BT::NodeStatus tick() override
    {
      bool is_update = ros::Time::now() - subscriber_.msgGetter<rm_msgs::ClientMapSendData>(perception::Subscriber::TopicId::CLIENT_MAP_SEND_DATA).stamp < ros::Duration(0.5);
      if (is_update == true)
      {
        uint8_t client_map_data = subscriber_.msgGetter<rm_msgs::ClientMapSendData>(perception::Subscriber::TopicId::CLIENT_MAP_SEND_DATA).message.command_keyboard;
        switch (client_map_data)
        {
        case rm_msgs::ClientMapSendData::KEY_A :
          setOutput("chassis_mode",19);
          break;
        case rm_msgs::ClientMapSendData::KEY_S :
          setOutput("chassis_mode",14);
          break;
        case rm_msgs::ClientMapSendData::KEY_D :
          setOutput("chassis_mode",13);
          break;
        default :
          ROS_ERROR("input keyboard command is wrong in IsClientMapUpdate");
        }
      }
      BT::NodeStatus status = is_update ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsSentryHpUrgent : public BT::ConditionNode
  {
  public:
    IsSentryHpUrgent(const std::string& name, const BT::NodeConfig& config,
                     perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("trigger_blood_return_hp")};
    }

    BT::NodeStatus tick() override
    {
      int trigger_hp;
      if (!getInput("trigger_blood_return_hp", trigger_hp))
      {
        ROS_ERROR("BT can not access key name [trigger_blood_return_hp] , default value is 30");
        trigger_hp = 30;
      }

      uint16_t remain_hp = subscriber_.msgGetter<rm_msgs::GameRobotStatus>(perception::Subscriber::TopicId::GAME_ROBOT_STATUS).message.remain_hp;

      bool is_urgent = remain_hp < trigger_hp;
      BT::NodeStatus status = is_urgent == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsNeedAvoidDrone : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsNeedAvoidDrone(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsNeedDefenseBase : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsNeedDefenseBase(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsTimeRangeCondition : public BT::ConditionNode
  {
  public:
    IsTimeRangeCondition(const std::string& name, const BT::NodeConfig& config,
                         perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("min_time"),
        BT::InputPort<double>("max_time"),
        BT::InputPort<double>("game_total_time")
      };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<double> min_time = getInput<double>("min_time");
      BT::Expected<double> max_time = getInput<double>("max_time");
      double game_total_time;
      if (!getInput("game_total_time", game_total_time))
      {
        ROS_ERROR("BT can not access key name [game_total_time] , default value is 420.0");
        game_total_time = 420.0;
      }
      double present_time = game_total_time - subscriber_.msgGetter<rm_msgs::GameStatus>(perception::Subscriber::TopicId::GAME_STATUS).message.stage_remain_time;

      if (present_time >= min_time.value() && present_time <= max_time.value())
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsDefenseBuffBelowTheThreshold : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsDefenseBuffBelowTheThreshold(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("defense_buff_threshold")};
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsOwnOutpostHpBeyondTheValue : public BT::ConditionNode
  {
  public:
    IsOwnOutpostHpBeyondTheValue(const std::string& name, const BT::NodeConfig& config,
                                 perception::Subscriber& subscriber) : ConditionNode(name, config),
                                                                       subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("outpost_hp_threshold")};
    }

    BT::NodeStatus tick() override
    {
      int threshold;
      if (!getInput("outpost_hp_threshold", threshold))
      {
        ROS_ERROR("BT can not access key name [outpost_hp_threshold] , default value is 800");
        threshold = 800;
      }
      BT::NodeStatus status = subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).message.ally_outpost_hp > threshold
                                ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
      return status;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsInOwnHalfArea : public BT::ConditionNode //TODO : 未完成逻辑
  {
  public:
    IsInOwnHalfArea(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsBulletsRemain : public BT::ConditionNode
  {
  public:
    IsBulletsRemain(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      uint16_t remain_bullet = subscriber_.msgGetter<rm_msgs::BulletAllowance>(perception::Subscriber::TopicId::BULLET_ALLOWANCE).message.bullet_allowance_num_17_mm;
      if (remain_bullet > 0 &&
        remain_bullet < 2000 == true)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class CheckTargetType : public BT::ConditionNode //若探测到的目标与给定的目标相同，则返回success，否则返回failure
  {
  public:
    CheckTargetType(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("track_id")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> track_id = getInput<int>("track_id");

      if (track_id.value() == subscriber_.msgGetter<rm_msgs::TrackData>(perception::Subscriber::TopicId::TRACK_DATA).message.id)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsHasEngineerMarked : public BT::ConditionNode
  {
  public:
    IsHasEngineerMarked(const std::string& name, const BT::NodeConfig& config,
                        perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      bool has_engineer_marked = subscriber_.msgGetter<rm_msgs::RadarToSentry>(perception::Subscriber::TopicId::RADAR_TO_SENTRY_DATA).message.engineer_marked;
      return has_engineer_marked == true ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsEngineerAlive : public BT::ConditionNode
  {
  public:
    IsEngineerAlive(const std::string& name, const BT::NodeConfig& config,
                    perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).message.ally_2_robot_hp > 0)
        return BT::NodeStatus::SUCCESS;
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsHeroInTrapezoid : public BT::ConditionNode
  {
  public:
    IsHeroInTrapezoid(const std::string &name , const BT::NodeConfig &config , perception::Subscriber &subscriber , tools::MiniMapTools &mini_map_tools , tools::NavigationTools &navigation_tools) : ConditionNode(name,config) , subscriber_(subscriber) , mini_map_tools_(mini_map_tools) , navigation_tools_(navigation_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      geometry_msgs::PoseStamped hero_pose;
      geometry_msgs::Point hero_point;
      mini_map_tools_.targetPoseTransform(subscriber_.msgGetter<rm_msgs::RobotsPositionData>(perception::Subscriber::TopicId::ROBOT_POSITION).message.hero_x,subscriber_.msgGetter<rm_msgs::RobotsPositionData>(perception::Subscriber::TopicId::ROBOT_POSITION).message.hero_y,&hero_pose);
      hero_point.x = hero_pose.pose.position.x;
      hero_point.y = hero_pose.pose.position.y;
      std::string area_name = navigation_tools_.determinePolygonInWhich(hero_point);
      if (area_name == "trapezoid_area")
      {
        return BT::NodeStatus::SUCCESS;
      }else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
    perception::Subscriber &subscriber_;
    tools::MiniMapTools &mini_map_tools_;
    tools::NavigationTools &navigation_tools_;
  };

  class IsOwnFortressBeenCap : public BT::ConditionNode
  {
  public:
    IsOwnFortressBeenCap(const std::string &name ,const BT::NodeConfig &config , perception::Subscriber &subscriber) : ConditionNode(name , config) , subscriber_(subscriber)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("capture_status")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> input_value =  getInput<int>("capture_status");
      int is_our_robot_capture = input_value.value();
      uint8_t fortress_point_state = subscriber_.msgGetter<rm_msgs::EventData>(perception::Subscriber::TopicId::EVENT_DATA).message.fortress_point_state;
      if (is_our_robot_capture == 1)//被己方占领
      {
        if (fortress_point_state == 1)
        {
          return BT::NodeStatus::SUCCESS;
        }else
        {
          return BT::NodeStatus::FAILURE;
        }
      }else if (is_our_robot_capture == 2)//被对方占领
      {
        if (fortress_point_state == 2)
        {
          return BT::NodeStatus::SUCCESS;
        }else
        {
          return BT::NodeStatus::FAILURE;
        }
      }else if (is_our_robot_capture == 3)//被双方占领
      {
        if (fortress_point_state == 3)
        {
          return BT::NodeStatus::SUCCESS;
        }else
        {
          return BT::NodeStatus::FAILURE;
        }
      }else
      {
        return BT::NodeStatus::SKIPPED;
      }
    }
  private:
    perception::Subscriber &subscriber_;
  };

  class IsOutpostAlive : public BT::ConditionNode
  {
  public:
    IsOutpostAlive(const std::string& name, const BT::NodeConfig& config,
                   perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      if (subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).message.ally_outpost_hp > 0)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsTrackLoss : public BT::ConditionNode
  {
  public:
    IsTrackLoss(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<ros::Time>("last_track_time"), //该值应当在进入track的时候被更新
          BT::InputPort<double>("lost_track_tolerant_sec") //该值应在参数文件中加载
      };
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<ros::Time> last_track_time_value = getInput<ros::Time>("last_track_time");
      ros::Time last_track_time = last_track_time_value.value();
      BT::Expected<double> lost_track_tolerant_sec_value = getInput<double>("lost_track_tolerant_sec");
      double lost_track_tolerant_sec = lost_track_tolerant_sec_value.value();

      if (ros::Time::now() - last_track_time > ros::Duration(lost_track_tolerant_sec))
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
  private:
  };

  class CheckGimbalMode : public BT::ConditionNode // 若实际的云台模式和给定的云台模式相同，则返回success，否则返回failure
  {
  public:
    CheckGimbalMode(const std::string& name, const BT::NodeConfig& config,
                    BT::Blackboard& blackboard) : ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("gimbal_mode"),  //该端口应绑定到实际的mode上面
                BT::InputPort<int>("expected_gimbal_mode")}; //该端口应绑定到期望的mode上面，一般为动态输入
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> gimbal_mode = getInput<int>("gimbal_mode");
      gimbal_mode_ = gimbal_mode.value();

      BT::Expected<int> expected_gimbal_mode = getInput<int>("expected_gimbal_mode");
      if (expected_gimbal_mode.value() == gimbal_mode_)
      {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    int gimbal_mode_;
  };

  class IsNeedInverseGimbal : public BT::ConditionNode
  {
  public:
    IsNeedInverseGimbal(const std::string& name, const BT::NodeConfig& config, BT::Blackboard& blackboard,
                        perception::Subscriber& subscriber) : BT::ConditionNode(name, config),
                                                              subscriber_(subscriber)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::vector<int>>("default_aim_rank")}; //需要绑定到实际的值
    }

    BT::NodeStatus tick() override
    {
      std::vector<int> default_aim_rank;
      BT::Expected<std::vector<int>> default_aim_rank_value = getInput<std::vector<int>>("default_aim_rank");
      default_aim_rank = default_aim_rank_value.value();
      if (!subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections.empty() &&
        subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].id != 0)
      {
        if (default_aim_rank[subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].id] == 0) //id是攻击优先级所在的数组下标，数组内部的值为攻击优先级
        {
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          return BT::NodeStatus::SUCCESS; //如果不是等于0的优先级（有效），就把云台反过来
        }
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsTargetNotInvincible : public BT::ConditionNode //TODO : 需重写无敌检测算法
  {
  public:
    IsTargetNotInvincible(const std::string& name, const BT::NodeConfig& config) : ConditionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
  };

  class IsTargetEffective : public BT::ConditionNode
  {
  public:
    IsTargetEffective(const std::string& name, const BT::NodeConfig& config,
                      perception::Subscriber& subscriber) : ConditionNode(name, config), subscriber_(subscriber)
    {
    }

    BT::NodeStatus tick() override
    {
      auto track_id = subscriber_.msgGetter<rm_msgs::TrackData>(perception::Subscriber::TopicId::TRACK_DATA).message.id;
      return track_id != 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

  private:
    perception::Subscriber& subscriber_;
  };

  class IsPoseValid : public BT::ConditionNode
  {
  public:
    IsPoseValid(const std::string &name , const BT::NodeConfig &config ,perception::TfAccessor &tf_accessor) : ConditionNode(name,config) , tf_accessor_(tf_accessor)
    {

    }

    BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::vector<double>>("map_bounds")};
    }

    BT::NodeStatus tick() override
    {
      std::vector<double> map_bounds;
      getInput("map_bounds",map_bounds);
      geometry_msgs::TransformStamped cur_in_map =  tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP,perception::TfAccessor::FrameId::BASE_LINK);
      for (int i = 0; i < map_bounds.size(); i++)
      {
        if (!std::isfinite(map_bounds[i]))
        {
          ROS_ERROR("map_bounds[%d] must be in finite", i);
          return BT::NodeStatus::FAILURE;
        }
      }
      if ((tools::isBetween(cur_in_map.transform.translation.x,map_bounds[0],map_bounds[1]) ||
        tools::isBetween(cur_in_map.transform.translation.y,map_bounds[2],map_bounds[3]) ||
        tools::isBetween(cur_in_map.transform.translation.z,map_bounds[4],map_bounds[5])) == false)
      {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
  private:
    perception::TfAccessor &tf_accessor_;
  };

  class IsRemoteControlTurnOn : public BT::ConditionNode
  {
  public:
    IsRemoteControlTurnOn(const std::string &name , const BT::NodeConfig &config , perception::Subscriber &subscriber , tools::ControllerTools &controller_tools) : ConditionNode(name , config) , subscriber_(subscriber) , controller_tools_(controller_tools)
    {

    }

    BT::NodeStatus tick() override
    {
      if (ros::Time::now() - subscriber_.msgGetter<rm_msgs::DbusData>(perception::Subscriber::TopicId::DBUS_DATA).stamp < ros::Duration(1.0))
      {
          if (controller_tools_.getControllerManager())//std::unique_ptr类型，当该指针持有对象时返回true，该对象在BasicControl中的构造函数被唯一赋值
            controller_tools_.startMainController();
          controller_tools_.calibrate();
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }

  private:
    perception::Subscriber &subscriber_;
    tools::ControllerTools &controller_tools_;
  };
}

#endif //NEW_BEHAVIOR_TREE_CONDITION_NODE_H
