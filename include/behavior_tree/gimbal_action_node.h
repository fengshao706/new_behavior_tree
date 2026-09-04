//
// Created by fengshao on 2026/3/26.
//

#ifndef NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H
#define NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H

#include <common/tools.h>

#include "common/types.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "geometry_msgs/TransformStamped.h"
#include <rm_msgs/PriorityArray.h>
#include "rm_common/ori_tool.h"
#include "common/invincible_detection.h"

namespace gimbal
{
  class SetGimbalMode : public BT::SyncActionNode
  {
  public:
    SetGimbalMode(const std::string &name ,const BT::NodeConfig &config) : SyncActionNode(name , config)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("input_gimbal_mode_id"),
                BT::OutputPort<int>("gimbal_mode")};
    }

    BT::NodeStatus tick() override
    {
      BT::Expected<int> input_gimbal_mode_id = getInput<int>("input_gimbal_mode_id");
      setOutput<int>("gimbal_mode", input_gimbal_mode_id.value());
      return BT::NodeStatus::SUCCESS;
    }

  private:

  };

  class YawSlowRound : public BT::StatefulActionNode
  {
  public:
    YawSlowRound(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools , tools::CmdTools &cmd_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools) , cmd_tools_(cmd_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("yaw_vel"), //yaw轴旋转速度
        BT::InputPort<int>("scan_range_circles"),
        BT::InputPort<double>("pitch_inside_vel"),
        BT::InputPort<double>("pitch_outside_vel"),
        BT::InputPort<double>("pitch_min"),
        BT::InputPort<double>("pitch_max"),
        BT::InputPort<double>("breach_thresholds")
      }; //yaw每转多少圈就回复
    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      double yaw_vel;
      int scan_range_circles;
      double pitch_inside_vel;
      double pitch_outside_vel;
      double pitch_min;
      double pitch_max;
      double breach_thresholds;

      getInput<double>("yaw_vel",yaw_vel);
      getInput<int>("scan_range_circles",scan_range_circles);
      getInput<double>("pitch_inside_vel",pitch_inside_vel);
      getInput<double>("pitch_outside_vel",pitch_outside_vel);
      getInput<double>("pitch_min",pitch_min);
      getInput<double>("pitch_max",pitch_max);
      getInput<double>("breach_thresholds",breach_thresholds);
      gimbal_tools_.lidarTwist(yaw_vel,scan_range_circles);
      gimbal_tools_.updatePitchStrafeDirect(pitch_min, pitch_max,
                  pitch_outside_vel, pitch_inside_vel,
                  breach_thresholds);
      gimbal_tools_.setStackGimbalRate();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
      cmd_tools_.getSenders()->base_gimbal_command_sender_->setZero();
    }
  private:
    tools::GimbalTools &gimbal_tools_;
    tools::CmdTools &cmd_tools_;
  };

  class InverseGimbal : public BT::StatefulActionNode //需用timeout节点维持运行一小段时间
  {
  public:
    InverseGimbal(const std::string &name ,const BT::NodeConfig &config ,tools::GimbalTools &gimbal_tools , tools::CmdTools &cmd_tools , perception::Subscriber &subscriber , perception::TfAccessor &tf_accessor) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools) , cmd_tools_(cmd_tools) , tf_accessor_(tf_accessor), subscriber_(subscriber)
    {

    }

    BT::NodeStatus onStart() override
    {
      geometry_msgs::TransformStamped back_camera2map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP , perception::TfAccessor::FrameId::BACK_CAMERA_OPTICAL_FRAME);
      geometry_msgs::PointStamped target2back;
      target2back.header.frame_id = "back_camera_optical_frame";
      target2back.point.x = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.x;
      target2back.point.y = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.y;
      target2back.point.z = subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::BACK_CAMERA_DETECTION_DATA).message.detections[0].pose.position.z;
      tf2::doTransform(target2back, back_of_map_, back_camera2map); //获取目标在map上面的位置
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(back_of_map_);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->base_gimbal_command_sender_->setZero();
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }
  private:
    tools::GimbalTools &gimbal_tools_;
    tools::CmdTools &cmd_tools_;
    perception::TfAccessor &tf_accessor_;
    perception::Subscriber &subscriber_;
    geometry_msgs::PointStamped back_of_map_;
  };

  class TrackEnemy : public BT::StatefulActionNode
  {
  public:
    TrackEnemy(const std::string &name ,const BT::NodeConfig &config , tools::CmdTools &cmd_tools ,tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , cmd_tools_(cmd_tools) , gimbal_tools_(gimbal_tools)
    {

    }

    BT::NodeStatus onStart() override
    {
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setStackGimbalTrack();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      cmd_tools_.getSenders()->base_gimbal_command_sender_->setZero();
      cmd_tools_.getSenders()->gimbal_command_sender_->setZero();
    }
  private:
    tools::CmdTools &cmd_tools_;
    tools::GimbalTools &gimbal_tools_;
  };

  class PreAimingOutpost : public BT::StatefulActionNode
  {
  public:
    PreAimingOutpost(const std::string &name , const BT::NodeConfig &config , tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color"),
                  BT::InputPort<double>("aim_per_point_sec"),
                BT::InputPort<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses") ,
                  BT::InputPort<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses")};
    }

    BT::NodeStatus onStart() override
    {
      aim_per_point_sec_ = getInput<double>("aim_per_point_sec").value();
      robot_color_ = getInput<std::string>("robot_color").value();
      if (robot_color_ == "blue")
      {
        enemy_outpost_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("blue_outpost_poses").value();
      }else
      {
        enemy_outpost_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("red_outpost_poses").value();
      }
      record_aim_time_ = ros::Time::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(enemy_outpost_positions_[current_point_ % enemy_outpost_positions_.size()]);
      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_)) //大于设定的超时时间时换下一个点
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(enemy_outpost_positions_.size()))
        {
          ROS_INFO("Fail to attack outpost.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      current_point_ = 0;
    }
  private:
    std::string robot_color_;
    std::vector<geometry_msgs::PointStamped> enemy_outpost_positions_;
    int current_point_ {0};
    double aim_per_point_sec_;
    ros::Time record_aim_time_ = ros::Time::now();
    tools::GimbalTools &gimbal_tools_;
  };

  class PreAimingBase : public BT::StatefulActionNode
  {
  public:
    PreAimingBase(const std::string &name , const BT::NodeConfig &config , tools::GimbalTools &gimbal_tools) : StatefulActionNode(name , config) , gimbal_tools_(gimbal_tools)
    {

    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("robot_color"),
                  BT::InputPort<double>("aim_per_point_sec"),
                BT::InputPort<std::vector<geometry_msgs::PointStamped>>("blue_base_poses") ,
                  BT::InputPort<std::vector<geometry_msgs::PointStamped>>("red_base_poses")};
    }

    BT::NodeStatus onStart() override
    {
      aim_per_point_sec_ = getInput<double>("aim_per_point_sec").value();
      robot_color_ = getInput<std::string>("robot_color").value();
      if (robot_color_ == "blue")
      {
        enemy_base_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("blue_base_poses").value();
      }else
      {
        enemy_base_positions_ = getInput<std::vector<geometry_msgs::PointStamped>>("red_base_poses").value();
      }
      record_aim_time_ = ros::Time::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      gimbal_tools_.setGimbalDirectPoint(enemy_base_positions_[current_point_ % enemy_base_positions_.size()]);
      if (ros::Time::now() - record_aim_time_ > ros::Duration(aim_per_point_sec_)) //大于设定的超时时间时换下一个点
      {
        current_point_++;
        record_aim_time_ = ros::Time::now();
        if (current_point_ > static_cast<int>(enemy_base_positions_.size()))
        {
          ROS_INFO("Fail to attack outpost.");
          current_point_ = 0;
        }
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      current_point_ = 0;
    }
  private:
    std::string robot_color_;
    std::vector<geometry_msgs::PointStamped> enemy_base_positions_;
    int current_point_ {0};
    double aim_per_point_sec_;
    ros::Time record_aim_time_ = ros::Time::now();
    tools::GimbalTools &gimbal_tools_;
  };

  class UpdateAimPriority : public BT::SyncActionNode
  {
  public:
    UpdateAimPriority(const std::string &name , const BT::NodeConfig &config , perception::TfAccessor &tf_accessor , perception::Subscriber &subscriber, perception::Publisher &publisher , tools::NavigationTools &navigation_tools , invincible_detection::EnemyInvincibilityManager &enemy_hp_state_tracker) : SyncActionNode(name , config) , tf_accessor_(tf_accessor) , subscriber_(subscriber), publisher_(publisher) , navigation_tools_(navigation_tools) , enemy_hp_state_tracker_(enemy_hp_state_tracker)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("chassis_mode") ,
                  BT::InputPort<std::string>("robot_color"),
                BT::OutputPort<std::vector<uint8_t>>("aim_priority")};
    }

    BT::NodeStatus tick() override
    {
      std::vector<uint8_t> aim_priority;
      int input_chassis_mode = getInput<int>("chassis_mode").value();
      std::string robot_color = getInput<std::string>("robot_color").value();
      std::string enemy_color;
      if (robot_color == "blue") //给enemy_color赋值
      {
        enemy_color = "red";
      }else if (robot_color == "red")
      {
        enemy_color = "blue";
      }
      types::ChassisMode chassis_mode = static_cast<types::ChassisMode>(input_chassis_mode);
      switch (chassis_mode)
      {
      case types::ChassisMode::GotoHitEnemyOutpostArea :
        {
          std::vector<uint8_t> src = {1,1,1,1,5,1,1,1};
          aim_priority = src;
          break;
        }
      case types::ChassisMode::GotoEnemyBase :
        {
          std::vector<uint8_t> src = {1,1,1,1,1,1,1,5};
          aim_priority = src;
          break;
        }
      case types::ChassisMode::GotoTrapezoidalHighland :
        {
          std::vector<uint8_t> src = {5,1,3,3,3,1,2,1};
          aim_priority = src;
          break;
        }
      default:
        {
          std::vector<uint8_t> src = {5, 3, 2, 2, 2, 5, 1, 0};
          aim_priority = src;
          break;
        }
      }

      //----------获取目标在map坐标系下的坐标------------
      geometry_msgs::TransformStamped camera_optical_frame2map;
      camera_optical_frame2map = tf_accessor_.getTfTransform(perception::TfAccessor::FrameId::MAP,perception::TfAccessor::FrameId::CAMERA_OPTICAL_FRAME);
      for (auto& detection : subscriber_.msgGetter<rm_msgs::TargetDetectionArray>(perception::Subscriber::TopicId::FRONT_CAMERA_DETECTION_DATA).message.detections)

      {
        geometry_msgs::TransformStamped target_at_map, target_at_camera;
        target_at_camera.transform.translation.x = detection.pose.position.x;
        target_at_camera.transform.translation.y = detection.pose.position.y;
        target_at_camera.transform.translation.z = detection.pose.position.z;
        target_at_camera.header.frame_id = "camera_optical_frame";
        target_at_camera.header.stamp = ros::Time::now();
        tf2::doTransform(target_at_camera, target_at_map, camera_optical_frame2map);

        geometry_msgs::Point enemy_position;
        enemy_position.x = target_at_map.transform.translation.x;
        enemy_position.y = target_at_map.transform.translation.y;
        enemy_position.z = target_at_map.transform.translation.z;
        std::string enemy_in_area = navigation_tools_.determinePolygonInWhich(enemy_position);

        if (detection.id > 0 && detection.id <=8)
        {
          //---------目标为建筑时判断建筑是否死亡以及数据是否新鲜-----------
          const bool is_hp_fresh = ros::Time::now() - subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).stamp < ros::Duration(1.5);
          rm_msgs::GameRobotHp game_robot_hp = subscriber_.msgGetter<rm_msgs::GameRobotHp>(perception::Subscriber::TopicId::GAME_ROBOT_HP).message;
          if (detection.id == static_cast<int>(types::RobotType::OUTPOST) || detection.id == static_cast<int>(types::RobotType::BASE))
          {
            if (is_hp_fresh == false) //血量数据新鲜度不足直接返回false
            {
              aim_priority[detection.id-1] = 0;
            }
            if (detection.id == static_cast<int>(types::RobotType::OUTPOST) && game_robot_hp.enemy_outpost_hp <= 0)
            {
              aim_priority[detection.id-1] = 0;
            }
            if (detection.id == static_cast<int>(types::RobotType::BASE) && game_robot_hp.enemy_base_hp <=0)
            {
              aim_priority[detection.id-1] = 0;
            }
          }

          //----------判断目标所处的区域并动态调整优先级------------
          if (enemy_in_area == robot_color+"_fortress_area")//目标位于自家堡垒区,将优先级开到最高
          {
            aim_priority[detection.id-1] = 5; //数组下标需要将实际id减去1
          }

          if (enemy_in_area == enemy_color+"_fortress_area")//目标位于敌方堡垒区，将优先级开到最低
          {
            aim_priority[detection.id-1] = 1; //数组下标需要将实际id减去1
          }

          if (detection.id == 2 && enemy_in_area == enemy_color+"_engineer_invincible_area")//目标是工程的情况
          {
            aim_priority[detection.id-1] = 0; //工程在无敌区不打
          }

          //-----------利用无敌状态检测，覆盖之前的优先级，无敌或死亡强制不打-----------
          if (detection.id != static_cast<int>(types::RobotType::OUTPOST) && detection.id != static_cast<int>(types::RobotType::BASE))
          {
            invincible_detection::EnemyLifeSnapshot target_life_snapshot = enemy_hp_state_tracker_.snapshot(detection.id,ros::Time::now());
            if (target_life_snapshot.state != invincible_detection::EnemyInvincibleState::ALIVE)
            {
              aim_priority[detection.id-1] = 0;
            }
          }
        }
      }

      setOutput<std::vector<uint8_t>>("aim_priority",aim_priority);

      rm_msgs::PriorityArray priority_array;
      priority_array.rank_arr = aim_priority;
      publisher_.getPublishers()->aim_priority_pub_.publish(priority_array);
      return BT::NodeStatus::SUCCESS;
    }
  private:
    perception::TfAccessor &tf_accessor_;
    perception::Subscriber &subscriber_;
    perception::Publisher &publisher_;
    tools::NavigationTools &navigation_tools_;
    invincible_detection::EnemyInvincibilityManager &enemy_hp_state_tracker_;
  };
}


#endif //NEW_BEHAVIOR_TREE_GIMBAL_ACTION_NODE_H