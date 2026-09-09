#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rm_common/decision/service_caller.h>
#include <rm_msgs/StatusChange.h>
#include <rm_msgs/StatusChangeRequest.h>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>

namespace auto_aim
{
// 双云台(dx_track)视觉"切目标"的总出口。行为树每次想"切去认装甲 / 认小符"，
// 都通过这里落地成两件事、且两件事必须对齐：
//   ① 调 /Processor/status_change 服务：让视觉处理器切模式 + 报要认的颜色；
//   ② 往 /autoaim_target 发目标 id 列表（dx_track 官方协议，latched 常亮）。
// 类头里的 English 注释是历史遗留：这个类独占 /vision/target_is_armor 的所有权，
// 意思是别的地方不许碰这条链，保证服务请求和话题上的目标永远对得上。
class DxTrackSwitchCaller
{
public:
  explicit DxTrackSwitchCaller(ros::NodeHandle& nh) : processor_switch_(nh)
  {
    // 目标 id 话题名和"本方配置认的敌方颜色"都从参数读，颜色错了直接报错清空
    nh.param("autoaim_target_topic", autoaim_target_topic_, std::string("/autoaim_target"));
    nh.param("configured_enemy_color", configured_enemy_color_, std::string("red"));
    if (configured_enemy_color_ != "red" && configured_enemy_color_ != "blue")
    {
      ROS_ERROR("[Vision][dx_track] configured_enemy_color must be red or blue; current value='%s'.",
                configured_enemy_color_.c_str());
      configured_enemy_color_.clear();
    }
    // latched=true：发一次就挂住，后来的订阅者一上来就能拿到，不用等下一 tick
    autoaim_target_pub_ = nh.advertise<std_msgs::Int8MultiArray>(autoaim_target_topic_, 1, true);

    // 启动先发一个安全默认铠甲 id 列表：机器人装甲 + 前哨(id 6)，彻底排除基地(id 8)。
    // 这样 AUTO/MANUAL 任何决策跑起来之前，视觉就已经有个明确目标，不会处在"谁都没指定"的悬空态。
    std_msgs::Int8MultiArray default_targets;
    default_targets.data = armorTargetIds(armor_target_);
    autoaim_target_pub_.publish(default_targets);
  }

  /**@brief 把"装甲目标模式"翻译成 dx_track 能认的一组 id。id 含义：1-5=五个兵种，6=前哨，7=哨兵，8=基地
   *@return 存放id的数组
   * **/
  static std::vector<int8_t> armorTargetIds(const uint8_t armor_target)
  {
    switch (armor_target)
    {
      case rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE:
        // 上游协议把前哨和基地捆成一个"装甲模式"，但哨兵只被授权认前哨，
        // 所以这一档只发前哨的 id，不把基地带上(免得视觉把不该打的当目标)
        return { 6 };
      case rm_msgs::StatusChangeRequest::ARMOR_WITHOUT_OUTPOST_BASE:
        // 不要前哨也不要基地，纯五个兵种 + 哨兵
        return { 1, 2, 3, 4, 5, 7 };
      case rm_msgs::StatusChangeRequest::ARMOR_ALL:
        // 全部要，但基地(8)必须显式排除——哨兵规则上打不了基地
        return { 1, 2, 3, 4, 5, 6, 7 };
      default:
        return {};
    }
  }

  /**@brief 根据所需要击打的装甲目标确定需要击打的颜色
   * **/
  static uint8_t processorRequestColor(const uint8_t target, const uint8_t enemy_color)
  {
    if (target == rm_msgs::StatusChangeRequest::SMALL_BUFF || target == rm_msgs::StatusChangeRequest::BIG_BUFF)
      return enemy_color == rm_msgs::StatusChangeRequest::RED ? rm_msgs::StatusChangeRequest::BLUE :
                                                               rm_msgs::StatusChangeRequest::RED;
    return enemy_color;
  }

  /**@brief 每tick由行为树调用的执行入口，做校验并call服务
   * **/
  void callService()
  {
    processor_switch_.resetSwitchResult();

    if (!(target_ == rm_msgs::StatusChangeRequest::ARMOR || target_ == rm_msgs::StatusChangeRequest::SMALL_BUFF) ||
      !(armor_target_ == rm_msgs::StatusChangeRequest::ARMOR_ALL ||
           armor_target_ == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE ||
           armor_target_ == rm_msgs::StatusChangeRequest::ARMOR_WITHOUT_OUTPOST_BASE))
    {
      ROS_ERROR_THROTTLE(1.0, "[Vision][dx_track] invalid target=%u or armor_target=%u.", target_, armor_target_);
      return;
    }
    // 请求的敌方颜色跟配置的颜色对不上的情况
    if (configured_enemy_color_.empty() || (color_ == rm_msgs::StatusChangeRequest::RED ? "red" : "blue") != configured_enemy_color_)
    {
      ROS_ERROR_THROTTLE(1.0, "[Vision][dx_track] requested enemy color '%s' differs from configured '%s'.",
                         (color_ == rm_msgs::StatusChangeRequest::RED ? "red" : "blue"), configured_enemy_color_.c_str());
      return;
    }

    //要切换为装甲的时候没有订阅者的情况
    if (!(target_ != rm_msgs::StatusChangeRequest::ARMOR || autoaim_target_pub_.getNumSubscribers() > 0))
    {
      ROS_WARN_THROTTLE(1.0, "[Vision][dx_track] waiting for subscriber on %s.", autoaim_target_topic_.c_str());
      return;
    }

    // 切的是装甲：把目标 id 列表发到 autoaim_target 话题
    if (target_ == rm_msgs::StatusChangeRequest::ARMOR)
    {
      std_msgs::Int8MultiArray target_ids;
      target_ids.data = explicit_armor_target_ids_.empty() ? armorTargetIds(armor_target_) : explicit_armor_target_ids_;
      autoaim_target_pub_.publish(target_ids);
    }
    // 然后调 /Processor/status_change 服务；颜色在函数里对 buff 做过反转
    processor_switch_.setRequest(processorRequestColor(target_, color_), target_, armor_target_);
    processor_switch_.callService();
  }

  /**@brief 判定是否获取响应
   * **/
  [[nodiscard]]bool isCalling()
  {
    return processor_switch_.isCalling();
  }

  /**@brief 判定服务是否响应
   * **/
  bool getIsSwitch()
  {
    return processor_switch_.getIsSwitch();
  }

  /**@brief 把上次的成功标志抹掉，重新开始一轮
   * **/
  void resetSwitchResult()
  {
    processor_switch_.resetSwitchResult();
  }

  /**@brief 获取服务重试的失败上限
   * **/
  int getFailLimit()
  {
    return processor_switch_.getFailLimit();
  }

  /**@brief 由己方颜色判定敌方颜色
   * **/
  void setEnemyColor(const std::string& own_color)
  {
    // 己方蓝 → 敌方红，反过来一样；存的就是敌方颜色
    color_ = own_color == "blue" ? rm_msgs::StatusChangeRequest::RED : rm_msgs::StatusChangeRequest::BLUE;
    callService();
  }

  /**@brief 设置目标类型
   *@param target 目标类型，rm_msgs::StatusChangeRequest::ARMOR || rm_msgs::StatusChangeRequest::SMALL_BUFF
   * **/
  void setTargetType(const uint8_t target)
  {
    target_ = target;
  }

  /**@brief 设置装甲子模式
  *@param armor_target 装甲子模式 ，rm_msgs::StatusChangeRequest::ARMOR_ALL ||
            rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE ||
           rm_msgs::StatusChangeRequest::ARMOR_WITHOUT_OUTPOST_BASE
   * **/
  void setArmorTargetType(const uint8_t armor_target)
  {
    armor_target_ = armor_target;
  }

  bool setExplicitArmorTargetIds(const std::vector<int8_t>& target_ids)
  {
    if (explicit_armor_target_ids_ == target_ids)
      return false;
    explicit_armor_target_ids_ = target_ids;
    return true;
  }

  const std::vector<int8_t>& getExplicitArmorTargetIds() const
  {
    return explicit_armor_target_ids_;
  }

  /**@brief 获取当前需要识别的敌方颜色
   * **/
  [[nodiscard]]int getColor() const
  {
    return color_;
  }

  /**@brief 获取当前目标类型
   *@details 0 = ARMOR , 1 = SMALL_BUFF
   * **/
  [[nodiscard]]int getTarget() const
  {
    return target_;
  }

  /**@brief 获取当前装甲子模式
   *@details 0 = ARMOR_ALL , 1 = ARMOR_OUTPOST_BASE , 2 = ARMOR_WITHOUT_OUTPOST_BASE
   ***/
  [[nodiscard]]int getArmorTarget() const
  {
    return armor_target_;
  }

  /**@brief 获取曝光等级
   * **/
  [[nodiscard]]uint8_t getExposureLevel() const
  {
    return rm_msgs::StatusChangeRequest::EXPOSURE_LEVEL_0;
  }

private:
  class ProcessorSwitchCaller : public rm_common::ServiceCallerBase<rm_msgs::StatusChange>
  {
  public:
    explicit ProcessorSwitchCaller(ros::NodeHandle& nh)
      : rm_common::ServiceCallerBase<rm_msgs::StatusChange>(nh, "/Processor/status_change")
    {
    }

    /**@brief 填充服务请求的各个字段
     *@param color 要击打的目标颜色
     *@param target 目标大类，例如装甲或者能量机关
     *@param armor_target 装甲板子模式
     * **/
    void setRequest(const uint8_t color, const uint8_t target, const uint8_t armor_target)
    {
      service_.request.color = color;
      service_.request.target = target;
      service_.request.armor_target = armor_target;
      service_.request.exposure = rm_msgs::StatusChangeRequest::EXPOSURE_LEVEL_0;
    }

    // 切换成功 = 请求已结束 && 服务回包明确说成功了
    /**@brief 判定服务是否响应
     * **/
    bool getIsSwitch()
    {
      return !isCalling() && service_.response.switch_is_success;
    }

    /**@brief 把上次的成功标志抹掉，重新开始一轮
     * **/
    void resetSwitchResult()
    {
      service_.response.switch_is_success = false;
    }
  };

  ros::Publisher autoaim_target_pub_;             // 往 /autoaim_target 发目标 id 列表的发布者(latched)
  std::vector<int8_t> explicit_armor_target_ids_; // 显式指定的装甲 id 清单（非空时优先于默认列表）
  std::string autoaim_target_topic_;              // 目标 id 话题名
  std::string configured_enemy_color_;            // 配置里声明的敌方颜色
  ProcessorSwitchCaller processor_switch_;        // 调用 /Processor/status_change 的服务封装
  uint8_t color_{ rm_msgs::StatusChangeRequest::RED };   // 敌方颜色（红/蓝枚举）
  uint8_t target_{ rm_msgs::StatusChangeRequest::ARMOR };  // 目标大类（默认认装甲）
  uint8_t armor_target_{ rm_msgs::StatusChangeRequest::ARMOR_ALL };  // 装甲子模式（默认全甲）
};

}  // namespace rm_behavior_tree
