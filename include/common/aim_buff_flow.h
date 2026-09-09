#pragma once

#include <cmath>
#include <cstdint>

#include <ros/ros.h>

namespace aim_buff
{
  // 哨兵打能量机关（符文）时的"云台+扳机"配合流程。
  // 管的是：裁判确认符被激活之后，视觉切到符文窗口、云台扫到符、盯住符暖机一段时间、然后开火。
  // 视觉切换的真正执行在 VisionTargetSwitcher，这里只看它的结果，自己不碰订阅、不发指令。
  class AimBuffFlow
  {
  public:
    enum class State
    {
      Idle, //没在打符流程里
      SwitchingVision, //等视觉从装甲模式切到符文模式
      AimFan, //视觉就绪了，云台扫符等目标出现
      TrackWarmup, //盯住符了，但还没暖够时间（这期间不扣扳机）
      Confirming, //暖够了，可以开火
      Failed //视觉切换失败，终态
    };

    // 每 tick 喂进来的跟踪帧信息，由调用方从 /track（自瞄）里装配
    struct TrackInput
    {
      uint8_t id{0};
      bool tracking{false};
      bool finite_position{false};
      ros::Time stamp;
      ros::Time now;
      double timeout_sec{0.3}; // 跟踪帧多久没更新就算丢了
      double loss_track_tolerant_sec{0.15}; // 丢跟踪后多少秒内先"假装还在"，防状态乱抖
    };

    /**@brief 开始打符流程，应在开始转换视觉目标时调用
     *@details 只在 Idle 时才进 SwitchingVision，已经在流程里就什么都不做
     * **/
    void enter()
    {
      if (state_ == State::Idle)
        state_ = State::SwitchingVision;
    }

    /**@brief 用于打符结束时复位
     * **/
    void leave()
    {
      state_ = State::Idle;
      vision_ready_stamp_ = ros::Time(0.0);
      last_valid_track_time_ = ros::Time(0.0);
      track_enter_time_ = ros::Time(0.0);
    }

    /**@brief 标记正在切换视觉模式的状态
     *@details 用于记录状态和时间戳，该函数需要在正在切换视觉模式时被调用
     * **/
    void markVisionSwitching()
    {
      if (state_ != State::Idle && state_ != State::Failed)
      {
        state_ = State::SwitchingVision;
        last_valid_track_time_ = ros::Time(0.0);
      }
    }

    /**@brief 标记成功切换视觉模式的状态
     *@details 用于记录状态和时间戳，该函数需要在成功切换视觉模式时被调用
     * **/
    void markVisionReady(const ros::Time& now)
    {
      if (state_ == State::Idle || state_ == State::Failed || isVisionReady())
        return;
      vision_ready_stamp_ = now;
      last_valid_track_time_ = ros::Time(0.0);
      state_ = State::AimFan;
    }

    /**@brief 标记切换视觉模式失败的状态
     *@details 用于记录状态和时间戳，该函数需要在切换视觉模式失败时被调用
     * **/
    void markVisionFailed()
    {
      state_ = State::Failed;
      last_valid_track_time_ = ros::Time(0.0);
      track_enter_time_ = ros::Time(0.0);
    }

    /**@brief 用于更新打符状态机状态，该函数须在每次tick时被调用
     *@param input 跟踪帧信息，需由调用方装配结构体
     *@param  track_before_shoot_sec 冷却时间，即在允许开火前的等待时间
     * **/
    void updateTrack(const TrackInput& input, const double track_before_shoot_sec = 0.0)
    {
      // 视觉没就绪（还在切/已失败）时，跟踪帧一概忽略
      if (!isVisionReady())
        return;

      if (isFreshTrack(input))
      {
        if (track_enter_time_.isZero() || input.now < track_enter_time_)
          track_enter_time_ = input.now;
        last_valid_track_time_ = input.now;
        // 大符要求先盯够 track_before_shoot_sec（默认 3 秒）才放行开火；小符是 0 秒，拿到就 Confirm。
        const double required_sec =
          std::isfinite(track_before_shoot_sec) && track_before_shoot_sec > 0.0 ? track_before_shoot_sec : 0.0;
        state_ = input.now - track_enter_time_ >= ros::Duration(required_sec)
                   ? State::Confirming
                   : State::TrackWarmup; //冷却指定的时间
        return;
      }
      // 这一帧不算新鲜（丢跟踪/超时）。只要丢的时间还在 loss_grace_sec 内，
      // 就先维持 Warmup/Confirming 别降级，免得偶尔一帧抽风就让云台在扫符/盯符之间来回抖。
      const bool tracking_state = state_ == State::TrackWarmup || state_ == State::Confirming;
      const bool grace_valid = tracking_state && !last_valid_track_time_.isZero() &&
        input.now >= last_valid_track_time_ && std::isfinite(input.loss_track_tolerant_sec) &&
        input.loss_track_tolerant_sec >= 0.0 &&
        input.now - last_valid_track_time_ <= ros::Duration(input.loss_track_tolerant_sec);
      if (!grace_valid)
        state_ = State::AimFan;
    }

    /**@brief 在打符流程中的时候供调用方判断这局打没打完
     * **/
    [[nodiscard]] bool active() const
    {
      return state_ != State::Idle;
    }

    /**@brief 视觉窗口已就绪（扫/盯/打都算），可以开始收跟踪帧
      * **/
    [[nodiscard]] bool isVisionReady() const
    {
      return state_ == State::AimFan || state_ == State::TrackWarmup || state_ == State::Confirming;
    }

    /**@brief 供外部调用，判定是否需要进入TrackEnemy模式，还是保留扫描模式
     * **/
    [[nodiscard]] bool shouldTrack() const
    {
      return state_ == State::TrackWarmup || state_ == State::Confirming;
    }

    /**@brief 判定是否能切换到push模式
     * **/
    [[nodiscard]] bool shouldShoot() const
    {
      return state_ == State::Confirming;
    }

    // 距离暖机开始过了多久，调试/日志用
    /**@brief 获取距离开始进入track的时间，供调试/日志用
     *@param now 当前时间
     * **/
    [[nodiscard]] double trackElapsedSec(const ros::Time& now) const
    {
      if (!shouldTrack() || track_enter_time_.isZero() || now < track_enter_time_)
        return 0.0;
      return (now - track_enter_time_).toSec();
    }

    /**@brief 将state结构体的枚举值转换为字符串，供外部调试使用
     * **/
    std::string stateName() const
    {
      switch (state_)
      {
      case State::Idle:
        return "Idle";
      case State::SwitchingVision:
        return "SwitchingVision";
      case State::AimFan:
        return "AimFan";
      case State::TrackWarmup:
        return "TrackWarmup";
      case State::Confirming:
        return "Confirming";
      case State::Failed:
        return "Failed";
      }
      return "Unknown";
    }

  private:
    static constexpr uint8_t kBuffTrackId = 12; // 能量机关在自瞄数据里固定占的 ID
    // 这一帧算不算"有效跟踪"。四条硬门槛，缺一条都是假帧：
    //   ① ID 是符　　② 在跟踪中且坐标是有限数
    //   ③ 帧是视觉切换完成之后产的（切之前装甲模式的老帧时间戳更早，不算）
    //   ④ 帧没老过 timeout_sec
    /**@brief 判定视觉帧是否刷新
     *@param input 输入的TrackInput结构体
     * **/
    [[nodiscard]] bool isFreshTrack(const TrackInput& input) const
    {
      return input.id == kBuffTrackId && input.tracking && input.finite_position && !input.stamp.isZero() &&
        !vision_ready_stamp_.isZero() && input.stamp >= vision_ready_stamp_ && input.now >= input.stamp &&
        std::isfinite(input.timeout_sec) && input.timeout_sec > 0.0 &&
        input.now - input.stamp <= ros::Duration(input.timeout_sec);
    }

    State state_{State::Idle}; // 当前阶段
    ros::Time vision_ready_stamp_{0.0}; // 视觉就绪时刻，跟踪帧晚于它才算有效
    ros::Time last_valid_track_time_{0.0}; // 最近一帧有效跟踪的时刻，丢跟踪的宽限就靠它算
    ros::Time track_enter_time_{0.0}; // 暖机起点，一局符只定一次
  };

  struct AimBuffAttemptSample
  {
    bool activation_status{ false }; //是否激活
    ros::Time activation_stamp; //激活时间戳
    uint8_t small_power_rune_state{ 0 }; //小能量机关状态
    uint8_t large_power_rune_state{ 0 }; //大能量机关状态
    ros::Time event_stamp;
    ros::Time now;
  };

  class AimBuffAttemptTracker
  {
  public:
    // 裁判判出来的符是哪种
    enum class RuneType
    {
      Unknown, // 还没判出来
      Small, // 小能量机关
      Big // 大能量机关
    };

    // 打符尝试进行到哪一步了
    enum class State
    {
      Idle, // 空闲，没在打符流程里
      Transit, // 已经决定要打，正在导航去符区
      AwaitingCommandPublish, // 到位了，等"请裁判激活"命令发出去
      AwaitingRuneType, // 命令已发布，等裁判回头判大小符
      Activating // 裁判确认激活中，正式开打
    };

    // 每次喂入观察后对外汇报的"这一步怎么收场"
    enum class Result
    {
      None, // 没啥大事，继续推进/继续等
      SmallSelected, // 判出是小符，进入 Activating
      BigSelected, // 判出是大符，进入 Activating
      Succeeded, // 打成了
      Failed, // 失败了（可触发全局锁）
      Aborted // 安全放弃（不锁全局）
    };

    /**@brief 发起一次打符尝试
     *@return 允许打符时返回true
     * **/
    bool beginAttempt(const AimBuffAttemptSample& sample, const bool debug_attempt)
    {
      if (state_ != State::Idle || (!debug_attempt && locked_out_))
        return false;

      // 正式开账：进交通状态，记下发起时刻，大小符还没判出来
      state_ = State::Transit;
      debug_attempt_ = debug_attempt;
      intent_time_ = sample.now;
      rune_type_ = RuneType::Unknown;
      return true;
    }

    // 底盘导航到位后由云台决策调用。先看看裁判现在怎么说：
    // 符已经打完了（白跑一趟但算成功）或被抢了（放弃），就直接收场；
    // 否则进"等发确认命令"阶段，并把发起意图时间重置为到位时刻。
    /**@brief 底盘到达打符位置的时候对能量机关状态做判定，该函数需在底盘到达指定位置时被调用
     * **/
    Result markArrived(const AimBuffAttemptSample& sample)
    {
      if (state_ != State::Transit)
        return Result::None;
      const Result gate = preActivationStateResult(sample);
      if (gate != Result::None)
        return gate;
      state_ = State::AwaitingCommandPublish; //未激活时的情况
      intent_time_ = sample.now;
      return Result::None;
    }

    // "请裁判激活"命令确认已发布（GetSentryCmd 节流发出去的那下才来调这个）。
    // 记下发布戳好判断裁判回的是什么，然后进"等裁判回头判大小符"阶段。
    /**@brief 哨兵向裁判系统发确认使能量机关进入激活状态时调用
     ***/
    Result markConfirmPublished(const AimBuffAttemptSample& sample, const ros::Time& publish_stamp)
    {
      if (state_ != State::AwaitingCommandPublish || publish_stamp.isZero())
        return Result::None;
      const Result gate = preActivationStateResult(sample);
      if (gate != Result::None)
        return gate;
      confirm_publish_stamp_ = publish_stamp;
      saw_fresh_feedback_ = false;
      state_ = State::AwaitingRuneType;
      return Result::None;
    }

    /**@brief 能量机关状态推进器，该函数应在激活能量机关时持续被调用
     *@param sample 裁判系统反馈数据结构体
     *@param event_fresh event_data是否刷新
     *@param confirm_feedback_timeout_sec  向裁判系统发送使能量机关正在激活命令的超时时间
     *@param active_watchdog_sec 击打能量机关超时时间
     * **/
    Result observe(const AimBuffAttemptSample& sample, const bool event_fresh,
                   const double confirm_feedback_timeout_sec, const double active_watchdog_sec = 22.0)
    {
      if (state_ == State::Idle) //未准备打符的情况
        return Result::None;

      if (state_ == State::Transit || state_ == State::AwaitingCommandPublish)
      {
        const Result gate = preActivationStateResult(sample); // 还在赶路/等发命令时，裁判反馈抢先有结论（已打完/被抢/数据坏了）的情况
        if (gate != Result::None)
          return gate;

        if (state_ == State::AwaitingCommandPublish && //向裁判系统发送使能量机关正在激活命令超时的情况
          sample.now >= intent_time_ &&
          sample.now - intent_time_ >= ros::Duration(confirm_feedback_timeout_sec))
          return Result::Failed;
        return Result::None;
      }

      if (state_ == State::AwaitingRuneType)  //正常向裁判系统发送了请求
      {
        // 裁判数据本身异常（状态值越界）
        if (sample.small_power_rune_state > 2 || sample.large_power_rune_state > 2)
          return Result::Aborted;
        // 关键判定：这条反馈是不是"确认命令发布之后"的新反馈？时间戳比发布晚才算数
        const bool feedback_after_publish = event_fresh && !sample.event_stamp.isZero() &&
          sample.event_stamp > confirm_publish_stamp_;
        if (feedback_after_publish) //正常获取到新反馈
        {
          saw_fresh_feedback_ = true;
          if (sample.small_power_rune_state == 1 || sample.large_power_rune_state == 1) //能量机关已激活
            return Result::Succeeded;
          const bool small_activating = sample.small_power_rune_state == 2;
          const bool big_activating = sample.large_power_rune_state == 2;
          // 大小符同时激活 = 裁判状态撞车了，谁也不选，安全放弃
          if (small_activating && big_activating)
            return Result::Aborted;
          // 只有一个在激活 → 它就是要打的符，记下类型进 Activating，跟云台/扳机说话
          if (small_activating || big_activating)
          {
            rune_type_ = small_activating ? RuneType::Small : RuneType::Big;
            activation_started_time_ = sample.now;
            state_ = State::Activating;
            return small_activating ? Result::SmallSelected : Result::BigSelected;
          }
        }
        // 命令发布后等反馈也有时限：见过新鲜反馈但迟迟没判出来 → 失败；一次都没见过 → 放弃
        if (sample.now >= confirm_publish_stamp_ &&
          sample.now - confirm_publish_stamp_ >= ros::Duration(confirm_feedback_timeout_sec))
          return saw_fresh_feedback_ ? Result::Failed : Result::Aborted;
        return Result::None;
      }

      // 已进入 Activating：只看选定符自己的状态变化
      if (sample.small_power_rune_state > 2 || sample.large_power_rune_state > 2)
        return Result::Aborted;
      const uint8_t selected_state =
        rune_type_ == RuneType::Small ? sample.small_power_rune_state : sample.large_power_rune_state;

      if (selected_state == 1) //已激活
        return Result::Succeeded;
      if (selected_state == 0) //未激活
        return Result::Failed;
      // 激活起就一直在打，超了 active_watchdog_sec（默认 22 秒）还没结束 → 看门狗放弃，防止死磕
      if (sample.now >= activation_started_time_ &&
        sample.now - activation_started_time_ >= ros::Duration(active_watchdog_sec))
        return Result::Aborted;
      return Result::None;
    }

    // 失败的收尾手段。enable_failure_lock 决定要不要锁全局：
    // 正常比赛失败 → 锁这一局的自动打符；调试尝试（bt_sim）失败 → 不锁，方便反复练。
    // 返回 true 表示"这一下刚刚上了全局锁"（只有第一次失败会返回 true）。
    bool finishFailure(const bool enable_failure_lock)
    {
      const bool newly_locked = enable_failure_lock && !debug_attempt_ && !locked_out_;
      // 是正常尝试就置全局锁；锁再次置只是幂等，不会真重复上锁
      if (enable_failure_lock && !debug_attempt_)
        locked_out_ = true;
      finishAttempt();
      return newly_locked;
    }

    /**@brief 在激活能量机关成功或者失败的时候清除所有中间状态
     * **/
    void finishAttempt()
    {
      state_ = State::Idle;
      debug_attempt_ = false;
      rune_type_ = RuneType::Unknown;
      intent_time_ = ros::Time(0.0);
      confirm_publish_stamp_ = ros::Time(0.0);
      activation_started_time_ = ros::Time(0.0);
      saw_fresh_feedback_ = false;
    }

    // 这一场是不是调试尝试（bt_sim 发起的，失败不锁全局）
    [[nodiscard]]bool debugAttempt() const
    {
      return debug_attempt_;
    }

    // 这一局判出来的是大符还是小符
    [[nodiscard]]RuneType getRuneType() const
    {
      return rune_type_;
    }

    // 是否已经整局锁死（不再自动发起打符）
    [[nodiscard]]bool lockedOut() const
    {
      return locked_out_;
    }

    /**@brief 获取状态枚举值
     * **/
    [[nodiscard]]State getState() const
    {
      return state_;
    }

    // 状态名，打日志用
    static const char* stateName(const State state)
    {
      switch (state)
      {
      case State::Idle:
        return "Idle";
      case State::Transit:
        return "Transit";
      case State::AwaitingCommandPublish:
        return "AwaitingCommandPublish";
      case State::AwaitingRuneType:
        return "AwaitingRuneType";
      case State::Activating:
        return "Activating";
      }
      return "Unknown";
    }

  private:
    // "激活之前"的通用提前判定：不管处在哪一段（赶路/等发布/发布瞬间），
    // 裁判如果已经给了结论就当场收掉——打完算成功、被抢算放弃、数据坏算放弃。
    /**@brief 根据裁判系统输入的能量机关激活状态做判定
     *@return 1为Succeeded,2为Aborted,0为None,除此之外为Aborted
     * **/
    static Result preActivationStateResult(const AimBuffAttemptSample& sample)
    {
      if (sample.small_power_rune_state > 2 || sample.large_power_rune_state > 2) //能量机关激活状态不正常
        return Result::Aborted;
      if (sample.small_power_rune_state == 1 || sample.large_power_rune_state == 1) //已激活
        return Result::Succeeded;
      if (sample.small_power_rune_state == 2 || sample.large_power_rune_state == 2) //正在激活
        return Result::Aborted;
      return Result::None; //未激活
    }

    // 新一局初始化：全局锁解开（上一局的失败不带到新局），顺手把账本清干净
    void resetForBattle()
    {
      locked_out_ = false;
      finishAttempt();
    }

    State state_{State::Idle}; // 当前阶段
    bool locked_out_{false}; // 本局自动打符是否被锁死（正常失败触发，调试尝试不触发）
    bool debug_attempt_{false}; // 本场是否是调试尝试
    bool saw_fresh_feedback_{false}; // 命令发布后是否见过裁判的新鲜反馈（决定超时时判 Failed 还是 Aborted）
    RuneType rune_type_{RuneType::Unknown}; // 判出来的大小符类型
    ros::Time intent_time_{0.0}; // 发起尝试/到位的时刻（"等发布/等反馈"阶段的超时起点）
    ros::Time confirm_publish_stamp_{0.0}; // "请激活"命令实际发布出去的戳（判断裁判反馈是不是发布后的）
    ros::Time activation_started_time_{0.0}; // 裁判确认激活的时刻（22 秒打符看门狗的起点）
  };
} // namespace aim_buff
