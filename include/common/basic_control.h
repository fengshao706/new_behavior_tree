//
// Created by root on 2026/3/5.
//

#ifndef NEW_BEHAVIOR_TREE_BASIC_CONTROL_H
#define NEW_BEHAVIOR_TREE_BASIC_CONTROL_H

#include <perception_layer.h>

#include "rm_behavior_tree/common/subscriber.h"
#include "common/tools.h"
#include "rm_behavior_tree/common/auto_control_info.h"
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>
#include "rm_behavior_tree/complex_behavior/manual.h"
#include "rm_behavior_tree/complex_behavior/chassis_behavior.h"
#include "rm_behavior_tree/complex_behavior/gimbal_behavior.h"
#include "rm_behavior_tree/complex_behavior/shooter_behavior.h"

class BasicControl
{
public:
  explicit BasicControl(ros::NodeHandle& nh);

  void update();
  void controllerUpdate();
  void remoteControlTurnOff();
  void resetBehaviorState();
  void sendPathData();
  void setGyro(double move_gyro_vel);
  void calibrate();
  void visionCalibrate() const;

  rm_common::ControllerManager controller_manager_;
  rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{}, *barrel_calibration_{};
  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  rm_common::SwitchDetectionCaller* switch_detection_black_srv_;

  tools::CmdTools cmd_tools_;
  perception::Subscriber subscriber_;
  AutoControlInfo auto_control_info_;

  Manual* manual_state_;
  ChassisBehavior* chassis_behavior_;
  GimbalBehavior* gimbal_behavior_;
  ShooterBehavior* shooter_behavior_;
  bool init_gyro_state_{ false }, has_calibrated_barrel_{ false };
};

#endif //NEW_BEHAVIOR_TREE_BASIC_CONTROL_H