//
// Created by root on 2026/3/5.
//

#include "behavior_tree/chassis_action_node.h"
#include "common/behavior_base.h"

namespace action_node
{
  BT::NodeStatus chassisSlowGyro()
  {
    BehaviorBase behavior_base(nh,cmd_tools,subscriber,autocontrol_info)
    setGryoBeforeCombat();
  }
}