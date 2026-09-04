//
// Created by root on 2026/6/17.
//

#ifndef NEW_BEHAVIOR_TREE_CONTROL_NODE_H
#define NEW_BEHAVIOR_TREE_CONTROL_NODE_H

#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

class ReactiveIfThenElse : public BT::ControlNode
{
public:
  ReactiveIfThenElse(const std::string& name , const BT::NodeConfig& config) : ControlNode(name, config)
  {

  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if(childrenCount() != 3)
    {
      throw BT::LogicError(
          "ReactiveIfThenElse requires exactly 3 children");
    }

    auto* condition = children_nodes_[0];
    auto* then_branch = children_nodes_[1];
    auto* else_branch = children_nodes_[2];

    auto cond_status = condition->executeTick();

    if(cond_status == BT::NodeStatus::RUNNING)
    {
      throw BT::LogicError(
        "Condition child of ReactiveIfThenElse "
        "must not return RUNNING");
    }

    if(cond_status == BT::NodeStatus::SUCCESS)
    {
      if(else_branch->status() == BT::NodeStatus::RUNNING)
      {
        haltChild(2);
      }

      return then_branch->executeTick();
    }

    if(cond_status == BT::NodeStatus::FAILURE)
    {
      if(then_branch->status() == BT::NodeStatus::RUNNING)
      {
        haltChild(1);
      }

      return else_branch->executeTick();
    }

    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    ControlNode::halt();
  }
};

class RunForSeconds : public BT::DecoratorNode
{
public:
  RunForSeconds(const std::string &name , const BT::NodeConfig &config) : DecoratorNode(name , config)
  {

  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("seconds")};
  }

  BT::NodeStatus tick() override
  {
    double seconds;
    if (!getInput("seconds", seconds))
    {
      throw BT::LogicError("Missing required input [seconds] in RunForSeconds");
    }

    // 子节点刚启动 → 重置计时器
    if (child_node_->status() != BT::NodeStatus::RUNNING)
    {
      start_time_ = Clock::now();
    }

    setStatus(BT::NodeStatus::RUNNING);
    auto child_status = child_node_->executeTick();

    // 子节点已结束 → 重置状态，透传结果
    if (isStatusCompleted(child_status))
    {
      resetChild();
      return child_status;
    }

    // 子节点还在运行 → 检查是否超时
    if (child_status == BT::NodeStatus::RUNNING)
    {
      auto elapsed = std::chrono::duration<double>(
          Clock::now() - start_time_).count();
      if (elapsed >= seconds)
      {
        haltChild();
        resetChild();
        return BT::NodeStatus::SUCCESS;   // 超时强制成功
      }
    }

    return child_status;  // 未超时，继续运行
  }


  void halt() override
  {
    DecoratorNode::halt();
  }

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_time_;
};

#endif //NEW_BEHAVIOR_TREE_CONTROL_NODE_H