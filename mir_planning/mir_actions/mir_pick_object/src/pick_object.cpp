#include "mir_pick_object/pick_object.hpp"

PickObjectAction::PickObjectAction(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   const std::shared_ptr<rclcpp::Node>& node)
  : BT::StatefulActionNode(name, config)
  , _node(node)
{
  RCLCPP_INFO(node->get_logger(), "PickObjectAction: %s", name.c_str());

  // create action client
  _action_client =
    rclcpp_action::create_client<mir_interfaces::action::ObjectSelector>(
      node, "/world_model_node/object_selector");
}

BT::PortsList
PickObjectAction::providedPorts()
{
  return { BT::InputPort<std::string>("workstation_name"),
           BT::InputPort<std::string>("object_name") };
}

// invoked once at the beginning
BT::NodeStatus
PickObjectAction::onStart()
{
  RCLCPP_INFO(_node->get_logger(), "PickObjectAction: onStart");

  // read input port
  BT::Expected<std::string> workstation_name =
    getInput<std::string>("workstation_name");

  BT::Expected<std::string> object_name = getInput<std::string>("object_name");

  if (!workstation_name) {
    throw BT::RuntimeError("missing required input [message]: ",
                           workstation_name.error());
  }

  if (!object_name) {
    throw BT::RuntimeError("missing required input [message]: ",
                           object_name.error());
  }

  RCLCPP_INFO(_node->get_logger(),
              "PickObjectAction: workstation_name: %s, object_name: %s",
              workstation_name.value().c_str(),
              object_name.value().c_str());

  // wait for action server
  if (!_action_client->wait_for_action_server()) {
    RCLCPP_ERROR(_node->get_logger(),
                 "PickObjectAction: action server not available after waiting");
    return BT::NodeStatus::FAILURE;
  }

  // send goal
  auto goal_msg = mir_interfaces::action::ObjectSelector::Goal();
  goal_msg.obj_name = object_name.value();
  goal_msg.workstation_name = workstation_name.value();

  RCLCPP_INFO(_node->get_logger(), "PickObjectAction: sending goal");

  goal_handle_future = _action_client->async_send_goal(goal_msg);

  return BT::NodeStatus::RUNNING;
}

// If onStart() returned RUNNING, we will keep calling
// this method until it return something different from RUNNING
BT::NodeStatus
PickObjectAction::onRunning()
{
  // spin
  auto ret = rclcpp::spin_until_future_complete(
    _node, goal_handle_future, std::chrono::milliseconds(100));

  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(_node->get_logger(), "PickObjectAction: goal not succ");
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_INFO(_node->get_logger(), "PickObjectAction: goal succ");

    // TODO: get the result fucking somehow
    
    }

  return BT::NodeStatus::SUCCESS;
}

// callback to execute if the action was aborted by another node
void
PickObjectAction::onHalted()
{
  std::cout << "PickObjectAction: onHalted" << std::endl;
}
