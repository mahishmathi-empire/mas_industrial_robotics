#include "mir_perceive_location/perceive_location.hpp"

PerceiveLocationAction::PerceiveLocationAction(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::shared_ptr<rclcpp::Node>& node)
  : BT::StatefulActionNode(name, config)
  , _node(node)
{
  RCLCPP_INFO(node->get_logger(), "PerceiveLocationAction: %s", name.c_str());

  // create action client
  _action_client =
    rclcpp_action::create_client<mir_interfaces::action::ObjectDetection>(
      node, "/object_detection");
}

BT::PortsList
PerceiveLocationAction::providedPorts()
{
  return { BT::InputPort<std::string>("workstation_name") };
}

// invoked once at the beginning
BT::NodeStatus
PerceiveLocationAction::onStart()
{
  RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: onStart");

  // read input port
  BT::Expected<std::string> workstation_name =
    getInput<std::string>("workstation_name");

  if (!workstation_name) {
    throw BT::RuntimeError("missing required input [workstation_name]: ",
                           workstation_name.error());
  }

  RCLCPP_INFO(_node->get_logger(),
              "PerceiveLocationAction: workstation_name: %s",
              workstation_name.value().c_str());

  // wait for action server
  if (!_action_client->wait_for_action_server()) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "PerceiveLocationAction: action server not available after waiting");
    return BT::NodeStatus::FAILURE;
  }

  // send goal
  auto goal_msg = mir_interfaces::action::ObjectDetection::Goal();
  goal_msg.obj_category = "atwork";
  goal_msg.obj_name = "";
  goal_msg.workstation = workstation_name.value();

  RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: sending goal");

  goal_handle_future = _action_client->async_send_goal(goal_msg);

  return BT::NodeStatus::RUNNING;
}

// If onStart() returned RUNNING, we will keep calling
// this method until it return something different from RUNNING
BT::NodeStatus
PerceiveLocationAction::onRunning()
{
  // spin
  rclcpp::spin_until_future_complete(
    _node, goal_handle_future, std::chrono::milliseconds(100));

  auto status = goal_handle_future.wait_for(std::chrono::seconds(0));

  if (status == std::future_status::ready) {
    auto result = goal_handle_future.get()->get_status();
      RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: goal succ");
  } else {
    RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: goal not ready");
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

// callback to execute if the action was aborted by another node
void
PerceiveLocationAction::onHalted()
{
  std::cout << "PerceiveLocationAction: onHalted" << std::endl;
}
