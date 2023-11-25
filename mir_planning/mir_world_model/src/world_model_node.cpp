#include "mir_world_model/world_model_node.hpp"

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode created");

  // print the parameters from the yaml file passed to the node
  RCLCPP_INFO(get_logger(), "WorldModelNode parameters:");
  auto parameter_names = rclcpp_lifecycle::LifecycleNode::list_parameters({}, 0).names;
  for (auto& parameter_name : parameter_names)
  {
    RCLCPP_INFO(get_logger(), "  %s: %s", parameter_name.c_str(), get_parameter(parameter_name).as_string().c_str());
  }
}

WorldModelNode::~WorldModelNode()
{
  RCLCPP_INFO(get_logger(), "WorldModelNode destroyed");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(WorldModelNode)