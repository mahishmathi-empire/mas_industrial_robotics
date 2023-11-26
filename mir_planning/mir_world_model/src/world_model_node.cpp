#include "mir_world_model/world_model_node.hpp"

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode created");

  // get node parameter overrides
  auto overrides = this->get_node_options().parameter_overrides();

  // declare overridable parameters
  for (auto& override : overrides) {
      declare_parameter(override.get_name(), override.get_parameter_value());
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

  // world model initialization
  world_model_ = std::make_shared<WorldModel>();

  // create wokstation objects from parameters
  auto workstation_names = this->list_parameters({"workstations"}, 2).prefixes;
  for (auto& workstation_name : workstation_names) {
    std::string name = this->get_parameter(workstation_name + ".name").as_string();
    std::string type = this->get_parameter(workstation_name + ".type").as_string();
    double height = this->get_parameter(workstation_name + ".height").as_double();
    world_model_->addWorkstation(name, type, height);
  }

  // print workstation names
  std::vector<Workstation> workstations;
  world_model_->getWorkstations(workstations);
  for (auto& workstation : workstations) {
    RCLCPP_INFO(get_logger(), "--> Workstation: %s", workstation.name.c_str());
  }


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