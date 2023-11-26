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
  auto workstation_names =
    this->list_parameters({ "workstations" }, 2).prefixes;
  for (auto& workstation_name : workstation_names) {
    std::string name =
      this->get_parameter(workstation_name + ".name").as_string();
    std::string type =
      this->get_parameter(workstation_name + ".type").as_string();
    double height =
      this->get_parameter(workstation_name + ".height").as_double();
    world_model_->addWorkstation(name, type, height);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode activated");

  // create subscibers
  object_list_subscriber_ =
    this->create_subscription<mir_interfaces::msg::ObjectList>(
      "~/object_list",
      rclcpp::SensorDataQoS(),
      std::bind(
        &WorldModelNode::objectListCallback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");

  LifecycleNode::on_deactivate(state);

  // shutdown subscribers
  object_list_subscriber_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");

  LifecycleNode::on_cleanup(state);

  // destroy subscribers
  object_list_subscriber_.reset();

  // destroy world model
  world_model_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode shut down");

  LifecycleNode::on_shutdown(state);

  // delete subscribers
  delete object_list_subscriber_.get();

  // destroy world model
  delete world_model_.get();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

void
WorldModelNode::objectListCallback(
  const mir_interfaces::msg::ObjectList::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "WorldModelNode received object list");

  // update world model
  for (auto& object : msg->objects) {
    world_model_->addAtworkObjectToWorkstation(
      msg->workstation, object.name, object.database_id, object.pose);
  }
}

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(WorldModelNode)