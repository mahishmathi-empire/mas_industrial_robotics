/**
 * @brief world_model ros node
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "mir_world_model/world_model.hpp"

class WorldModelNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WorldModelNode(const rclcpp::NodeOptions& options);

  virtual ~WorldModelNode();

  // default node options
  static rclcpp::NodeOptions default_options() {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    return options;
  }

  // Transition callback for state configuring
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  // Transition callback for state activating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state);

  // Transition callback for state deactivating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state);

  // Transition callback for state cleaningup
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&);

  // Transition callback for state shutting down
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state);
};