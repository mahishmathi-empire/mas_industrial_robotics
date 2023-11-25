#include "mir_world_model/world_model_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<WorldModelNode>());
  rclcpp::shutdown();
  return 0;
}