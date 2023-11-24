// bethavior tree example

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <std_msgs/msg/string.hpp>

#include "mir_interfaces/action/object_detection.hpp"

class PerceiveLocationAction : public BT::StatefulActionNode
{
  public:
    PerceiveLocationAction(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<rclcpp::Node>& node)
      : BT::StatefulActionNode(name, config), _node(node)
    {
      RCLCPP_INFO(node->get_logger(), "PerceiveLocationAction: %s", name.c_str());

      // create action client
      _action_client = rclcpp_action::create_client<mir_interfaces::action::ObjectDetection>(node, "ppt_detection");
    }

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("message") };
    }

    // invoked once at the beginning
    BT::NodeStatus onStart() override
    {
      RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: onStart");

      // read input port
      BT::Expected<std::string> msg = getInput<std::string>("message");

      if(!msg)
      {
        throw BT::RuntimeError("missing required input [message]: ", msg.error());
      }

      RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: message: %s", msg.value().c_str());

      // wait for action server
      if(!_action_client->wait_for_action_server())
      {
        RCLCPP_ERROR(_node->get_logger(), "PerceiveLocationAction: action server not available after waiting");
        return BT::NodeStatus::FAILURE;
      }

      // send goal
      auto goal_msg = mir_interfaces::action::ObjectDetection::Goal();
      goal_msg.obj_category = "ppt";

      RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: sending goal");

      goal_handle_future = _action_client->async_send_goal(goal_msg);

      return BT::NodeStatus::RUNNING;
    }

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override
    {
      // spin
      rclcpp::spin_until_future_complete(_node, goal_handle_future, std::chrono::milliseconds(100));
      // check if goal is succ
      if(goal_handle_future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
      {
        RCLCPP_INFO(_node->get_logger(), "PerceiveLocationAction: goal not succ");
        return BT::NodeStatus::RUNNING;
      }

      return BT::NodeStatus::SUCCESS;

    }

    // callback to execute if the action was aborted by another node
    void onHalted() override
    {
      std::cout << "PerceiveLocationAction: onHalted" << std::endl;
    }

  private:

    std::shared_ptr<rclcpp::Node> _node;

    // action client
    rclcpp_action::Client<mir_interfaces::action::ObjectDetection>::SharedPtr _action_client;

    // future to store the goal handle
    std::shared_future<rclcpp_action::ClientGoalHandle<mir_interfaces::action::ObjectDetection>::SharedPtr> goal_handle_future;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create a node
  auto node = std::make_shared<rclcpp::Node>("PerceiveLocationAction");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<PerceiveLocationAction>("PerceiveLocationAction", node);

  // get xml file
  std::string pkg_path = ament_index_cpp::get_package_share_directory("mir_perceive_location");

  std::string bt_path = pkg_path + "/bt/";
  
  // load all xml files in this directory
  for (const auto& entry : std::filesystem::directory_iterator(bt_path))
  {
    if (entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  BT::Tree tree = factory.createTree("MainTree");

  // tick the root of the tree
  // tree.tickWhileRunning();

  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";

  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(100));

    // spin
    rclcpp::spin_some(node);

    std::cout << "--- ticking\n";
    status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }

  rclcpp::shutdown();

  return 0;
}