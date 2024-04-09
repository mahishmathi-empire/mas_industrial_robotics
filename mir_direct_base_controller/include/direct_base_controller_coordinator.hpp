#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"    
#include "component_wise_pose_error_calculator.hpp"
#include "component_wise_pose_error_monitor.hpp"
#include "twist_controller.hpp"
#include "twist_limiter.hpp"
#include "twistsynchronizer.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class DirectBaseControllerCoordinator : public rclcpp_lifecycle::LifecycleNode {
public:
    DirectBaseControllerCoordinator();

    void start();
    /// Transition callback for state configuring
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    /// Transition callback for state activating
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    /// Transition callback for state deactivating
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

      /// Transition callback for state cleaningup
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    /// Transition callback for state shutting down
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state);

private:

    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void laserdataCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void runningState();
    void obstical_avoidance();
    void preprocess_laser_data();
    void publish_zero_state();
    void readParamsFromConf();
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string baseFrame;
    ControllerParameters constants;
    ComponentWiseCartesianDifference error;
    LimiterParameters limiter;
    bool target_pose_received;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> baseTwist;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetPose;
    std::string event;
    geometry_msgs::msg::Twist zero_twist;
    geometry_msgs::msg::PoseStamped::SharedPtr targetPoseMsg;
    ComponentWisePoseErrorMonitor componentWisePoseErrorMonitor;
    geometry_msgs::msg::Twist cartesian_velocity;
    geometry_msgs::msg::Twist limited_twist;
    geometry_msgs::msg::Twist synchronized_twist;
    TwistSynchronizer twistSynchronizer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserdata_sub_;    
    bool useCollisionAvoidance;
    bool laser_data_received;
    sensor_msgs::msg::LaserScan laser1_ ;
    
};
