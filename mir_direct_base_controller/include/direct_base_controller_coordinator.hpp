#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <dynamic_reconfigure/server.h>   check this
// #include <rcl_interfaces/srv/reconfigure.hpp"
// #include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "component_wise_pose_error_calculator.hpp"
#include "component_wise_pose_error_monitor.hpp"
#include "twist_controller.hpp"
#include "twist_limiter.hpp"
#include "twistsynchronizer.hpp"


// struct CollisionParameters{
//         bool use_collision_avoidance;
//         double front_laser_threshold;
//         double right_laser_threshold;
//         double rear_laser_threshold;
//         double left_laser_threshold;
//         double loop_rate;
// };



class DirectBaseControllerCoordinator : public rclcpp::Node {
public:
    DirectBaseControllerCoordinator();

    void start();

private:
    void eventInCallback(const std_msgs::msg::String::SharedPtr msg);
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void laserDistancesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void initState();
    void runningState();

    void publish_zero_state();

    // rclcpp::Rate loopRate;
    // rclcpp::Rate idleLoopRate;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string baseFrame;
    ControllerParameters constants;
    ComponentWiseCartesianDifference error;
    LimiterParameters limiter;
    bool useCollisionAvoidance;
    bool target_pose_received;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr baseTwist;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetPose;
    // rclcpp::Subscription<mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback>::SharedPtr collisionFilter;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr laserDistances;

    std::string event;
    geometry_msgs::msg::Twist zero_twist;
    geometry_msgs::msg::PoseStamped::SharedPtr targetPoseMsg;
    std::vector<float> laserDistancesData;

    // ComponentWisePoseErrorCalculator componentWisePoseErrorCalculator;
    ComponentWisePoseErrorMonitor componentWisePoseErrorMonitor;

    // twist_controller twistController;
    // twist_limiter twistLimiter;
    geometry_msgs::msg::Twist cartesian_velocity;
    geometry_msgs::msg::Twist limited_twist;
    geometry_msgs::msg::Twist synchronized_twist;
    TwistSynchronizer twistSynchronizer;
};
