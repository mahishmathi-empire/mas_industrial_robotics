#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dynamic_reconfigure/server.h>
#include <mcr_direct_base_controller/DirectBaseControllerConfig.h>
#include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.hpp>

#include "component_wise_pose_error_calculator.hpp"
#include "component_wise_pose_error_monitor.hpp"
#include "twist_controller.hpp"
#include "twist_limiter.hpp"
#include "twistsynchronizer.hpp"

class DirectBaseControllerCoordinator : public rclcpp::Node {
public:
    DirectBaseControllerCoordinator();

    void start();

private:
    void eventInCallback(const std_msgs::msg::String::SharedPtr msg);
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void collisionFilterCallback(const mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback::SharedPtr msg);
    void laserDistancesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void initState();
    void runningState();

    void publishZeroVelocities();
    void dynamicReconfigureCallback(const mcr_direct_base_controller::DirectBaseControllerConfig& config);

    rclcpp::Rate loopRate;
    rclcpp::Rate idleLoopRate;

    std::string baseFrame;

    bool useCollisionAvoidance;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr eventOut;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr baseTwist;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eventIn;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetPose;
    rclcpp::Subscription<mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback>::SharedPtr collisionFilter;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr laserDistances;

    std::string event;
    geometry_msgs::msg::PoseStamped targetPoseMsg;
    mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback collisionFilterFeedback;
    std::vector<float> laserDistancesData;

    ComponentWisePoseErrorCalculator componentWisePoseErrorCalculator;
    ComponentWisePoseErrorMonitor componentWisePoseErrorMonitor;
    mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback feedback;

    TwistController twistController;
    TwistLimiter twistLimiter;
    TwistSynchronizer twistSynchronizer;
};
