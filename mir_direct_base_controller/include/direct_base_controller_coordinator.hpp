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
// #include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/msg/laser_scan.hpp>


class DirectBaseControllerCoordinator : public rclcpp::Node {
public:
    DirectBaseControllerCoordinator();

    void start();
// protected:
//     void on_activate() override;
//     void on_deactivate() override;
//     void on_cleanup() override;
private:
    // void eventInCallback(const std_msgs::msg::String::SharedPtr msg);
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // void laserDistancesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    
    void laserdataCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // void convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan,
    //                                 pcl::PointCloud<pcl::PointXYZ> &pointcloud);
    void initState();
    void runningState();
    void obstical_avoidance();
    void preprocess_laser_data();
    void publish_zero_state();

    // rclcpp::Rate loopRate;
    // rclcpp::Rate idleLoopRate;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string baseFrame;
    ControllerParameters constants;
    ComponentWiseCartesianDifference error;
    LimiterParameters limiter;
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

    // collision avoidance
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserdata_sub_;
    bool useCollisionAvoidance;
    bool laser_data_received;
    sensor_msgs::msg::LaserScan laser1_ ;
    // pcl::PointCloud<pcl::PointXYZ> pointcloud;
};
