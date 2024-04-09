/*
 * Copyright 2024 Bonn-Rhein-Sieg University
 *
 * Author: Gaurav Shetty, , 
 *
 */

#include "direct_base_controller_coordinator.hpp"

/*
 * Params used in the direct controller.
 */
double threshold_linear_x;
double threshold_linear_y;
double threshold_linear_z;
double threshold_angular_x;
double threshold_angular_y;
double threshold_angular_z;
double p_gain_x;
double p_gain_y;
double p_gain_z;
double p_gain_roll;
double p_gain_yaw;
double p_gain_pitch;
double max_velocity_x;
double max_velocity_y;
double max_velocity_z;
double max_velocity_roll;
double max_velocity_pitch;
double max_velocity_yaw;
double loop_rate;
bool use_collision_avoidance;
double collision_distance;

/**
 * This component moves the mobile base in Cartesian space until a pose is reached.
 * The input pose must be provided in some static world frame.
 */
DirectBaseControllerCoordinator::DirectBaseControllerCoordinator()
    : LifecycleNode("direct_base_controller")
{
    RCLCPP_INFO(get_logger(), "Node created.");
}

/**
 * Obtains the target pose.
 */
void DirectBaseControllerCoordinator::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    targetPoseMsg = msg;
    target_pose_received = true;
}

/**
 * Obtains the received data from laser.
 */
void DirectBaseControllerCoordinator::laserdataCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser1_ = *msg;
    laser_data_received = true;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", "LaserScan received");
}
/**
 * Reads the data from the file config.yaml.
 */
void DirectBaseControllerCoordinator::readParamsFromConf()
{
    this->declare_parameter("threshold_linear_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("threshold_linear_y", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("threshold_linear_z", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("threshold_angular_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("threshold_angular_y", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("threshold_angular_z", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_y", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_z", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_yaw", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p_gain_pitch", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_x", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_y", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_z", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_roll", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_pitch", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_yaw", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("use_collision_avoidance", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("collision_distance", rclcpp::PARAMETER_DOUBLE);
    threshold_linear_x = this->get_parameter("threshold_linear_x").as_double();
    threshold_linear_y = this->get_parameter("threshold_linear_y").as_double();
    threshold_linear_z = this->get_parameter("threshold_linear_z").as_double();
    threshold_angular_x = this->get_parameter("threshold_angular_x").as_double();
    threshold_angular_y = this->get_parameter("threshold_angular_y").as_double();
    threshold_angular_z = this->get_parameter("threshold_angular_z").as_double();
    p_gain_x = this->get_parameter("p_gain_x").as_double();
    p_gain_y = this->get_parameter("p_gain_y").as_double();
    p_gain_z = this->get_parameter("p_gain_z").as_double();
    p_gain_roll = this->get_parameter("p_gain_roll").as_double();
    p_gain_yaw = this->get_parameter("p_gain_yaw").as_double();
    p_gain_pitch = this->get_parameter("p_gain_pitch").as_double();
    max_velocity_x = this->get_parameter("max_velocity_x").as_double();
    max_velocity_y = this->get_parameter("max_velocity_y").as_double();
    max_velocity_z = this->get_parameter("max_velocity_z").as_double();
    max_velocity_roll = this->get_parameter("max_velocity_roll").as_double();
    max_velocity_pitch = this->get_parameter("max_velocity_pitch").as_double();
    max_velocity_yaw = this->get_parameter("max_velocity_yaw").as_double();
    RCLCPP_INFO(get_logger(), "Helooo...%f", threshold_linear_x);
    collision_distance = this->get_parameter("collision_distance").as_double();
    loop_rate = this->get_parameter("loop_rate").as_double();
    use_collision_avoidance = this->get_parameter("use_collision_avoidance").as_bool();
}

/**
 * Starts the robot running.
 */
 
void DirectBaseControllerCoordinator::start()
{
    readParamsFromConf();
    RCLCPP_INFO(get_logger(), "Ready to start...");
    std::string state = "RUNNING";

    rclcpp::Rate loopRate(100);
    while (rclcpp::ok())
    {
        runningState();
        loopRate.sleep();
        rclcpp::spin_some(this -> get_node_base_interface());
    }
}

/**
 * Running state of the robot.
 */
void DirectBaseControllerCoordinator::runningState()
{
    auto originPose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    originPose->header.frame_id = baseFrame;
    originPose->pose.orientation.w = 1.0;

    if (target_pose_received)
    {
        std::cout << "target pose received" << std::endl;
        std::cout << laser_data_received << std::endl;
        bool poseError = get_component_wise_pose_error(originPose, targetPoseMsg, error, tf_buffer_);

        if (!poseError)
        {
            publish_zero_state();
        }
        ComponentWisePoseErrorMonitor monitor;
        monitor.setParameters(threshold_linear_x, threshold_linear_y, threshold_linear_z, threshold_angular_x, threshold_angular_y, threshold_angular_z);
        if (monitor.isComponentWisePoseErrorWithinThreshold(error))
        {
            RCLCPP_INFO(get_logger(), "error within threshold");
            publish_zero_state();
        }
        else
        {
            if (laser_data_received)
            {
                std::cout << "laser data received" << std::endl;
                preprocess_laser_data();
                std::cout << "laser data processed" << std::endl;
                obstical_avoidance();
                std::cout << "obstacle avoidance ran" << std::endl;
                std::cout << useCollisionAvoidance << std::endl;

                if (useCollisionAvoidance)
                {
                    std::cout << "Collision avoidance running!" << std::endl;
                    publish_zero_state();
                }
                else
                {
                    constants.p_gain_x = p_gain_x;
                    constants.p_gain_y = p_gain_y;
                    constants.p_gain_z = p_gain_z;
                    constants.p_gain_roll = p_gain_roll;
                    constants.p_gain_pitch = p_gain_pitch;
                    constants.p_gain_yaw = p_gain_yaw;
                    cartesian_velocity = get_cartesian_velocity(error, constants);
                    limiter.max_velocity_x = max_velocity_x;
                    limiter.max_velocity_y = max_velocity_y;
                    limiter.max_velocity_z = max_velocity_z;
                    limiter.max_velocity_roll = max_velocity_roll;
                    limiter.max_velocity_pitch = max_velocity_pitch;
                    limiter.max_velocity_yaw = max_velocity_yaw;
                    limited_twist = get_limited_twist(cartesian_velocity, limiter);
                    synchronized_twist = twistSynchronizer.synchronizeTwist(limited_twist, error);
                    baseTwist->publish(synchronized_twist);
                }
            }
        }
    }
}

/**
 * Preprocessing the data obtained from.
 */
void DirectBaseControllerCoordinator::preprocess_laser_data()
{

    for (size_t i = 0; i < laser1_.ranges.size(); ++i)
    {
        // Check for invalid range values (e.g., NaN or Inf)
        if (std::isnan(laser1_.ranges[i]) || std::isinf(laser1_.ranges[i]))
        {
            // Replace invalid range values with a default maximum range
            laser1_.ranges[i] = laser1_.range_max;
        }
    }

    // Apply median filter to smooth the laser scan data
    const int window_size = 3; // Window size for median filter
    std::vector<float> ranges_sorted(window_size);

    for (size_t i = 1; i < laser1_.ranges.size() - 1; ++i)
    {
        // Extract window of range values
        std::copy(laser1_.ranges.begin() + i - 1, laser1_.ranges.begin() + i + 2, ranges_sorted.begin());

        // Sort window values
        std::sort(ranges_sorted.begin(), ranges_sorted.end());

        // Replace current range value with median of window
        laser1_.ranges[i] = ranges_sorted[window_size / 2];
    }
}

/**
 * Checking for close obstacles and stopping if found any.
 */
void DirectBaseControllerCoordinator::obstical_avoidance()
{
    // Iterate through laser ranges
    for (float range : laser1_.ranges)
    {
        // Check if the range is less than 1.0 meter
        if (range < collision_distance)
        {
            // Execute obstacle avoidance action
            std::cout << "Obstacle detected!" << std::endl;
            useCollisionAvoidance = true;
            // publish_zero_state();
            break; // Stop further processing as obstacle detected
        }
        useCollisionAvoidance = false;
    }
}

/**
 * Stopping the robbot from moving.
 */
void DirectBaseControllerCoordinator::publish_zero_state()
{
    baseTwist->publish(zero_twist);
}

/**
 * NodeLifeCycle: Configuring the node.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DirectBaseControllerCoordinator::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Node configured.");
    baseTwist = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    targetPose = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&DirectBaseControllerCoordinator::targetPoseCallback, this, std::placeholders::_1));
    rclcpp::QoS laser_scan_qos(10);       // Keep last 10 messages
    laser_scan_qos.best_effort();         // Set reliability to best effort
    laser_scan_qos.durability_volatile(); // Set durability to volatile

    laserdata_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "front_scan", laser_scan_qos,
        std::bind(&DirectBaseControllerCoordinator::laserdataCallback, this, std::placeholders::_1));
    target_pose_received = false;
    laser_data_received = false;
    useCollisionAvoidance = use_collision_avoidance;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    std::cout << laser_data_received << std::endl;
    get_parameter_or<std::string>("base_frame", baseFrame, "base_footprint");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * NodeLifeCycle: Activating the node.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DirectBaseControllerCoordinator::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Node activated.");
    baseTwist->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * NodeLifeCycle: Deactivating the node.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DirectBaseControllerCoordinator::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Node deactivated.");
    publish_zero_state();
    baseTwist->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * NodeLifeCycle: CleaningUp the node.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DirectBaseControllerCoordinator::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Node cleaned up.");
    baseTwist.reset();  // Release the publisher
    targetPose.reset(); // Release the subscriber
    tf_buffer_.reset(); // Release the tf2 buffer
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * NodeLifeCycle: Shutting Down the node.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DirectBaseControllerCoordinator::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Node shutting down.");
    baseTwist.reset(); // Release the publisher
    targetPose.reset(); // Release the subscriber
    tf_buffer_.reset(); // Release the tf2 buffer
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * Main function of the node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectBaseControllerCoordinator>();
    node->start();
    rclcpp::shutdown();
    return 0;
}