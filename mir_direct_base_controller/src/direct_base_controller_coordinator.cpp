#include "direct_base_controller_coordinator.hpp"

DirectBaseControllerCoordinator::DirectBaseControllerCoordinator()
    : Node("direct_base_controller")
{
    baseTwist = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    targetPose = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&DirectBaseControllerCoordinator::targetPoseCallback, this, std::placeholders::_1));
    target_pose_received = false;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_parameter_or<std::string>("base_frame", baseFrame, "base_footprint");
}
void DirectBaseControllerCoordinator::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    targetPoseMsg = msg;
    target_pose_received = true;
}

void DirectBaseControllerCoordinator::start()
{
    RCLCPP_INFO(get_logger(), "Ready to start...");
    std::string state = "RUNNING";

    rclcpp::Rate loopRate(100);
    while (rclcpp::ok())
    {
        runningState();
        loopRate.sleep();
        rclcpp::spin_some(shared_from_this());
    }
}

void DirectBaseControllerCoordinator::initState()
{
    runningState();
}

void DirectBaseControllerCoordinator::runningState()
{
    auto originPose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    originPose->header.frame_id = baseFrame;
    originPose->pose.orientation.w = 1.0;

    if (target_pose_received)
    {

        bool poseError = get_component_wise_pose_error(originPose, targetPoseMsg, error, tf_buffer_);

        if (!poseError)
        {
            publish_zero_state();
        }
        ComponentWisePoseErrorMonitor monitor;
        monitor.setParameters(0.02, 0.02, 15, 15, 15, 0.04);
        if (monitor.isComponentWisePoseErrorWithinThreshold(error))
        {
            RCLCPP_INFO(get_logger(), "error within threshold");
            publish_zero_state();
        }
        else
        {

            constants.p_gain_x = 1.4;
            constants.p_gain_y = 1.4;
            constants.p_gain_z = 1.4;
            constants.p_gain_roll = 1.4;
            constants.p_gain_pitch = 1.4;
            constants.p_gain_yaw = 1.2;
            cartesian_velocity = get_cartesian_velocity(error, constants);

            limiter.max_velocity_x = 0.65;
            limiter.max_velocity_y = 0.6;
            limiter.max_velocity_z = 0.0;
            limiter.max_velocity_roll = 0.0;
            limiter.max_velocity_pitch = 0.0;
            limiter.max_velocity_yaw = 0.6;
            limited_twist = get_limited_twist(cartesian_velocity, limiter);
            synchronized_twist = twistSynchronizer.synchronizeTwist(limited_twist, error);
            baseTwist->publish(synchronized_twist);
        }
    }
}

void DirectBaseControllerCoordinator::publish_zero_state()
{

    baseTwist->publish(zero_twist);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectBaseControllerCoordinator>();
    node->start();
    return 0;
}