#include "direct_base_controller_coordinator.hpp"

DirectBaseControllerCoordinator::DirectBaseControllerCoordinator()
    : Node("direct_base_controller"),
      loopRate(rclcpp::Rate(get_parameter_or<double>("loop_rate", 100.0))),
      idleLoopRate(rclcpp::Rate(get_parameter_or<double>("idle_loop_rate", 1.0))),
      baseFrame(get_parameter_or<std::string>("base_frame", "base_link")),
      useCollisionAvoidance(get_parameter_or<bool>("use_collision_avoidance", true)) {
    eventOut = create_publisher<std_msgs::msg::String>("event_out", 1);
    baseTwist = create_publisher<geometry_msgs::msg::Twist>("twist", 1);

    eventIn = create_subscription<std_msgs::msg::String>("event_in", 1, std::bind(&DirectBaseControllerCoordinator::eventInCallback, this, std::placeholders::_1));
    targetPose = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&DirectBaseControllerCoordinator::targetPoseCallback, this, std::placeholders::_1));
    collisionFilter = create_subscription<mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback>("collision_filter", 1, std::bind(&DirectBaseControllerCoordinator::collisionFilterCallback, this, std::placeholders::_1));
    laserDistances = create_subscription<std_msgs::msg::Float32MultiArray>("/mcr_navigation/laser_distances/distances", 1, std::bind(&DirectBaseControllerCoordinator::laserDistancesCallback, this, std::placeholders::_1));

    auto dynamicReconfigureCb = [this](const mcr_direct_base_controller::DirectBaseControllerConfig& config) {
        dynamicReconfigureCallback(config);
    };
    dynamic_reconfigure::Server<mcr_direct_base_controller::DirectBaseControllerConfig>::CallbackType f;
    f = dynamicReconfigureCb;
    server.setCallback(f);
}

void DirectBaseControllerCoordinator::eventInCallback(const std_msgs::msg::String::SharedPtr msg) {
    event = msg->data;
}

void DirectBaseControllerCoordinator::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    targetPoseMsg = *msg;
}

void DirectBaseControllerCoordinator::collisionFilterCallback(const mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback::SharedPtr msg) {
    collisionFilterFeedback = *msg;
}

void DirectBaseControllerCoordinator::laserDistancesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    laserDistancesData = msg->data;
}

void DirectBaseControllerCoordinator::start() {
    RCLCPP_INFO(get_logger(), "Ready to start...");
    std::string state = "INIT";

    while (rclcpp::ok()) {
        if (state == "INIT") {
            initState();
        } else if (state == "RUNNING") {
            runningState();
        }

        RCLCPP_DEBUG(get_logger(), "State: %s", state.c_str());
        if (state == "INIT") {
            idleLoopRate.sleep();
        } else if (state == "RUNNING") {
            loopRate.sleep();
        }

        rclcpp::spin_some(shared_from_this());
    }
}

void DirectBaseControllerCoordinator::initState() {
    if (event == "e_start") {
        event = "";
        collisionFilterFeedback = mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback();
        runningState();
    } else if (event == "e_stop") {
        event = "";
        collisionFilterFeedback = mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback();
        publishZeroVelocities();
        eventOut->publish("e_stopped");
        state = "INIT";
    } else {
        state = "INIT";
    }
}

void DirectBaseControllerCoordinator::runningState() {
    if (event == "e_stop") {
        event = "";
        publishZeroVelocities();
        eventOut->publish("e_stopped");
        state = "INIT";
    }

    geometry_msgs::msg::PoseStamped originPose;
    originPose.header.frame_id = baseFrame;
    originPose.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped poseError = componentWisePoseErrorCalculator.getComponentWisePoseError(originPose, targetPoseMsg);
    if (poseError.pose.orientation.w != 0.0) {
        eventOut->publish("e_success");
        publishZeroVelocities();
       
    }
}