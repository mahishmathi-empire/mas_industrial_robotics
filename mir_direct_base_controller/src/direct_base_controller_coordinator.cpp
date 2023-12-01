#include "direct_base_controller_coordinator.hpp"

DirectBaseControllerCoordinator::DirectBaseControllerCoordinator()
    : Node("direct_base_controller"),
      loopRate(rclcpp::Rate(get_parameter_or<double>("loop_rate", 100.0))),
      idleLoopRate(rclcpp::Rate(get_parameter_or<double>("idle_loop_rate", 1.0))),
      baseFrame(get_parameter_or<std::string>("base_frame", "base_link"))
    //   useCollisionAvoidance(get_parameter_or<bool>("use_collision_avoidance", true)) 
{
    baseTwist = create_publisher<geometry_msgs::msg::Twist>("twist", 1);
    targetPose = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&DirectBaseControllerCoordinator::targetPoseCallback, this, std::placeholders::_1));
    target_pose_received = false;
    // collisionFilter = create_subscription<mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback>("collision_filter", 1, std::bind(&DirectBaseControllerCoordinator::collisionFilterCallback, this, std::placeholders::_1));
    // laserDistances = create_subscription<std_msgs::msg::Float32MultiArray>("/mcr_navigation/laser_distances/distances", 1, std::bind(&DirectBaseControllerCoordinator::laserDistancesCallback, this, std::placeholders::_1));
}
    void DirectBaseControllerCoordinator::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        targetPoseMsg = msg;
        target_pose_received = true;
    }

    // void DirectBaseControllerCoordinator::collisionFilterCallback(const mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback::SharedPtr msg) {
    //     collisionFilterFeedback = *msg;
    // }

    // void DirectBaseControllerCoordinator::laserDistancesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    //     laserDistancesData = msg->data;
    // }

    void DirectBaseControllerCoordinator::start() 
    {
        RCLCPP_INFO(get_logger(), "Ready to start...");
        std::string state = "";

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

    void DirectBaseControllerCoordinator::initState() 
    {
        runningState();
    }

    void DirectBaseControllerCoordinator::runningState() 
    {
        auto originPose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        originPose->header.frame_id = "baseFrame";
        originPose->pose.orientation.w = 1.0;
        
        if(target_pose_received){

        
            bool poseError = get_component_wise_pose_error(originPose, targetPoseMsg, error);
            
            if(poseError)
            {
                publish_zero_state();
                // return "INIT";
            }
            ComponentWisePoseErrorMonitor monitor;
            monitor.setParameters(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
            if(monitor.isComponentWisePoseErrorWithinThreshold(error))
            {
                publish_zero_state();
                // return "INIT"; 
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
                limiter.max_velocity_y =0.6;
                limiter.max_velocity_z =0;
                limiter.max_velocity_roll =0;
                limiter.max_velocity_pitch =0;
                limiter.max_velocity_yaw = 0.6;
                limited_twist = get_limited_twist(cartesian_velocity, limiter);
                synchronized_twist = twistSynchronizer.synchronizeTwist(limited_twist, error);
                baseTwist->publish(synchronized_twist)   ;
                // return "RUNNING";
            }
        }
    }

    void DirectBaseControllerCoordinator::publish_zero_state()
    {
        
        baseTwist->publish(zero_twist);
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectBaseControllerCoordinator>());
  rclcpp::shutdown();
  return 0;
}