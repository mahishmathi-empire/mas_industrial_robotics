#include "direct_base_controller_coordinator.hpp"

DirectBaseControllerCoordinator::DirectBaseControllerCoordinator()
    : Node("direct_base_controller")
    //   useCollisionAvoidance(get_parameter_or<bool>("use_collision_avoidance", true)) 
{
    baseTwist = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    targetPose = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&DirectBaseControllerCoordinator::targetPoseCallback, this, std::placeholders::_1));
    target_pose_received = false;
    // double loop_rate; 
    // get_parameter_or<double>("loop_rate",loop_rate,100.0); 
    // loopRate = rclcpp::Rate(loop_rate);
    
    // double idle_loop_rate;
    // get_parameter_or<double>("idle_loop_rate",idle_loop_rate,1.0); 
    // idleLoopRate = rclcpp::Rate(idle_loop_rate);

  
    get_parameter_or<std::string>("base_frame", baseFrame,"base_link");
    

    // collisionFilter = create_subscription<mcr_monitoring_msgs::msg::ComponentWisePoseErrorMonitorFeedback>("collision_filter", 1, std::bind(&DirectBaseControllerCoordinator::collisionFilterCallback, this, std::placeholders::_1));
    // laserDistances = creaSte_subscription<std_msgs::msg::Float32MultiArray>("/mcr_navigation/laser_distances/distances", 1, std::bind(&DirectBaseControllerCoordinator::laserDistancesCallback, this, std::placeholders::_1));
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
        std::string state = "RUNNING";

        // while (rclcpp::ok()) {
        //     if (state == "INIT") {
        //         initState();
        //     } else if (state == "RUNNING") {
        //         runningState();
        //     }

        //     RCLCPP_DEBUG(get_logger(), "State: %s", state.c_str());
        //     if (state == "INIT") {
        //         idleLoopRate.sleep();
        //     } else if (state == "RUNNING") {
        //         loopRate.sleep();
        //     }
            while (rclcpp::ok())
            {
                runningState();
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
        
        if(target_pose_received){

        
            bool poseError = get_component_wise_pose_error(originPose, targetPoseMsg, error);
            
            if(poseError)
            {
                publish_zero_state();
                // return "INIT";
            }
            ComponentWisePoseErrorMonitor monitor;
            monitor.setParameters(0.02, 0.02, 15, 15, 15, 0.04);
            if(monitor.isComponentWisePoseErrorWithinThreshold(error))
            {
                RCLCPP_INFO(get_logger(), "error within threshold");
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
    {15
        
        baseTwist->publish(zero_twist);
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectBaseControllerCoordinator>();
//   rclcpp::spin(std::make_shared<DirectBaseControllerCoordinator>());
    node->start();
//   run_node.start();  
//   rclcpp::shutdown();
  return 0;
}