#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "component_wise_pose_error_calculator.hpp"
#include "component_wise_pose_error_monitor.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class DirectBaseController : public rclcpp::Node
{
  public:
  //'node - class name'
  //'DirectBaseController- constructor'
    DirectBaseController()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&DirectBaseController::topic_callback, this, _1));
      //'this referes to pointer to the object of the class'
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {

   

      auto message = geometry_msgs::msg::Twist();
      RCLCPP_INFO(this->get_logger(), "I heard: '%f %f'", msg->pose.position.x, msg->pose.position.y);

      auto origin_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
      origin_pose->header.frame_id = "odom"; 
      origin_pose->pose.orientation.w = 1.0;
      ComponentWiseCartesianDifference error;
      bool success = get_component_wise_pose_error(origin_pose, msg, error);
      RCLCPP_INFO(this->get_logger(), "Error found: %f", error.linear_x);
      ComponentWisePoseErrorMonitor monitor;
      monitor.setParameters(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
      bool is_within_threshold = monitor.isComponentWisePoseErrorWithinThreshold(error);
      RCLCPP_INFO(this->get_logger(), "Is within threshold: %d", is_within_threshold);

      

      message.linear.x = 0.1; // set the linear x value

      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);

      publisher_->publish(message);


      
    }
    //'member name - subscription_'
    // 'rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr - variable, subdcription - type'
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectBaseController>());
  rclcpp::shutdown();
  return 0;
}