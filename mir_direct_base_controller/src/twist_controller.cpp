#include "component_wise_pose_error_calculator.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "twist_controller.hpp"

double control(double proportional_constant, double  set_value, double  current_value){
    double error = set_value - current_value;
    double control_value= current_value + (proportional_constant * error);
    return control_value;
}

        
geometry_msgs::msg::Twist get_cartesian_velocity(const ComponentWiseCartesianDifference & cw, const ControllerParameters & cp )
{
geometry_msgs::msg::Twist cartesian_velocity;
cartesian_velocity.linear.x=control(cp.p_gain_x,cw.linear_x,0);
cartesian_velocity.linear.y=control(cp.p_gain_y,cw.linear_y,0);
cartesian_velocity.linear.z=control(cp.p_gain_z,cw.linear_z,0);
cartesian_velocity.angular.x=control(cp.p_gain_roll,cw.angular_x,0);
cartesian_velocity.angular.y=control(cp.p_gain_pitch,cw.angular_y,0);
cartesian_velocity.angular.z=control(cp.p_gain_yaw,cw.angular_z,0);
return cartesian_velocity;
}
