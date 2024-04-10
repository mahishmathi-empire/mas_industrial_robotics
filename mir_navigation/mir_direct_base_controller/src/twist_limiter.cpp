#include <geometry_msgs/msg/twist.hpp>
#include "twist_limiter.hpp"
/**
 * Getting an allowed value for current twist value compared with max allowed twist value.
 */
double limit_value(double value, double limit)
{
    double limited_value = 0;
    if (value > limit)
        limited_value = limit;
    else if (value < -limit)
        limited_value = -limit;
    else
        return value;
    return limited_value;
}
/**
 * If any of twist values violated max allowed value, we replace it.
 */
geometry_msgs::msg::Twist get_limited_twist(const geometry_msgs::msg::Twist &twist, const LimiterParameters &lp)
{
    geometry_msgs::msg::Twist limited_twist;
    limited_twist.linear.x = limit_value(twist.linear.x, lp.max_velocity_x);
    limited_twist.linear.y = limit_value(twist.linear.y, lp.max_velocity_y);
    limited_twist.linear.z = limit_value(twist.linear.z, lp.max_velocity_z);
    limited_twist.angular.x = limit_value(twist.angular.x, lp.max_velocity_roll);
    limited_twist.angular.y = limit_value(twist.angular.y, lp.max_velocity_pitch);
    limited_twist.angular.z = limit_value(twist.angular.z, lp.max_velocity_yaw);
    return limited_twist;
}