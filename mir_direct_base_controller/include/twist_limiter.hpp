#ifndef TWIST_LIMITER_HPP
#define TWIST_LIMITER_HPP

struct LimiterParameters{
        double max_velocity_x;
        double max_velocity_y;
        double max_velocity_z;
        double max_velocity_yaw;
        double max_velocity_roll;
        double max_velocity_pitch;
};
geometry_msgs::msg::Twist get_limited_twist(const geometry_msgs::msg::Twist & twist, const LimiterParameters & lp );
#endif