#ifndef TWIST_CONTROLLER_HPP
#define TWIST_CONTROLLER_HPP

struct ControllerParameters
{
        double p_gain_x ;
        double p_gain_y ;
        double p_gain_z ;
        double p_gain_roll ;
        double p_gain_pitch ;
        double p_gain_yaw ;
};

geometry_msgs::msg::Twist get_cartesian_velocity(const ComponentWiseCartesianDifference & cw, const ControllerParameters & cp );
#endif