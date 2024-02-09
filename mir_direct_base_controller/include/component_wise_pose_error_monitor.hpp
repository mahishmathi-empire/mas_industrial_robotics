#ifndef COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
#define COMPONENT_WISE_POSE_ERROR_MONITOR_HPP

#include "component_wise_pose_error_calculator.hpp"

class ComponentWisePoseErrorMonitor
{
public:
    ComponentWisePoseErrorMonitor();
    void setParameters(double threshold_linear_x, double threshold_linear_y, double threshold_linear_z,
                       double threshold_angular_x, double threshold_angular_y, double threshold_angular_z);
    bool isComponentWisePoseErrorWithinThreshold(const ComponentWiseCartesianDifference &pose_error);

private:
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
    double use_collision_avoidance;
    double loop_rate;
};

#endif // COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
