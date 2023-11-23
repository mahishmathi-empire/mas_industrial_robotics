#ifndef COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
#define COMPONENT_WISE_POSE_ERROR_MONITOR_HPP

#include "component_wise_pose_error_calculator.hpp"


class ComponentWisePoseErrorMonitor
{
public:
    ComponentWisePoseErrorMonitor();
    void setParameters(double threshold_linear_x, double threshold_linear_y, double threshold_linear_z,
                       double threshold_angular_x, double threshold_angular_y, double threshold_angular_z);
    bool isComponentWisePoseErrorWithinThreshold(const ComponentWiseCartesianDifference & pose_error);

private:
    
    double threshold_linear_x;
    double threshold_linear_y;
    double threshold_linear_z;
    double threshold_angular_x;
    double threshold_angular_y;
    double threshold_angular_z;
};

#endif // COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
