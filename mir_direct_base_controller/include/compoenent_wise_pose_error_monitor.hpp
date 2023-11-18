#ifndef COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
#define COMPONENT_WISE_POSE_ERROR_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <mcr_manipulation_msgs/ComponentWisePoseError.h>
#include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.h>

class ComponentWisePoseErrorMonitor
{
public:
    ComponentWisePoseErrorMonitor();
    void setParameters(double threshold_linear_x, double threshold_linear_y, double threshold_linear_z,
                       double threshold_angular_x, double threshold_angular_y, double threshold_angular_z);
    bool isComponentWisePoseErrorWithinThreshold(const mcr_manipulation_msgs::ComponentWisePoseError& pose_error);

private:
    mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback feedback;
    double threshold_linear_x;
    double threshold_linear_y;
    double threshold_linear_z;
    double threshold_angular_x;
    double threshold_angular_y;
    double threshold_angular_z;
};

#endif // COMPONENT_WISE_POSE_ERROR_MONITOR_HPP
