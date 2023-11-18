#include "component_wise_pose_error_monitor.hpp"

ComponentWisePoseErrorMonitor::ComponentWisePoseErrorMonitor()
    : threshold_linear_x(0), threshold_linear_y(0), threshold_linear_z(0),
      threshold_angular_x(0), threshold_angular_y(0), threshold_angular_z(0)
{
}

void ComponentWisePoseErrorMonitor::setParameters(double threshold_linear_x, double threshold_linear_y, double threshold_linear_z,
                                                  double threshold_angular_x, double threshold_angular_y, double threshold_angular_z)
{
    this->threshold_linear_x = threshold_linear_x;
    this->threshold_linear_y = threshold_linear_y;
    this->threshold_linear_z = threshold_linear_z;
    this->threshold_angular_x = threshold_angular_x;
    this->threshold_angular_y = threshold_angular_y;
    this->threshold_angular_z = threshold_angular_z;
}

bool ComponentWisePoseErrorMonitor::isComponentWisePoseErrorWithinThreshold(const mcr_manipulation_msgs::ComponentWisePoseError& pose_error)
{
    feedback.is_linear_x_within_tolerance = std::abs(pose_error.linear.x) < threshold_linear_x;
    feedback.is_linear_y_within_tolerance = std::abs(pose_error.linear.y) < threshold_linear_y;
    feedback.is_linear_z_within_tolerance = std::abs(pose_error.linear.z) < threshold_linear_z;
    feedback.is_angular_x_within_tolerance = std::abs(pose_error.angular.x) < threshold_angular_x;
    feedback.is_angular_y_within_tolerance = std::abs(pose_error.angular.y) < threshold_angular_y;
    feedback.is_angular_z_within_tolerance = std::abs(pose_error.angular.z) < threshold_angular_z;
    return (feedback.is_linear_x_within_tolerance &&
            feedback.is_linear_y_within_tolerance &&
            feedback.is_linear_z_within_tolerance &&
            feedback.is_angular_x_within_tolerance &&
            feedback.is_angular_y_within_tolerance &&
            feedback.is_angular_z_within_tolerance);
}
