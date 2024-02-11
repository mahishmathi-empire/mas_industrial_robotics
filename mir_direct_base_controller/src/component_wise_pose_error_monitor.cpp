
#include "component_wise_pose_error_monitor.hpp"

ComponentWisePoseErrorMonitor::ComponentWisePoseErrorMonitor()
    : threshold_linear_x(0), threshold_linear_y(0), threshold_linear_z(0),
      threshold_angular_x(0), threshold_angular_y(0), threshold_angular_z(0)
{
}

/**
 * Setting params for the component error checker.
*/
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

/**
 * Checking if error is violating any threshold limit or not.
*/
bool ComponentWisePoseErrorMonitor::isComponentWisePoseErrorWithinThreshold(const ComponentWiseCartesianDifference & pose_error)
{
    bool is_linear_x_within_tolerance = std::abs(pose_error.linear_x) < threshold_linear_x;
    bool is_linear_y_within_tolerance = std::abs(pose_error.linear_y) < threshold_linear_y;
    bool is_linear_z_within_tolerance = std::abs(pose_error.linear_z) < threshold_linear_z;
    bool is_angular_x_within_tolerance = std::abs(pose_error.angular_x) < threshold_angular_x;
    bool is_angular_y_within_tolerance = std::abs(pose_error.angular_y) < threshold_angular_y;
    bool is_angular_z_within_tolerance = std::abs(pose_error.angular_z) < threshold_angular_z;
    return (is_linear_x_within_tolerance &&
            is_linear_y_within_tolerance &&
            is_linear_z_within_tolerance &&
            is_angular_x_within_tolerance &&
            is_angular_y_within_tolerance &&
            is_angular_z_within_tolerance);
}
