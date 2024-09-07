#ifndef TWIST_SYNCHRONIZER_HPP
#define TWIST_SYNCHRONIZER_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "twist_synchronizer_utils.hpp"
#include "component_wise_pose_error_calculator.hpp"
class TwistSynchronizer
{
public:
    TwistSynchronizer();
    void setAngularSynchronization(bool angularSynchronization);
    geometry_msgs::msg::Twist synchronizeTwist(const geometry_msgs::msg::Twist &twist, const ComponentWiseCartesianDifference &poseError);

private:
    bool angularSynchronization;
    double nearZero;

    double calculateMaxTime(const std::vector<double> &error, const std::vector<double> &velocity);
    std::vector<double> calculateSyncVelocity(const std::vector<double> &error, const std::vector<double> &velocity, double maxTime);
};

#endif // TWIST_SYNCHRONIZER_HPP
