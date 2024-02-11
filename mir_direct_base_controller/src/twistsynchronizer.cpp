#include "twistsynchronizer.hpp"
#include "twist_synchronizer_utils.hpp"

TwistSynchronizer::TwistSynchronizer() : angularSynchronization(true),
                                         nearZero(0.001){}

/**
 * Setting if synchronization is needed.
 */

void TwistSynchronizer::setAngularSynchronization(bool angularSynchronization)
{
    this->angularSynchronization = angularSynchronization;
}

/**
 * Synchronizing twist values.
 */
geometry_msgs::msg::Twist TwistSynchronizer::synchronizeTwist(const geometry_msgs::msg::Twist &twist, const ComponentWiseCartesianDifference &poseError)
{
    geometry_msgs::msg::Twist synchronizedTwist;

    std::vector<double> error;
    std::vector<double> velocity;

    if (angularSynchronization)
    {
        error = {poseError.linear_x, poseError.linear_y, poseError.linear_z,
                 poseError.angular_x, poseError.angular_y, poseError.angular_z};

        velocity = {twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.x, twist.angular.y, twist.angular.z};
    }
    else
    {
        error = {poseError.linear_x, poseError.linear_y, poseError.linear_z};
        velocity = {twist.linear.x, twist.linear.y, twist.linear.z};
    }

    double maxTime = calculate_max_time(error, velocity, angularSynchronization, nearZero);
    std::vector<double> syncVelocities = calculate_sync_velocity(error, velocity, maxTime, angularSynchronization);

    synchronizedTwist.linear.x = syncVelocities[0];
    synchronizedTwist.linear.y = syncVelocities[1];
    synchronizedTwist.linear.z = syncVelocities[2];

    if (angularSynchronization)
    {
        synchronizedTwist.angular.x = syncVelocities[3];
        synchronizedTwist.angular.y = syncVelocities[4];
        synchronizedTwist.angular.z = syncVelocities[5];
    }

    return synchronizedTwist;
}
