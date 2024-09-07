#include "twist_synchronizer_utils.hpp"
// Function to calculate the sign of a number

int sign(double val)
{
    if (val > 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

double cal_dist(double dist, double speed)
{
    return std::abs(dist / speed);
}

/**
 * Calculating the maximum time required to reach the goal
 * */
double calculate_max_time(const std::vector<double> &error, const std::vector<double> &velocity, bool angular_synchronization, double zero)
{
    assert(error.size() == velocity.size());

    if (angular_synchronization)
    {
        assert(error.size() == 6);
    }
    else
    {
        assert(error.size() == 3);
    }

    std::vector<double> durations;
    for (size_t i = 0; i < error.size(); ++i)
    {
        double duration = (std::abs(velocity[i]) >= zero) ? cal_dist(error[i], velocity[i]) : 0.0;
        durations.push_back(duration);
    }

    return *std::max_element(durations.begin(), durations.end());
}

double synchronize_velocity(double dist, double vel, double max_time)
{
    return (max_time && vel) ? (std::abs(dist) / max_time) * sign(vel) : 0.0;
}

/**
 * Calculating the synchronized velocity
 */
std::vector<double> calculate_sync_velocity(const std::vector<double> &error, const std::vector<double> &velocity, double max_time, bool angular_synchronization)
{
    assert(error.size() == velocity.size());

    if (angular_synchronization)
    {
        assert(error.size() == 6);
    }
    else
    {
        assert(error.size() == 3);
    }

    std::vector<double> synchronized_velocities;
    for (size_t i = 0; i < error.size(); ++i)
    {
        synchronized_velocities.push_back(synchronize_velocity(error[i], velocity[i], max_time));
    }

    return synchronized_velocities;
}
