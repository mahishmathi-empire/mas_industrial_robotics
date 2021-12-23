#include <mir_joint_space_controller/model.h>

std::vector<JointValue> Model::calculateTrajectory(const JointValue& current,
        const JointValue& curr_vel, const JointValue& acc,
        const std::vector<float>& sample_times)
{
    // std::cout << "inside Model::calculateTrajectory" << std::endl;
    std::vector<JointValue> traj(sample_times.size());

    // std::cout << "current: " << current << std::endl;
    // std::cout << "curr_vel: " << curr_vel << std::endl;
    // std::cout << "acc: " << acc << std::endl;

    /* v = v0 + a*t */
    JointValue vel = curr_vel + (acc * sample_times[0]);
    // std::cout << "vel: " << vel << std::endl;

    /* d = d0 + v0*t + 0.5*a*t^2 */
    traj[0] = current + (curr_vel * sample_times[0])
                      + (acc * (0.5f * sample_times[0] * sample_times[0]));

    for ( size_t i = 1; i < sample_times.size(); i++ )
    {
        /* d = d0 + v*t */
        traj[i] = traj[i-1] + (vel * sample_times[i]);
    }

    return traj;
}
