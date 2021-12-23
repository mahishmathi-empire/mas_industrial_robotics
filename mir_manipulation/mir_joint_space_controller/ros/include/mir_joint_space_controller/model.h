#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <mir_joint_space_controller/joint_value.h>

class Model
{
    public:

        /* 
         * Apply acceleration for first sample time, then apply 0 accelerations
         * for the rest of the trajectory
         *
         * ASSUMPTIONS
         * accelerations are constant over sample_time and can be applied
         * instantaneously (i.e. no jerk limits)
         */
        static std::vector<JointValue> calculateTrajectory(
                const JointValue& current,
                const JointValue& curr_vel,
                const JointValue& acc,
                const std::vector<float>& sample_times);

};

#endif // MPCN_MODEL_H
