#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>

#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <std_msgs/Float32MultiArray.h>

#include <mir_joint_space_controller/joint_value.h>

class Utils
{
    public:

        static std::string getMsgMod(const std::string& command);

        static float clip(float value, float max_limit, float min_limit);

        static float signedClip(float value, float max_limit, float min_limit);

        static brics_actuator::JointVelocities getJointVelocitiesFromJointValue(
                const JointValue& joint_vel);

};

#endif // UTILS_H
