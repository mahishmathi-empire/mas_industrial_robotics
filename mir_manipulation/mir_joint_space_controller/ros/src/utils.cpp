#include <mir_joint_space_controller/utils.h>
#include <math.h>

#include <brics_actuator/JointValue.h>

std::string Utils::getMsgMod(const std::string& command)
{
    if ( command == "err" )
    {
        return "\033[31m";
    }
    else if ( command == "warn" )
    {
        return "\033[33m";
    }
    else if ( command == "success" )
    {
        return "\033[32m";
    }
    else if ( command == "end" )
    {
        return "\033[0m";
    }
    else
    {
        return "";
    }
}

float Utils::clip(float value, float max_limit, float min_limit)
{
    return std::max(std::min(value, max_limit), min_limit);
}

float Utils::signedClip(float value, float max_limit, float min_limit)
{
    int sign = ( value >= 0.0f ) ? 1 : -1;
    return sign * std::max(std::min(fabs(value), max_limit), min_limit);
}

brics_actuator::JointVelocities Utils::getJointVelocitiesFromJointValue(
        const JointValue& joint_vel)
{
    brics_actuator::JointVelocities joint_vel_msg;
    for ( size_t i = 0; i < joint_vel.size(); i++ )
    {
        brics_actuator::JointValue joint_value_msg;
        joint_value_msg.joint_uri = "arm_joint_" + std::to_string(i+1);
        joint_value_msg.unit = "s^-1 rad";
        joint_value_msg.value = joint_vel[i];
        joint_vel_msg.velocities.push_back(joint_value_msg);
    }
    return joint_vel_msg;
}

