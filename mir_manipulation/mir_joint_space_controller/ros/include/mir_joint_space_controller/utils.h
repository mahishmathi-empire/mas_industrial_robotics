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
                const JointValue& joint_vel,
                const std::vector<std::string>& joint_names);

        static std::vector<float> getPascalTriangleRow(size_t n);

        static JointValue getSplineCurvePoint(
                const std::vector<JointValue>& control_points,
                const std::vector<float>& coefficients,
                const float t);

        static JointValue getJointValueAtTime(const std::vector<JointValue>& traj,
                                              float time_from_start,
                                              float control_sample_time);

        static float calcMinimumRequiredTime(float curr, float goal, float max_vel,
                                             float max_acc);

        static std::vector<float> calcTrajSingleJoint(float curr, float goal,
                                                      float max_vel, float max_acc,
                                                      float control_sample_time);

        static std::vector<JointValue> calcSplineTrajectory(
                const std::vector<JointValue>& control_points,
                const std::vector<float>& t_array);

};

#endif // UTILS_H
