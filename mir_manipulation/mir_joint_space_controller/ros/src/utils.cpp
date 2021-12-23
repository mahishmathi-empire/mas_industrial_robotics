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

JointValue Utils::interpolateLinearly(const JointValue& a, const JointValue& b, float t)
{
    return (a * (1.0f - t)) + (b * t);
}

JointValue Utils::clip(const JointValue& value, const JointValue& max_limit, const JointValue& min_limit)
{
    JointValue clipped_val;
    for ( size_t i = 0; i < value.size(); i++ )
    {
        clipped_val[i] = Utils::clip(value[i], max_limit[i], min_limit[i]);
    }
    return clipped_val;
}

JointValue Utils::signedClip(const JointValue& value, const JointValue& max_limit, const JointValue& min_limit)
{
    JointValue clipped_val;
    for ( size_t i = 0; i < value.size(); i++ )
    {
        clipped_val[i] = Utils::signedClip(value[i], max_limit[i], min_limit[i]);
    }
    return clipped_val;
}

JointValue Utils::normalisedClip(const JointValue& value, const JointValue& target, const JointValue& max_limit)
{
    JointValue clipped_val = target;
    for ( size_t i = 0; i < clipped_val.size(); i++ )
    {
        if ( target[i] <= max_limit[i] && target[i] >= -max_limit[i] )
        {
            continue;
        }
        float clipped_value = Utils::clip(clipped_val[i], max_limit[i], -max_limit[i]);
        float t = (clipped_value - value[i])/(target[i] - value[i]);
        clipped_val = Utils::interpolateLinearly(value, target, t);
    }
    return clipped_val;
}

float Utils::calcDistSq(const JointValue& jval_1, const JointValue& jval_2)
{
    float dist_sq = 0.0f;
    for ( size_t i = 0; i < jval_1.size(); i++ )
    {
        dist_sq += pow(jval_1[i] - jval_2[i], 2);
    }
    return dist_sq;
}

brics_actuator::JointVelocities Utils::getJointVelocitiesFromJointValue(
        const JointValue& joint_vel,
        const std::vector<std::string>& joint_names)
{
    brics_actuator::JointVelocities joint_vel_msg;
    if ( joint_names.size() != joint_vel.size() )
    {
        std::cout << Utils::getMsgMod("err")
                  << "[JointSpaceController] joint_names and joint_vel size mismatch."
                  << Utils::getMsgMod("end") << std::endl;
        return joint_vel_msg;
    }

    for ( size_t i = 0; i < joint_vel.size(); i++ )
    {
        brics_actuator::JointValue joint_value_msg;
        joint_value_msg.joint_uri = joint_names[i];
        joint_value_msg.unit = "s^-1 rad";
        joint_value_msg.value = joint_vel[i];
        joint_vel_msg.velocities.push_back(joint_value_msg);
    }
    return joint_vel_msg;
}

std::vector<float> Utils::getPascalTriangleRow(size_t n)
{
    std::vector<float> coefficents;
    coefficents.push_back(1);
    for ( size_t i = 1; i < n+1; ++i )
    {
        int coeff = coefficents.back() * ((float)(n + 1 - i)/i);
        coefficents.push_back(coeff);
    }
    return coefficents;
}

JointValue Utils::getSplineCurvePoint(const std::vector<JointValue>& control_points,
                                      const std::vector<float>& coefficients,
                                      const float t)
{
    JointValue curve_point;
    curve_point.fill(0.0f);
    size_t order = control_points.size() - 1;
    for ( size_t i = 0; i < order+1; ++i )
    {
        for ( size_t j = 0; j < curve_point.size(); j++ )
        {
            curve_point[j] += coefficients[i] * pow(1.0f-t, order-i) * pow(t, i) * control_points[i][j];
        }
    }
    return curve_point;
}

std::vector<JointValue> Utils::calcSplineTrajectory(
        const std::vector<JointValue>& control_points,
        const std::vector<float>& t_array)
{
    std::vector<JointValue> traj;
    if ( control_points.size() < 2 )
    {
        return traj;
    }
    std::vector<float> coefficients = Utils::getPascalTriangleRow(control_points.size() - 1);
    traj.resize(t_array.size());
    for ( size_t i = 0; i < t_array.size(); i++ )
    {
        traj[i] = Utils::getSplineCurvePoint(control_points, coefficients, t_array[i]);
    }
    return traj;
}

std::vector<JointValue> Utils::calcSplineTrajectory(
        const std::vector<JointValue>& control_points, size_t num_of_pts)
{
    /* generate t_array */
    std::vector<float> t_array(num_of_pts+1);
    float factor = 1.0f / num_of_pts;
    for ( size_t i = 0; i <= num_of_pts; i++ )
    {
        t_array[i] = i * factor;
    }
    return Utils::calcSplineTrajectory(control_points, t_array);
}

std::vector<JointValue> Utils::calcSplineTrajectory(
        const std::vector<JointValue>& control_points, float resolution)
{
    JointValue total_dist;
    for ( size_t i = 0; i+1 < control_points.size(); i++ )
    {
        for ( size_t j = 0; j < total_dist.size(); j++ )
        {
            total_dist[j] = fabs(control_points[i+1][j] - control_points[i][j]);
        }
    }
    float max_dist = 0.0f;
    for ( size_t i = 0; i < total_dist.size(); i++ )
    {
        if ( total_dist[i] > max_dist )
        {
            max_dist = total_dist[i];
        }
    }
    size_t num_of_pts = std::ceil(max_dist/resolution);
    return Utils::calcSplineTrajectory(control_points, num_of_pts);
}

JointValue Utils::getJointValueAtTime(const std::vector<JointValue>& traj,
                                      float time_from_start,
                                      float control_sample_time)
{
    float traj_index_float = time_from_start / control_sample_time;
    int traj_index = time_from_start / control_sample_time;
    float t = traj_index_float - traj_index;
    int traj_next_index = traj_index + 1;

    if ( traj_index >= traj.size() )
    {
        traj_index = traj.size() - 1;
    }
    if ( traj_next_index >= traj.size() )
    {
        traj_next_index = traj.size() - 1;
    }
    return (traj[traj_index] * (1.0f - t)) + (traj[traj_next_index] * t);
}

float Utils::calcMinimumRequiredTime(float curr, float goal, float max_vel,
                                     float max_acc)
{
    float D = fabs(goal - curr);
    float acc_time, const_vel_time;
    if ( D < (max_vel*max_vel)/max_acc )
    {
        // std::cout << "acc and dec" << std::endl;
        acc_time = sqrt(D/max_acc);
        const_vel_time = 0.0f;
    }
    else
    {
        // std::cout << "acc, const and dec" << std::endl;
        acc_time = max_vel/max_acc;
        const_vel_time = (D/max_vel) - (max_vel/max_acc);
    }
    float min_req_time = (2 * acc_time) + const_vel_time;
    return min_req_time;
}

std::vector<float> Utils::calcTrajSingleJoint(float curr, float goal,
                                              float max_vel, float max_acc,
                                              float control_sample_time)
{
    float D = fabs(goal - curr);
    float acc_time, const_vel_time, desired_vel;
    if ( D < (max_vel*max_vel)/max_acc )
    {
        acc_time = sqrt(D/max_acc);
        const_vel_time = 0.0f;
        desired_vel = max_acc * acc_time;
    }
    else
    {
        acc_time = max_vel/max_acc;
        const_vel_time = (D/max_vel) - (max_vel/max_acc);
        desired_vel = max_vel;
    }
    float min_req_time = (2 * acc_time) + const_vel_time;
    float acc_dist = 0.5 * max_acc * acc_time * acc_time;
    float const_vel_dist = const_vel_time * max_vel;
    // std::cout << "acc_time: " << acc_time << std::endl;
    // std::cout << "const_vel_time: " << const_vel_time << std::endl;
    // std::cout << "acc_dist: " << acc_dist << std::endl;
    // std::cout << "const_vel_dist: " << const_vel_dist << std::endl;
    // std::cout << "min_req_time: " << min_req_time << std::endl;
    size_t num_of_control = std::ceil(min_req_time / control_sample_time) + 1;
    // std::cout << "num_of_control: " << num_of_control << std::endl;
    std::vector<float> times(num_of_control, 0.0f);
    std::vector<float> t_array(num_of_control, 0.0f);
    std::vector<float> joint_angles(num_of_control, 0.0f);
    for ( size_t i = 0; i < num_of_control; i++ )
    {
        times[i] = control_sample_time * i;
    }
    joint_angles[0] = curr;
    t_array[0] = 0.0f;
    joint_angles.back() = goal;
    t_array.back() = 1.0f;
    float remaining_time;
    for ( size_t i = 1; i+1 < num_of_control; i++ )
    {
        if ( times[i] <= acc_time )
        {
            // accelerate
            joint_angles[i] = curr + (0.5f * max_acc * times[i] * times[i]);
        }
        else if ( times[i] <= acc_time + const_vel_time )
        {
            // const vel
            remaining_time = times[i] - acc_time;
            joint_angles[i] = curr + acc_dist + (max_vel * (remaining_time));
        }
        else
        {
            // deccelerate
            remaining_time = times[i] - (acc_time + const_vel_time);
            joint_angles[i] = curr + acc_dist + const_vel_dist
                             + (desired_vel * remaining_time)
                             - (0.5f * max_acc * remaining_time * remaining_time);
        }
        t_array[i] = (joint_angles[i] - curr) / D;
    }
    return t_array;
}

float Utils::getProjectedPointRatioOnSegment(
        const JointValue& a, const JointValue& b, const JointValue& p)
{
    float length_sq = Utils::calcDistSq(a, b);
    if ( length_sq == 0.0f )
    {
        return 0.0f;
    }
    float t_sum = 0.0f;
    for ( size_t i = 0; i < a.size(); i++ )
    {
        t_sum += (p[i] - a[i]) * (b[i] - a[i]);
    }
    float t = t_sum/length_sq;
    return Utils::clip(t, 1.0f, 0.0f);
}
