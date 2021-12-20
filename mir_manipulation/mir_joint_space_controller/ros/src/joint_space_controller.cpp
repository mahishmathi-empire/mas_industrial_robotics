#include <mir_joint_space_controller/joint_space_controller.h>
#include <mir_joint_space_controller/utils.h>
#include <yaml-cpp/yaml.h>

JointSpaceController::JointSpaceController():
    nh_("~")
{
    /* ros params */
    nh_.param<bool>("debug", debug_, true);
    nh_.param<float>("control_rate", control_rate_, 10.0f);
    control_sample_time_ = 1.0f/control_rate_;
    nh_.param<float>("goal_tolerance", goal_tolerance_, 0.01f);
    nh_.param<bool>("open_loop", open_loop_, false);

    active_ = false;
    goal_.fill(0.0f);
    current_.fill(0.0f);
    curr_vel_.fill(0.0f);

    // TODO read from config file or ros param
    max_vel_.fill(0.5f);
    min_vel_.fill(0.001f);
    max_acc_.fill(1.0f);
    des_acc_.fill(0.8f);

    // TODO read from config file or ros param
    for ( size_t i = 0; i < current_.size(); i++ )
    {
        joint_names_.push_back("arm_joint_" + std::to_string(i+1));
    }

    /* read named configurations */
    readNamedConfig();

    traj_index_ = 0;

    /* publishers */
    joint_vel_pub_ = nh_.advertise<brics_actuator::JointVelocities>("joint_vel", 1);

    /* subscribers */
    goal_sub_ = nh_.subscribe("goal", 1, &JointSpaceController::goalCb, this);
    named_goal_sub_ = nh_.subscribe("named_goal", 1, &JointSpaceController::namedGoalCb, this);
    cancel_sub_ = nh_.subscribe("cancel", 1, &JointSpaceController::cancelCb, this);
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &JointSpaceController::jointStatesCb, this);
    joy_sub_ = nh_.subscribe("joy", 1, &JointSpaceController::joyCb, this);

    tf_listener_.reset(new tf::TransformListener);

    ros::Duration(0.2).sleep();

    std::cout << std::setprecision(3) << std::fixed;

    ROS_INFO("JointSpaceController initialised");
    std::cout << Utils::getMsgMod("success")
              << "[JointSpaceController] Initialised"
              << Utils::getMsgMod("end") << std::endl;
}

JointSpaceController::~JointSpaceController()
{
}

void JointSpaceController::goalCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if ( msg->data.size() != goal_.size() )
    {
        std::cout << Utils::getMsgMod("err")
                  << "[JointSpaceController] Goal message has size"
                  << msg->data.size() << " but expecting size "
                  << goal_.size() << ". Ignoring."
                  << Utils::getMsgMod("end") << std::endl;
    }

    // TODO: check goal pos with joint limits

    for ( size_t i = 0; i < goal_.size(); i++ )
    {
        goal_[i] = msg->data[i];
    }
    // FIXME
    // active_ = true;
}

void JointSpaceController::namedGoalCb(const std_msgs::String::ConstPtr& msg)
{
    if ( named_configurations_.find(msg->data) == named_configurations_.end() )
    {
        std::cout << Utils::getMsgMod("warn")
                  << "[JointSpaceController] Received unknown named goal '"
                  << msg->data << "'."
                  << Utils::getMsgMod("end") << std::endl;
        return;
    }

    if ( active_ )
    {
        std::cout << Utils::getMsgMod("warn")
                  << "[JointSpaceController] Received named goal."
                  << " Aborting current goal."
                  << Utils::getMsgMod("end") << std::endl;
        reset();
    }

    goal_ = named_configurations_[msg->data];
    JointValue min_req_times;
    float max_req_time = 0.0f;
    size_t slowest_joint = 0;
    for ( size_t i = 0; i < goal_.size(); i++ )
    {
        min_req_times[i] = calcMinimumRequiredTime(current_[i], goal_[i],
                                                   max_vel_[i], des_acc_[i]);
        if ( min_req_times[i] > max_req_time )
        {
            max_req_time = min_req_times[i];
            slowest_joint = i;
        }
    }
    std::cout << "req_time: " << max_req_time << std::endl;
    std::cout << "slowest_joint: " << slowest_joint << std::endl;

    std::vector<float> t_array = calcTrajSingleJoint(
            current_[slowest_joint], goal_[slowest_joint],
            max_vel_[slowest_joint], des_acc_[slowest_joint]);

    traj_ = calcArmTraj(current_, goal_, t_array);
    traj_index_ = 0;

    active_ = true;
}

void JointSpaceController::cancelCb(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << Utils::getMsgMod("warn")
              << "[JointSpaceController] Preempting due to cancel message"
              << Utils::getMsgMod("end") << std::endl;
    reset();
}

void JointSpaceController::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    for ( size_t i = 0; i < msg->name.size(); i++ )
    {
        for ( size_t j = 0; j < joint_names_.size(); j++ )
        {
            if ( msg->name[i] == joint_names_[j] )
            {
                current_[j] = msg->position[i];
                if ( !open_loop_ )
                {
                    curr_vel_[j] = msg->velocity[i];
                }
            }
        }
    }
}

void JointSpaceController::joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    if ( active_ && msg->buttons[5] == 1 )
    {

        std::cout << Utils::getMsgMod("warn")
                  << "[JointSpaceController] Preempting due to joypad interrupt"
                  << Utils::getMsgMod("end") << std::endl;
        reset();
    }
}

void JointSpaceController::run(const ros::TimerEvent& event)
{
    if ( !active_ )
    {
        return;
    }

    std::cout << "current: " << current_ << std::endl;
    std::cout << "curr_vel: " << curr_vel_ << std::endl;
    std::cout << "goal: " << goal_ << std::endl;

    traj_index_++;
    bool traj_exec_complete = ( traj_index_ >= traj_.size() );
    JointValue target = ( traj_exec_complete ) ? goal_ : traj_[traj_index_];
    JointValue error = target - current_;

    /* check goal tolerance */
    if ( traj_exec_complete )
    {
        std::cout << "traj_exec_complete" << std::endl;
        bool reached_goal = true;
        for ( size_t i = 0; i < error.size(); i++ )
        {
            if ( fabs(error[i]) > goal_tolerance_ )
            {
                reached_goal = false;
                break;
            }
        }

        if ( reached_goal )
        {
            std::cout << Utils::getMsgMod("success")
                      << "[JointSpaceController] REACHED GOAL"
                      << Utils::getMsgMod("end") << std::endl;
            reset();
            return;
        }
    }

    JointValue raw_vel = error * control_rate_;
    JointValue vel;
    vel.fill(0.0f);
    for ( size_t i = 0; i < error.size(); i++ )
    {
        float raw_vel_clipped = Utils::signedClip(raw_vel[i], max_vel_[i], min_vel_[i]);
        float req_acc = (raw_vel_clipped - curr_vel_[i]) * control_rate_;
        float acc = Utils::clip(req_acc, max_acc_[i], -max_acc_[i]);
        vel[i] = curr_vel_[i] + (acc * control_sample_time_);
        if ( open_loop_ )
        {
            curr_vel_[i] = vel[i];
        }
    }
    std::cout << "target: " << target << std::endl;
    std::cout << "err: " << error << std::endl;
    std::cout << "raw_vel: " << raw_vel << std::endl;
    std::cout << "vel: " << vel << std::endl;

    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
    std::cout << std::endl;
}

// void JointSpaceController::run(const ros::TimerEvent& event)
// {
//     if ( !active_ )
//     {
//         return;
//     }

//     std::cout << "current: " << current_ << std::endl;
//     std::cout << "curr_vel: " << curr_vel_ << std::endl;
//     std::cout << "goal: " << goal_ << std::endl;

//     JointValue error, vel;
//     vel.fill(0.0f);
//     for ( size_t i = 0; i < error.size(); i++ )
//     {
//         error[i] = goal_[i] - current_[i];
//         if ( fabs(error[i]) < goal_tolerance_ )
//         {
//             error[i] = 0.0f;
//             continue;
//         }
//         float raw_vel = error[i];
//         float raw_vel_clipped = Utils::signedClip(raw_vel, max_vel_[i], min_vel_[i]);
//         float req_acc = (raw_vel_clipped - curr_vel_[i]) * control_rate_;
//         float acc = Utils::clip(req_acc, max_acc_[i], -max_acc_[i]);
//         vel[i] = curr_vel_[i] + (acc * control_sample_time_);
//         if ( open_loop_ )
//         {
//             curr_vel_[i] = vel[i];
//         }
//     }
//     std::cout << "err: " << error << std::endl;
//     std::cout << "vel: " << vel << std::endl;

//     bool reached_goal = true;
//     for ( size_t i = 0; i < error.size(); i++ )
//     {
//         if ( error[i] != 0.0f )
//         {
//             reached_goal = false;
//             break;
//         }
//     }
//     if ( reached_goal )
//     {
//         std::cout << Utils::getMsgMod("success")
//                   << "[JointSpaceController] REACHED GOAL"
//                   << Utils::getMsgMod("end") << std::endl;
//         reset();
//         return;
//     }
//     // joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
//     std::cout << std::endl;
// }

float JointSpaceController::calcMinimumRequiredTime(float curr, float goal,
                                                    float max_vel, float max_acc)
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

std::vector<float> JointSpaceController::calcTrajSingleJoint(
        float curr, float goal, float max_vel, float max_acc)
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
    size_t num_of_control = std::ceil(min_req_time / control_sample_time_) + 1;
    // std::cout << "num_of_control: " << num_of_control << std::endl;
    std::vector<float> times(num_of_control, 0.0f);
    std::vector<float> t_array(num_of_control, 0.0f);
    std::vector<float> joint_angles(num_of_control, 0.0f);
    for ( size_t i = 0; i < num_of_control; i++ )
    {
        times[i] = control_sample_time_ * i;
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
    // for ( size_t i = 0; i < num_of_control; i++ )
    // {
    //     std::cout << times[i] << " " << joint_angles[i] << " " << t_array[i] << std::endl;
    // }
    // std::cout << std::endl;
    return t_array;
}

std::vector<JointValue> JointSpaceController::calcArmTraj(const JointValue& curr,
        const JointValue& goal, const std::vector<float>& t_array)
{
    std::vector<JointValue> traj(t_array.size());
    float t;
    for ( size_t i = 0; i < t_array.size(); i++ )
    {
        t = t_array[i];
        for ( size_t j = 0; j < curr.size(); j++ )
        {
            traj[i][j] = ((1.0f - t) * curr[j]) + (t * goal[j]);
        }
        std::cout << (control_sample_time_ * i) << " " << t << " " << traj[i] << std::endl;
    }
    return traj;
}

void JointSpaceController::reset()
{
    active_ = false;
    goal_.fill(0.0f);
    pubZeroVel();
    traj_index_ = 0;
    traj_.clear();
}

void JointSpaceController::pubZeroVel()
{
    JointValue vel;
    vel.fill(0.0f);
    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
}

bool JointSpaceController::readNamedConfig()
{
    std::string named_configuration_file;
    nh_.param<std::string>("named_configuration_file", named_configuration_file, "");

    if ( named_configuration_file == "" )
    {
        std::cerr << Utils::getMsgMod("err")
                  << "Named configuration file not provided"
                  << Utils::getMsgMod("end") << std::endl;
        return false;
    }

    YAML::Node named_configuration_yaml;
    try
    {
        named_configuration_yaml = YAML::LoadFile(named_configuration_file);
    }
    catch( YAML::BadFile )
    {
        std::cerr << Utils::getMsgMod("err")
                  << "YAML threw BadFile exception. Does the file exist?" 
                  << std::endl << named_configuration_file
                  << Utils::getMsgMod("end") << std::endl;
        return false;
    }
    catch( const YAML::ParserException& e )
    {
        std::cerr << Utils::getMsgMod("err")
                  << "YAML parsing error" << std::endl << e.what()
                  << Utils::getMsgMod("end") << std::endl;
        return false;
    }

    if ( !named_configuration_yaml.IsMap() )
    {
        std::cerr << Utils::getMsgMod("err")
                  << "Named configuration file does not have a correct format."
                  << Utils::getMsgMod("end") << std::endl;
        return false;
    }

    /* read individual named joint configurations */
    for ( YAML::const_iterator it = named_configuration_yaml.begin();
          it != named_configuration_yaml.end(); ++it )
    {
        std::string joint_config_name = it->first.as<std::string>();

        if ( named_configurations_.find(joint_config_name) != named_configurations_.end() )
        {
            std::cerr << Utils::getMsgMod("err")
                      << "Named configurations file contains multiple joint"
                      << " configs with same name: " << joint_config_name
                      << Utils::getMsgMod("end") << std::endl;
            named_configurations_.clear();
            return false;
        }

        std::vector<float> joint_config_vec = it->second.as<std::vector<float>>();
        if ( joint_config_vec.size() != current_.size() )
        {
            std::cerr << Utils::getMsgMod("err")
                      << "Named configurations file contains joint config"
                      << " with incorrect num of values: " << joint_config_name
                      << Utils::getMsgMod("end") << std::endl;
            continue;
        }
        JointValue joint_config;
        for ( size_t i = 0; i < joint_config.size(); i++ )
        {
            joint_config[i] = joint_config_vec[i];
        }
        named_configurations_[joint_config_name] = joint_config;
    }

    for ( auto itr = named_configurations_.begin(); itr != named_configurations_.end(); itr++ )
    {
        std::cout << itr->first << " " << itr->second << std::endl;
    }


    return true;
}
