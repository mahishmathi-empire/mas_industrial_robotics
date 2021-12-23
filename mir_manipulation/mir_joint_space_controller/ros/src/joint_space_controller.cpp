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
    nh_.param<float>("lookahead_time", lookahead_time_, 0.2f);

    active_ = false;
    goal_.fill(0.0f);
    current_.fill(0.0f);
    curr_vel_.fill(0.0f);

    // TODO read from config file or ros param
    // max_vel_.fill(1.5f);
    max_vel_.fill(1.0f);
    des_vel_.fill(1.0f);
    min_vel_.fill(0.001f);
    // max_acc_.fill(1.5f);
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
    debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug", 1);

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
    JointValue intermediate = named_configurations_["elbow"];

    std::vector<JointValue> ctrl_pts({current_, goal_});
    // std::vector<JointValue> ctrl_pts({current_, intermediate, goal_});
    // std::vector<JointValue> ctrl_pts({current_, intermediate, intermediate, goal_});
    traj_ = Utils::calcSplineTrajectory(ctrl_pts, 0.01f);
    if ( traj_.size() == 0 )
    {
        std::cout << Utils::getMsgMod("warn")
                  << "[JointSpaceController] Could not plan trajectory."
                  << Utils::getMsgMod("end") << std::endl;
        reset();
    }

    /* print traj */
    for ( size_t i = 0; i < traj_.size(); i++ )
    {
        std::cout << i << " " << traj_[i] << std::endl;
    }

    pubDebugMsg();
    traj_start_time_ = std::chrono::steady_clock::now();
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

    if ( traj_.size() == 0 )
    {
        reset();
        return;
    }

    std::cout << "current: " << current_ << std::endl;
    std::cout << "curr_vel: " << curr_vel_ << std::endl;
    std::cout << "goal: " << goal_ << std::endl;

    float min_dist_sq = 1e6f;
    size_t min_dist_index = traj_index_;
    for ( size_t i = traj_index_; i < traj_.size(); i++ )
    {
        float dist_sq = Utils::calcDistSq(current_, traj_[i]);
        if ( dist_sq < min_dist_sq )
        {
            min_dist_sq = dist_sq;
            min_dist_index = i;
        }
    }
    std::cout << min_dist_sq << " " << min_dist_index << " " << traj_[min_dist_sq] << std::endl;
    if ( min_dist_index > 0 && min_dist_index+1 < traj_.size() )
    {
        size_t min_dist_index_prev = min_dist_index - 1;
        size_t min_dist_index_next = min_dist_index + 1;
        if ( Utils::calcDistSq(current_, traj_[min_dist_index-1]) < Utils::calcDistSq(current_, traj_[min_dist_index+1]) )
        {
            min_dist_index--;
        }
    }
    float t = ( min_dist_index+1 == traj_.size() )
              ? 0.0f
              : Utils::getProjectedPointRatioOnSegment(traj_[min_dist_index],
                                                       traj_[min_dist_index+1],
                                                       current_);
    traj_index_ = min_dist_index;

    size_t future_index_offset = 5;
    float p_gain = 0.1f;
    size_t future_traj_index = traj_index_ + future_index_offset;
    size_t future_traj_index_next = traj_index_ + future_index_offset + 1;
    if ( future_traj_index >= traj_.size() )
    {
        future_traj_index = traj_.size() - 1;
    }
    if ( future_traj_index_next >= traj_.size() )
    {
        future_traj_index_next = traj_.size() - 1;
    }

    JointValue target = Utils::interpolateLinearly(
            traj_[future_traj_index], traj_[future_traj_index_next], t);
    JointValue error = target - current_;

    if ( traj_index_+1 == traj_.size() )
    {
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
            std::cout << "error: " << error << std::endl;
            std::cout << Utils::getMsgMod("success")
                      << "[JointSpaceController] REACHED GOAL"
                      << Utils::getMsgMod("end") << std::endl;
            reset();
            return;
        }
    }

    JointValue curr_acc;
    curr_acc.fill(0.0f);
    JointValue raw_vel = error * p_gain * control_rate_;
    JointValue raw_vel_clipped = Utils::normalisedClip(curr_vel_, raw_vel, max_vel_);
    JointValue req_acc = (raw_vel_clipped - curr_vel_) * control_rate_;
    JointValue acc = Utils::normalisedClip(curr_acc, req_acc, max_acc_);
    JointValue vel = curr_vel_ + (acc * control_sample_time_);
    if ( open_loop_ )
    {
        curr_vel_ = vel;
    }
    std::cout << "target: " << target << std::endl;
    std::cout << "err: " << error << std::endl;
    std::cout << "raw_vel: " << raw_vel << std::endl;
    std::cout << "raw_vel_clipped: " << raw_vel_clipped << std::endl;
    std::cout << "req_acc: " << req_acc << std::endl;
    std::cout << "acc: " << acc << std::endl;
    std::cout << "vel: " << vel << std::endl;

    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
    std::cout << std::endl;

    /* fill debug msg */
    std_msgs::Float32MultiArray debug_msg;
    for ( size_t i = 0; i < current_.size(); i++ ) // 0 - 4
    {
        debug_msg.data.push_back(current_[i]);
    }
    for ( size_t i = 0; i < curr_vel_.size(); i++ ) // 5 - 9
    {
        debug_msg.data.push_back(curr_vel_[i]);
    }
    for ( size_t i = 0; i < target.size(); i++ ) // 10 - 14
    {
        debug_msg.data.push_back(target[i]);
    }
    for ( size_t i = 0; i < error.size(); i++ ) // 15 - 19
    {
        debug_msg.data.push_back(error[i]);
    }
    for ( size_t i = 0; i < raw_vel.size(); i++ ) // 20 - 24
    {
        debug_msg.data.push_back(raw_vel[i]);
    }
    for ( size_t i = 0; i < raw_vel_clipped.size(); i++ ) // 25 - 29
    {
        debug_msg.data.push_back(raw_vel_clipped[i]);
    }
    for ( size_t i = 0; i < req_acc.size(); i++ ) // 30 - 24
    {
        debug_msg.data.push_back(req_acc[i]);
    }
    for ( size_t i = 0; i < acc.size(); i++ ) // 35 - 39
    {
        debug_msg.data.push_back(acc[i]);
    }
    for ( size_t i = 0; i < vel.size(); i++ ) // 40 - 44
    {
        debug_msg.data.push_back(vel[i]);
    }
    JointValue ideal_vel;
    ideal_vel.fill(0.0f);
    for ( size_t i = 0; i < ideal_vel.size(); i++ ) // 45 - 49
    {
        debug_msg.data.push_back(ideal_vel[i]);
    }
    debug_pub_.publish(debug_msg);
}

// void JointSpaceController::run(const ros::TimerEvent& event)
// {
//     if ( !active_ )
//     {
//         return;
//     }

//     if ( traj_.size() == 0 )
//     {
//         reset();
//         return;
//     }

//     std::cout << "current: " << current_ << std::endl;
//     std::cout << "curr_vel: " << curr_vel_ << std::endl;
//     std::cout << "goal: " << goal_ << std::endl;

//     std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

//     std::chrono::duration<float> duration_from_traj_start = current_time - traj_start_time_;
//     float curr_time_from_traj_start = duration_from_traj_start.count();
//     std::cout << "curr_time: " << curr_time_from_traj_start << std::endl;
//     float future_time_from_traj_start = curr_time_from_traj_start + lookahead_time_;
//     std::cout << "future_time: " << future_time_from_traj_start << std::endl;
//     JointValue target = Utils::getJointValueAtTime(
//             traj_, future_time_from_traj_start, control_sample_time_);
//     JointValue error = target - current_;

//     if ( target == goal_ )
//     {
//         bool reached_goal = true;
//         for ( size_t i = 0; i < error.size(); i++ )
//         {
//             if ( fabs(error[i]) > goal_tolerance_ )
//             {
//                 reached_goal = false;
//                 break;
//             }
//         }

//         if ( reached_goal )
//         {
//             std::cout << "error: " << error << std::endl;
//             std::cout << Utils::getMsgMod("success")
//                       << "[JointSpaceController] REACHED GOAL"
//                       << Utils::getMsgMod("end") << std::endl;
//             reset();
//             return;
//         }
//     }

//     JointValue raw_vel = error * (1.0f/lookahead_time_);
//     // if ( traj_exec_complete )
//     // {
//     //     raw_vel = raw_vel * 0.1f;
//     // }
//     JointValue raw_vel_clipped = Utils::signedClip(raw_vel, max_vel_, min_vel_);
//     JointValue req_acc = (raw_vel_clipped - curr_vel_) * control_rate_;
//     JointValue acc = Utils::clip(req_acc, max_acc_, max_acc_*-1.0f);
//     JointValue vel = curr_vel_ + (acc * control_sample_time_);
//     if ( open_loop_ )
//     {
//         curr_vel_ = vel;
//     }
//     std::cout << "target: " << target << std::endl;
//     std::cout << "err: " << error << std::endl;
//     std::cout << "raw_vel: " << raw_vel << std::endl;
//     std::cout << "vel: " << vel << std::endl;

//     joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
//     std::cout << std::endl;

//     /* fill debug msg */
//     std_msgs::Float32MultiArray debug_msg;
//     for ( size_t i = 0; i < current_.size(); i++ ) // 0 - 4
//     {
//         debug_msg.data.push_back(current_[i]);
//     }
//     for ( size_t i = 0; i < curr_vel_.size(); i++ ) // 5 - 9
//     {
//         debug_msg.data.push_back(curr_vel_[i]);
//     }
//     for ( size_t i = 0; i < target.size(); i++ ) // 10 - 14
//     {
//         debug_msg.data.push_back(target[i]);
//     }
//     for ( size_t i = 0; i < error.size(); i++ ) // 15 - 19
//     {
//         debug_msg.data.push_back(error[i]);
//     }
//     for ( size_t i = 0; i < raw_vel.size(); i++ ) // 20 - 24
//     {
//         debug_msg.data.push_back(raw_vel[i]);
//     }
//     for ( size_t i = 0; i < raw_vel_clipped.size(); i++ ) // 25 - 29
//     {
//         debug_msg.data.push_back(raw_vel_clipped[i]);
//     }
//     for ( size_t i = 0; i < req_acc.size(); i++ ) // 30 - 24
//     {
//         debug_msg.data.push_back(req_acc[i]);
//     }
//     for ( size_t i = 0; i < acc.size(); i++ ) // 35 - 39
//     {
//         debug_msg.data.push_back(acc[i]);
//     }
//     for ( size_t i = 0; i < vel.size(); i++ ) // 40 - 44
//     {
//         debug_msg.data.push_back(vel[i]);
//     }
//     JointValue ideal_curr_traj_point = Utils::getJointValueAtTime(
//             traj_, curr_time_from_traj_start, control_sample_time_);
//     JointValue ideal_next_traj_point = Utils::getJointValueAtTime(
//             traj_, curr_time_from_traj_start+control_sample_time_, control_sample_time_);
//     JointValue ideal_vel = (ideal_next_traj_point - ideal_curr_traj_point) * control_rate_;
//     for ( size_t i = 0; i < ideal_vel.size(); i++ ) // 45 - 49
//     {
//         debug_msg.data.push_back(ideal_vel[i]);
//     }
//     debug_pub_.publish(debug_msg);
// }

void JointSpaceController::reset()
{
    active_ = false;
    goal_.fill(0.0f);
    pubZeroVel();
    traj_index_ = 0;
    traj_.clear();

    pubDebugMsg();
}

void JointSpaceController::pubDebugMsg()
{
    /* fill debug msg */
    std_msgs::Float32MultiArray debug_msg;
    for ( size_t i = 0; i < current_.size(); i++ ) // 0 - 4
    {
        debug_msg.data.push_back(current_[i]);
    }
    for ( size_t i = 0; i < 9*current_.size(); i++ ) // 5 - 49
    {
        debug_msg.data.push_back(0.0f);
    }
    debug_pub_.publish(debug_msg);
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
