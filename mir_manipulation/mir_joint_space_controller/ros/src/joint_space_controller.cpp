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

    // TODO read from config file or ros param
    for ( size_t i = 0; i < current_.size(); i++ )
    {
        joint_names_.push_back("arm_joint_" + std::to_string(i+1));
    }

    /* read named configurations */
    readNamedConfig();

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
        min_req_times[i] = calcMinimumRequiredTime(current_[i], goal_[i], max_vel_[i], max_acc_[i]);
        if ( min_req_times[i] > max_req_time )
        {
            max_req_time = min_req_times[i];
            slowest_joint = i;
        }
    }
    std::cout << max_req_time << " " << slowest_joint << std::endl;

    // FIXME
    // active_ = true;
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

    JointValue error, vel;
    vel.fill(0.0f);
    for ( size_t i = 0; i < error.size(); i++ )
    {
        error[i] = goal_[i] - current_[i];
        if ( fabs(error[i]) < goal_tolerance_ )
        {
            error[i] = 0.0f;
            continue;
        }
        float raw_vel = error[i];
        float raw_vel_clipped = Utils::signedClip(raw_vel, max_vel_[i], min_vel_[i]);
        float req_acc = (raw_vel_clipped - curr_vel_[i]) * control_rate_;
        float acc = Utils::clip(req_acc, max_acc_[i], -max_acc_[i]);
        vel[i] = curr_vel_[i] + (acc * control_sample_time_);
        if ( open_loop_ )
        {
            curr_vel_[i] = vel[i];
        }
    }
    std::cout << "err: " << error << std::endl;
    std::cout << "vel: " << vel << std::endl;

    bool reached_goal = true;
    for ( size_t i = 0; i < error.size(); i++ )
    {
        if ( error[i] != 0.0f )
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
    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel, joint_names_));
    std::cout << std::endl;
}

float JointSpaceController::calcMinimumRequiredTime(float curr, float goal,
                                                    float max_vel, float max_acc)
{
    // std::cout << "inside JointSpaceController::calcMinimumRequiredTime" << std::endl;
    // std::cout << "curr: " << curr << std::endl;
    // std::cout << "goal: " << goal << std::endl;
    // std::cout << "max_vel: " << max_vel << std::endl;
    // std::cout << "max_acc: " << max_acc << std::endl;

    float D = fabs(goal - curr);
    // std::cout << "D: " << D << std::endl;
    if ( D < (max_vel*max_vel)/max_acc )
    {
        // std::cout << "acc and dec" << std::endl;
        float desired_vel = sqrt((max_acc * D)/2);
        float min_req_time = 2 * (desired_vel/max_acc);
        // std::cout << "desired_vel: " << desired_vel << std::endl;
        // std::cout << "min_req_time: " << min_req_time << std::endl;
        return min_req_time;
    }
    else
    {
        // std::cout << "acc, const and dec" << std::endl;
        float acc_time = max_vel/max_acc;
        float const_vel_time = D - ((max_vel*max_vel)/max_acc);
        float min_req_time = (2 * acc_time) + const_vel_time;
        // std::cout << "acc_time: " << acc_time << std::endl;
        // std::cout << "const_vel_time: " << const_vel_time << std::endl;
        // std::cout << "min_req_time: " << min_req_time << std::endl;
        return min_req_time;
    }
}

void JointSpaceController::reset()
{
    active_ = false;
    goal_.fill(0.0f);
    pubZeroVel();
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
