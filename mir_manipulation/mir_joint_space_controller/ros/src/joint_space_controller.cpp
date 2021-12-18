#include <mir_joint_space_controller/joint_space_controller.h>
#include <mir_joint_space_controller/utils.h>

JointSpaceController::JointSpaceController():
    nh_("~")
{
    /* ros params */
    nh_.param<bool>("debug", debug_, true);
    nh_.param<float>("control_rate", control_rate_, 10.0f);
    control_sample_time_ = 1.0f/control_rate_;
    nh_.param<float>("goal_tolerance", goal_tolerance_, 0.01f);

    active_ = false;
    goal_.fill(0.0f);
    current_.fill(0.0f);
    curr_vel_.fill(0.0f);

    // TODO read from config file or ros param
    max_vel_.fill(0.5f);
    min_vel_.fill(0.001f);
    max_acc_.fill(1.0f);

    /* publishers */
    joint_vel_pub_ = nh_.advertise<brics_actuator::JointVelocities>("joint_vel", 1);

    /* subscribers */
    goal_sub_ = nh_.subscribe("goal", 1, &JointSpaceController::goalCb, this);
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
    if ( msg->name.size() == current_.size() && msg->name[0] == "arm_joint_1" )
    {
        for ( size_t i = 0; i < current_.size(); i++ )
        {
            current_[i] = msg->position[i];
        }
        for ( size_t i = 0; i < curr_vel_.size(); i++ )
        {
            curr_vel_[i] = msg->velocity[i];
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

    std::cout << current_ << std::endl;
    std::cout << curr_vel_ << std::endl;
    std::cout << goal_ << std::endl;

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
        std::cout << "REACHED GOAL" << std::endl;
        reset();
        return;
    }
    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel));
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
    joint_vel_pub_.publish(Utils::getJointVelocitiesFromJointValue(vel));
}
