#ifndef JOINT_SPACE_CONTROLLER_H
#define JOINT_SPACE_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

#include <mir_joint_space_controller/joint_value.h>

class JointSpaceController
{
    public:
        JointSpaceController();
        virtual ~JointSpaceController();

        void run(const ros::TimerEvent& event);

    protected:
        ros::NodeHandle nh_;

        ros::Publisher joint_vel_pub_;

        ros::Subscriber joint_states_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber cancel_sub_;
        ros::Subscriber joy_sub_;

        float control_rate_;
        float control_sample_time_;

        bool active_;
        JointValue goal_;
        JointValue current_;
        JointValue curr_vel_;
        JointValue max_vel_;
        JointValue min_vel_;
        JointValue max_acc_;
        float goal_tolerance_;

        /* transform related variables */
        std::shared_ptr<tf::TransformListener> tf_listener_;

        bool debug_;

        void goalCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void cancelCb(const std_msgs::Empty::ConstPtr& msg);
        void joyCb(const sensor_msgs::Joy::ConstPtr& msg);
        void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);

        void reset();
        void pubZeroVel();

};

#endif // JOINT_SPACE_CONTROLLER_H
