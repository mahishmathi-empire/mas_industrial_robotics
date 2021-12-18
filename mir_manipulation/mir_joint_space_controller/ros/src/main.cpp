#include <mir_joint_space_controller/joint_space_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_space_controller");
    ros::NodeHandle nh("~");

    /* ros params */
    float control_rate;
    nh.param<float>("control_rate", control_rate, 10.0f);

    JointSpaceController controller;

    ros::Timer run_timer = nh.createTimer(
            ros::Duration(1.0f/control_rate),
            &JointSpaceController::run,
            &controller);

    ros::spin();

    return 0;
}
