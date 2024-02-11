#ifndef COMPONENT_WISE_POSE_ERROR_CALCULATOR_HPP
#define COMPONENT_WISE_POSE_ERROR_CALCULATOR_HPP
#include "tf2_ros/buffer.h" 
#include "geometry_msgs/msg/pose_stamped.hpp"
struct ComponentWiseCartesianDifference
{
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
};

bool get_component_wise_pose_error(const geometry_msgs::msg::PoseStamped::SharedPtr origin_pose, 
                                   const geometry_msgs::msg::PoseStamped::SharedPtr target_pose,
                                   ComponentWiseCartesianDifference &error,
                                   std::unique_ptr<tf2_ros::Buffer> &tf_buffer_);

bool transform_pose(const geometry_msgs::msg::PoseStamped::SharedPtr reference_pose,
                    const geometry_msgs::msg::PoseStamped::SharedPtr target_pose, 
                    std::unique_ptr<tf2_ros::Buffer> &tf_buffer_,
                    geometry_msgs::msg::PoseStamped & pose_out_);

float get_shortest_angle_difference(float angle_1, float angle_2);

#endif 