
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <utility>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/utilities.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mas_perception_msgs/msg/object_list.hpp"
#include "mas_perception_msgs/msg/bounding_box_list.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "mir_perception_utils/clustered_point_cloud_visualizer.hpp"
#include "mir_perception_utils/bounding_box_visualizer.hpp"
#include "mir_perception_utils/label_visualizer.hpp"
#include "mir_perception_utils/object_utils_ros.hpp"
#include "mir_perception_utils/pointcloud_utils_ros.hpp"
#include "mir_object_segmentation/scene_segmentation_ros.hpp"
#include "mir_perception_utils/bounding_box.hpp"

// just for testing, remove later
#include "mir_perception_utils/planar_polygon_visualizer.hpp"



using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace mpu = mir_perception_utils;
using mpu::visualization::BoundingBoxVisualizer;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::LabelVisualizer;
using mpu::visualization::Color;


struct Object
{
  std::string name;
  std::string shape;
  std::string color;
};
typedef std::vector<Object> ObjectInfo;

class MultiModalObjectRecognitionROS: public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit MultiModalObjectRecognitionROS(const std::string & node_name, bool intra_process_comms);

        /// Transition callback for state configuring
        /**
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        void declare_all_parameters();
        void get_all_parameters();

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        /// Transition callback for state activating
        /**
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "active"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state);

        /// Transition callback for state deactivating
        /**
         * on_deactivate callback is being called when the lifecycle node
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "active".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & state);

         /// Transition callback for state cleaningup
        /**
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "unconfigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        /// Transition callback for state shutting down
        /**
         * on_shutdown callback is being called when the lifecycle node
         * enters the "shuttingdown" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "finalized" state or stays
         * in its current state.
         * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
         * TRANSITION_CALLBACK_FAILURE transitions to current state
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state);

    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mas_perception_msgs::msg::ObjectList>> obj_list_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> pub_pc_object_pose_array_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> pub_rgb_object_pose_array_;
        
        message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> image_sub_;
        
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode> cloud_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                sensor_msgs::msg::PointCloud2> msgSyncPolicy;
        typedef message_filters::Synchronizer<msgSyncPolicy> Sync;
        std::shared_ptr<Sync> msg_sync_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> publisher_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

        // publisher object list
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> pub_workspace_height_;

        // publisher debug
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_plane_;


        // --------------------------- function declarations -----------------------------------
        
        void synchronizeCallback(const std::shared_ptr<sensor_msgs::msg::Image> &image, 
                const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud);

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        
        // void preprocessPointCloud(const sensor_msgs::msg::PointCloud2 &cloud_msg);

        /** \brief Transform pointcloud to the given frame id ("base_link" by default)
         * \param[in] PointCloud2 input
        */
        void preprocessPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud_msg);

        /** \brief Add cloud accumulation, segment accumulated pointcloud, find the plane, 
         *     clusters table top objects, find object heights.
         * \param[out] 3D object list with unknown label
         * \param[out] Table top pointcloud clusters
         **/
        void segmentPointCloud(mas_perception_msgs::msg::ObjectList &object_list,
                        std::vector<PointCloudBSPtr> &clusters,
                        std::vector<mpu::object::BoundingBox> boxes);
        
        /** \brief Recognize 2D and 3D objects, estimate their pose, filter them, and publish the object_list*/
        void recognizeCloudAndImage();
        
        void publishDebug(mas_perception_msgs::msg::ObjectList &combined_object_list,
                                                std::vector<PointCloudBSPtr> &clusters_3d,
                                                std::vector<PointCloudBSPtr> &clusters_2d);

        typedef std::shared_ptr<SceneSegmentationROS> SceneSegmentationROSSPtr;
        SceneSegmentationROSSPtr scene_segmentation_ros_;
        mas_perception_msgs::msg::ObjectList recognized_cloud_list_; 
        mas_perception_msgs::msg::ObjectList recognized_image_list_;

    // protected:
        //visualization
        BoundingBoxVisualizer bounding_box_visualizer_pc_;
        ClusteredPointCloudVisualizer cluster_visualizer_rgb_;
        ClusteredPointCloudVisualizer cluster_visualizer_pc_;
        LabelVisualizer label_visualizer_rgb_;
        LabelVisualizer label_visualizer_pc_;

        //parameters
        bool debug_mode_;
        std::string pointcloud_source_frame_id_;
        std::string target_frame_id_;
        // std::set<std::string> round_objects_;
        // typedef std::vector<Object> ObjectInfo;
        // ObjectInfo object_info_;
        // std::string object_info_path_;

        // Used to store pointcloud and image received from callback
        std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud_msg_;
        std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
        PointCloudBSPtr cloud_;
        

        // Dynamic parameter
        double voxel_leaf_size_;
        std::string voxel_filter_field_name_;
        double voxel_filter_limit_min_;
        double voxel_filter_limit_max_;
        bool enable_passthrough_filter_;
        std::string passthrough_filter_field_name_;
        double passthrough_filter_limit_min_;
        double passthrough_filter_limit_max_;
        std::string passthrough_filter_field_y_;
        double passthrough_filter_y_limit_min_;
        double passthrough_filter_y_limit_max_;
        double normal_radius_search_;
        bool use_omp_;
        int num_cores_;
        int sac_max_iterations_;
        double sac_distance_threshold_;
        bool sac_optimize_coefficients_;
        double sac_x_axis_;
        double sac_y_axis_;
        double sac_z_axis_;
        double sac_eps_angle_;
        double sac_normal_distance_weight_;
        double prism_min_height_;
        double prism_max_height_;
        double outlier_radius_search_;
        int outlier_min_neighbors_;
        double cluster_tolerance_;
        int cluster_min_size_;
        int cluster_max_size_;
        double cluster_min_height_;
        double cluster_max_height_;
        double cluster_max_length_;
        double cluster_min_distance_to_polygon_;

        //cluster
        bool center_cluster_;
        bool pad_cluster_;
        int padded_cluster_size_;

        double octree_resolution_;
        double object_height_above_workspace_;
        double container_height_;
        bool enable_rgb_recognizer_;
        bool enable_pc_recognizer_;
        int rgb_roi_adjustment_;
        int rgb_bbox_min_diag_;
        int rgb_bbox_max_diag_;
        double rgb_cluster_filter_limit_min_;
        double rgb_cluster_filter_limit_max_;
        bool rgb_cluster_remove_outliers_;
        bool enable_roi_;
        double roi_base_link_to_laser_distance_;
        double roi_max_object_pose_x_to_base_link_;
        double roi_min_bbox_z_;

};

#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H