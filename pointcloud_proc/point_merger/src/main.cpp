#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

class POINT_MERGER: public rclcpp::Node
{
        
    public:

		void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr front_scan_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr back_scan_msg)
        {

            try {
                
                tf_buffer_->lookupTransform(
                    base_frame, front_scan_msg->header.frame_id, this->get_clock()->now());

                projector_1.transformLaserScanToPointCloud(base_frame, *front_scan_msg, *front_cloud_msg_ptr, *tf_buffer_, 10);

                tf_buffer_->lookupTransform(
                    base_frame, back_scan_msg->header.frame_id, this->get_clock()->now());

                projector_2.transformLaserScanToPointCloud(base_frame, *back_scan_msg, *back_cloud_msg_ptr, *tf_buffer_, 10);

            } 
            catch (tf2::TransformException &ex) 
            {
            RCLCPP_WARN(this->get_logger(), "could not transform scan to map frame: %s", ex.what());
            return;
            }

            // Convert pointclouds msgs to pcl format
            pcl::fromROSMsg(*front_cloud_msg_ptr, *front_cloud);
            pcl::fromROSMsg(*back_cloud_msg_ptr, *back_cloud);

            // Concatenate both pointclouds
            joint_cloud = front_cloud;
            *joint_cloud += *back_cloud;

            // Convert to ROS  pointcloud MSG
            pcl::toROSMsg(*joint_cloud, *joint_cloud_msg_ptr);
            joint_cloud->header.frame_id = base_frame;

            // Publish Concatenated Pointcloud
            joint_cloud_pub_->publish(*joint_cloud_msg_ptr);

            // Save the present cloud for further registration
            *joint_cloud_past = *joint_cloud;

        }

        // Constructor
        // subscribers

        POINT_MERGER(std::string node_name):Node(node_name)
        {
        
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
            scan_sub_1_.subscribe(this, "itav_agv/safety_lidar_front_link/scan", rmw_qos_profile_sensor_data);
            scan_sub_2_.subscribe(this, "itav_agv/safety_lidar_back_link/scan", rmw_qos_profile_sensor_data);
            sync_.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), scan_sub_1_, scan_sub_2_));
            sync_->registerCallback(std::bind(&POINT_MERGER::scanCallback, this, std::placeholders::_1, std::placeholders::_2));

            joint_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("joint_points", sensor_qos);

            front_cloud_msg_ptr = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2());
            back_cloud_msg_ptr = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2());
            joint_cloud_msg_ptr = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2());

            temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            joint_cloud_past = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

            joint_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            front_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            back_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        }
        ~POINT_MERGER(){}

	private:
		message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_1_;
      	message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_2_;

		using approximate_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
		std::unique_ptr<message_filters::Synchronizer<approximate_policy>> sync_;
		laser_geometry::LaserProjection projector_1, projector_2;

        sensor_msgs::msg::PointCloud2::SharedPtr front_cloud_msg_ptr;
        sensor_msgs::msg::PointCloud2::SharedPtr back_cloud_msg_ptr;
        sensor_msgs::msg::PointCloud2::SharedPtr joint_cloud_msg_ptr;

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud_past;
        pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr back_cloud;

        std::string base_frame = "base_link";

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr joint_cloud_pub_;

        bool init_ = true;

        Eigen::Matrix4d T_estimation = Eigen::Matrix4d::Identity();

};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<POINT_MERGER>("pm_node"));
    rclcpp::shutdown();

    return 0;
}
    