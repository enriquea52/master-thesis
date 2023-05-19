#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/recognition/ransac_based/trimmed_icp.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

class ICP_ODOM: public rclcpp::Node
{
        
    public:

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {    
            std::cout << "receiving PC" << std::endl;

            if (init_)
            {
                pcl::fromROSMsg(*msg, *past_cloud);
                init_ = false;
                prev_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9);
                return;
            }

            current_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9);

            double dt = (current_t_ - prev_t_);

            if(dt < 0.1)
            {
                return;
            }

            std::cout << "DT: " << dt << std::endl;
            prev_t_ = current_t_;
    

            // Convert pointclouds msgs to pcl format

            // Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
            // tricp.init(front_cloud);
            // tricp.align(*back_cloud, 0.6*back_cloud->size(), transformation);

            // // icp.setInputSource(back_cloud);
            // // icp.setInputTarget(front_cloud);
            // // icp.align (*temp_cloud);

            // // while(!icp.hasConverged()){}

            // // Eigen::Matrix4f transformation = icp.getFinalTransformation();

            // std::cout << "translation: \n" << transformation.block<3, 1>(0, 3) <<std::endl;
            // std::cout << "orientation: \n" <<  transformation.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;


            // if (init_)
            // {
            //     *joint_cloud_past = *joint_cloud;
            //     init_ = false;
            // }

            // Convert to ROS  pointcloud MSG
        }

        //Constructor
        // subscribers

        ICP_ODOM(std::string node_name):Node(node_name)
        {

            // auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
            // qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

            point_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("joint_points", qos, std::bind(&ICP_ODOM::points_callback, this, std::placeholders::_1));
    
            past_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            current_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

            icp.setMaximumIterations(100);

        }
        ~ICP_ODOM(){}

	private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub;

        pcl::PointCloud<pcl::PointXYZ>::Ptr past_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        Eigen::Matrix4d T_estimation = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
        bool init_ = true;
        double current_t_, prev_t_;


};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_ODOM>("icp_odom_node"));
    rclcpp::shutdown();

    return 0;
}
    