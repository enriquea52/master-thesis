#include "backend/EKF.hpp"
#include "utilities/vis.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "april_detect_msgs/msg/detections.hpp"
#include "april_detect_msgs/msg/tag.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "ekf_interfaces/srv/set_pose.hpp"
#include "ekf_interfaces/srv/save_map.hpp"
#include "ekf_interfaces/srv/load_map.hpp"

#include<iostream>
#include <sys/stat.h>
#include <chrono>
using namespace std::chrono;

class SLAM_NODE : public rclcpp::Node
{
    private:
    visualization vis_tool;
    std::string node_name;
    double prev_t, current_t;
    double dt;
    bool init = true;
    double u[2];
    GENERICS::Vec3 y;
    geometry_msgs::msg::TransformStamped t;

    std::string path_maps;

    int counter_delete = 0;

    MOTION_MODEL *ROBOT_MODEL;
    LANDMARK_MODEL *LANDMARK_3D_SENSOR;
    EKF::SLAM *SLAM_BACKEND;

    // Subscribers for Prediction and Update
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<april_detect_msgs::msg::Detections>::SharedPtr tag_sub;

    // tf2 Listener Pointers
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Service<ekf_interfaces::srv::SetPose>::SharedPtr setPoseService;
    rclcpp::Service<ekf_interfaces::srv::SetPose>::SharedPtr resetService;
    rclcpp::Service<ekf_interfaces::srv::SaveMap>::SharedPtr saveService;
    rclcpp::Service<ekf_interfaces::srv::LoadMap>::SharedPtr loadService;

    void predict(const nav_msgs::msg::Odometry & msg) 
    {
        if (init)
        { 
            prev_t = (double)msg.header.stamp.sec + (msg.header.stamp.nanosec/1.0e9); 
            init = false; 
            return;
        }

        current_t = (double)msg.header.stamp.sec + (msg.header.stamp.nanosec/1.0e9);

        dt = (current_t - prev_t);

        // Grab inputs
        u[0] = msg.twist.twist.linear.x; /* Linear Velocity */
        u[1] = msg.twist.twist.angular.z; /* Angular Velocity */

        // Store Current Time later dt Calculation
        prev_t = current_t;

        /* Discard Problems with dt */
        if (dt == 0 || dt > 1){return;}

        // Apply Prediction
        SLAM_BACKEND->PREDICT(u, dt);

        // Visualize Robot Pose Covariance
        vis_tool.robot_pose_estimate_vis(SLAM_BACKEND->Xr(), SLAM_BACKEND->Prr().block(0, 0, 2, 2));
    }

    void update(const april_detect_msgs::msg::Detections & msg) 
    {

        for (april_detect_msgs::msg::Tag aprilTag : msg.detections)
        {
            aprilTag.id++;

            /*Get Measurment*/
            y << aprilTag.x,
                 aprilTag.y,
                 aprilTag.z;

            /*If not Landmark in state vector add the First One*/
            if (SLAM_BACKEND->LM_NUM < 1)
            {
                SLAM_BACKEND->ADD_STATE(y, aprilTag.id);
            }
            else
            {
                // Check if the Landmark ID exists
                if (SLAM_BACKEND->landmark.count(aprilTag.id))
                {
                    // update is the landmark already exists
                    SLAM_BACKEND->UPDATE(y, aprilTag.id);
                }
                else
                {
                    // add new landmark if not known already
                    SLAM_BACKEND->ADD_STATE(y, aprilTag.id);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Landmarks [%d]", SLAM_BACKEND->LM_NUM);

        vis_tool.publish_landmarks(SLAM_BACKEND->l(), SLAM_BACKEND->Pl(), SLAM_BACKEND->LM_NUM);

    }

    void set_pose(const std::shared_ptr<ekf_interfaces::srv::SetPose::Request> request,
                        std::shared_ptr<ekf_interfaces::srv::SetPose::Response>response)
    {
        SLAM_BACKEND->setPose(request->x, request->y, request->theta);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting Pose to \nx: %ld" " y: %ld" " theta: %ld",
                        request->x, request->y, request->theta);
        response->success=true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (bool)response->success);
    }

    void reset(const std::shared_ptr<ekf_interfaces::srv::SetPose::Request> request,
                     std::shared_ptr<ekf_interfaces::srv::SetPose::Response>response)
    {
        SLAM_BACKEND->reset(request->x, request->y, request->theta);

        if (request->x + request->y + request->theta != 0.0)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ressetig and Setting Pose to \nx: %ld" " y: %ld" " theta: %ld", request->x, request->y, request->theta);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resseting EKF System \n");
        }

        response->success=true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (bool)response->success);
    } 

    void saveMap(const std::shared_ptr<ekf_interfaces::srv::SaveMap::Request> request,
                       std::shared_ptr<ekf_interfaces::srv::SaveMap::Response> response)
    {
        std::string map_name = request->map_name;
        std::string path = path_maps;

        mkdir( (path + map_name).c_str() , 0777);

        SLAM_BACKEND->SAVE_STATE(path + map_name + "/landmarks.csv");
        SLAM_BACKEND->SAVE_COV(path + map_name + "/cov.csv");
        SLAM_BACKEND->SAVE_IDS(path + map_name + "/ids.csv");

        response->success=true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SAVE sending back response: [%ld]", (bool)response->success);
    } 


    void loadMap(const std::shared_ptr<ekf_interfaces::srv::LoadMap::Request> request,
                       std::shared_ptr<ekf_interfaces::srv::LoadMap::Response> response)
    {
        std::string map_name = request->map_name;
        std::string path = path_maps;
        
        std::string state = path + map_name + "/landmarks.csv";
        std::string cov = path + map_name + "/cov.csv";
        std::string ids = path + map_name + "/ids.csv";
        
        SLAM_BACKEND->LOAD_STATE(state);
        SLAM_BACKEND->LOAD_COV(cov, 10.0);
        SLAM_BACKEND->LOAD_IDS(ids);

        response->success=true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LOAD sending back response: [%ld]", (bool)response->success);
    } 

    public:
        SLAM_NODE(std::string node_name)
        : Node(node_name)
        {
        RCLCPP_INFO(this->get_logger(), "Initializing : [%s] Node", node_name.c_str());

        /* Read Parameters */ 
        // Motion Uncertainty
        this->declare_parameter<double>("motion_covariance.std_v", 0.0);
        this->declare_parameter<double>("motion_covariance.std_w", 0.0);
        // Kinematic Params
        this->declare_parameter<double>("kinematic_params.a", 0.0);
        this->declare_parameter<double>("kinematic_params.l", 0.0);
        this->declare_parameter<double>("kinematic_params.r", 0.0);
        // Observation Uncertainty
        this->declare_parameter<double>("tag_covariance.std_x", 0.0);
        this->declare_parameter<double>("tag_covariance.std_y", 0.0);
        this->declare_parameter<double>("tag_covariance.std_z", 0.0);
        // Camera Pose with Respect the Base_link Frame
        this->declare_parameter<double>("camera_pose_wrt_base.x", 1.284);
        this->declare_parameter<double>("camera_pose_wrt_base.y", 0.0);
        this->declare_parameter<double>("camera_pose_wrt_base.z", 1.8);
        // Define Topics to Subscribe
        this->declare_parameter<std::string>("odometry_topic", "/odom");
        this->declare_parameter<std::string>("tag_topic", "/detections");
        // Define where the maps are saved
        this->declare_parameter<std::string>("maps_path", "/home");

        /*SLAM Models' Definitions*/
        ROBOT_MODEL = new MOTION_MODEL(get_parameter("kinematic_params.r").as_double(),
                                       get_parameter("kinematic_params.l").as_double(), 
                                       get_parameter("kinematic_params.a").as_double(), 
                                       get_parameter("motion_covariance.std_v").as_double(),
                                       get_parameter("motion_covariance.std_w").as_double());

        LANDMARK_3D_SENSOR = new LANDMARK_MODEL(get_parameter("tag_covariance.std_x").as_double(),
                                                get_parameter("tag_covariance.std_y").as_double(),
                                                get_parameter("tag_covariance.std_z").as_double());


        path_maps = get_parameter("maps_path").as_string();


        std::cout << "LANDMARK_3D_SENSOR: \n" << LANDMARK_3D_SENSOR->R() << std::endl;
        std::cout << "ROBOT_MODEL: \n" << ROBOT_MODEL->N() << std::endl;

        /* Camera's Pose wrt base_link*/
        double xc = get_parameter("camera_pose_wrt_base.x").as_double();
        double yc = get_parameter("camera_pose_wrt_base.y").as_double();
        double zc = get_parameter("camera_pose_wrt_base.z").as_double();

        GENERICS::MatX rTc(3, 4);

        rTc <<  0.0,  0.0,  1.0, xc,
                1.0,  0.0,  0.0, yc,
                0.0,  1.0,  0.0, zc;

        LANDMARK_3D_SENSOR->set_cam_to_bot(rTc);

        /* Initiialize Back-End*/
        SLAM_BACKEND = new EKF::SLAM(3, 3, 20); /*State Vector of Size 4, Landmark Vector of Size 3, Handle a Maximum of 20 Artifical Landmarks and 5000 Natural*/

        SLAM_BACKEND->setMotionModel(ROBOT_MODEL);
        SLAM_BACKEND->setSensorModel(LANDMARK_3D_SENSOR);

        /*Definition of Subscribers and Publishers*/
        /* Joint State Subscriber */ 
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(get_parameter("odometry_topic").as_string(), 10, std::bind(&SLAM_NODE::predict, this, std::placeholders::_1));
        
        /* Apriltag detector Subscriber */
        tag_sub = this->create_subscription<april_detect_msgs::msg::Detections>(get_parameter("tag_topic").as_string(), 1, std::bind(&SLAM_NODE::update, this, std::placeholders::_1));
    
        /* Create Service Set Pose */
        setPoseService = this->create_service<ekf_interfaces::srv::SetPose>("ekf/set_pose", std::bind(&SLAM_NODE::set_pose, this, std::placeholders::_1, std::placeholders::_2));
        
        /* Create Service Reset EKF */
        resetService = this->create_service<ekf_interfaces::srv::SetPose>("ekf/reset", std::bind(&SLAM_NODE::reset, this, std::placeholders::_1, std::placeholders::_2));

        /* Create Service Save Map */
        saveService = this->create_service<ekf_interfaces::srv::SaveMap>("ekf/save", std::bind(&SLAM_NODE::saveMap, this, std::placeholders::_1, std::placeholders::_2));

        /* Create Service Load Map */
        loadService = this->create_service<ekf_interfaces::srv::LoadMap>("ekf/load", std::bind(&SLAM_NODE::loadMap, this, std::placeholders::_1, std::placeholders::_2));

        /* Definition of TF Listener */
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }

        ~SLAM_NODE()
        {
            delete SLAM_BACKEND;
            delete ROBOT_MODEL;
            delete LANDMARK_3D_SENSOR;
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAM_NODE>("ekf_slam_node"));
    rclcpp::shutdown();
    return 0;
}