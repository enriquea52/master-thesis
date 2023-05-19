#include"backend/EKF.hpp"
#include"utilities/vis.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "april_detect_msgs/msg/detections.hpp"
#include "april_detect_msgs/msg/tag.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include<iostream>

#include<rcpputils/rcppmath/rolling_mean_accumulator.hpp>

#include <chrono>
using namespace std::chrono;

class FUSION_NODE : public rclcpp::Node
{
    private:

    rcppmath::RollingMeanAccumulator<double> acum_imu;
    rcppmath::RollingMeanAccumulator<double> acum_icp_v;
    rcppmath::RollingMeanAccumulator<double> acum_icp_w;

    // Full_bag3
    double x_map;
    double y_map;
    double theta_map;

    std::string node_name;
    bool init = true;
    double prev_t, current_t;

    bool icp_init_ = true;
    double icp_prev_t_, icp_current_t_;
    Mat2 R_icp = Mat2::Zero();

    bool imu_init_ = true;
    double imu_prev_t_, imu_current_t_;
    double R_imu = 0;

    double dt;

    bool do_predict = true;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub;

    int counter_delete = 0;

    Vec2 X = Vec2::Zero();
    Mat2 P = 10*Mat2::Identity();

    Mat2 Q = Mat2::Zero();
    Mat2 R_enc = Mat2::Zero();

    double R, L, a;

    // Subscribers for Prediction and Update
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr V_ACCUM_PUB;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr W_ACCUM_PUB;

    void prediction()
    {
        P = P + Q;

        publish_fused_odom(X(0), X(1));
    }

    void encoder_update(const sensor_msgs::msg::JointState & msg) 
    {

        // Grab inputs
        double alpha = msg.position[0]; /* Steering Angle Position (rad) */
        double w = msg.velocity[1];     /* Traction Wheel Angular Speed (rad/s) */
        
        double c_alpha = std::cos(alpha), 
               s_alpha = std::sin(alpha);

        // Apply Update

        double vk = (R*w)*(c_alpha - (a*s_alpha/L));

        double wk = (R*w)*(s_alpha/L);

        Vec2 Y;
        Y(0) = vk;
        Y(1) = wk;

        Mat2 K = P*(P + R_enc).inverse();

        X = X + K*(Y-X);

        P = (Mat2::Identity() - K)*P;

        do_predict = true;
    }

    void icp_update(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        // Time Initialization
        if (icp_init_)
        { 
            icp_prev_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9); 
            icp_init_ = false; 
            return;
        }

        icp_current_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9); 

        double dt = (icp_current_t_ - icp_prev_t_);

        acum_icp_v.accumulate(msg->twist.twist.linear.x);

        acum_icp_w.accumulate(msg->twist.twist.angular.z);

        if(dt < 0.1)
        {
            return;
        }
        else
        {

            Vec2 Y;
            Y(0) = acum_icp_v.getRollingMean();
            Y(1) = acum_icp_w.getRollingMean();

            Mat2 K = P*(P + R_icp).inverse();
            X = X + K*(Y-X);
            P = (Mat2::Identity() - K)*P;
        }
    }

    void imu_update(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        // Time Initialization
        if (imu_init_)
        { 
            imu_prev_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9); 
            imu_init_ = false; 
            return;
        }

        acum_imu.accumulate(msg->angular_velocity.z);

        // Kalman Filter Update

        imu_current_t_ = (double)msg->header.stamp.sec + (msg->header.stamp.nanosec/1.0e9);

        double dt = (imu_current_t_ - imu_prev_t_);

        if (dt < 0.05)
        {
            return;
        }
        else
        {
            double Y;

            Y = acum_imu.getRollingMean();

            double K = P(1, 1)/(P(1, 1) + R_imu);

            X(1) = X(1) + K*(Y-X(1));

            P(1, 1) = (1 - K)*P(1, 1);

            imu_prev_t_ = imu_current_t_;
        }

    }
    
    public:
        FUSION_NODE(std::string node_name)
        : Node(node_name), acum_imu(100), acum_icp_v(20), acum_icp_w(20)
        {

        RCLCPP_INFO(this->get_logger(), "Initializing : [%s] Node", node_name.c_str());

        /* Topics Involved */
        this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
        this->declare_parameter<std::string>("icp_topic", "/kiss_odom");
        this->declare_parameter<std::string>("imu_topic", "/zed2i/zed_node/imu/data");
        this->declare_parameter<std::string>("output_fusion_topic", "/fused_odom");

        /* Kinematic Parameters */
        this->declare_parameter<double>("kinematic_params.l", 0.1161);
        this->declare_parameter<double>("kinematic_params.r", 1.33595);
        this->declare_parameter<double>("kinematic_params.a", 0.267);

        /* Initial Robot Pose */
        this->declare_parameter<double>("initial.x", 0.0);
        this->declare_parameter<double>("initial.y", 0.0);
        this->declare_parameter<double>("initial.theta", 0.0);

        /* Independet Covariance Matrices (Prediction, Wheel Odometry, ICP, IMU) */
        this->declare_parameter<double>("pred_cov.std_v", 0.3);
        this->declare_parameter<double>("pred_cov.std_w", 0.2);
        this->declare_parameter<double>("enc_cov.std_v", 0.1);
        this->declare_parameter<double>("enc_cov.std_w", 0.05);    
        this->declare_parameter<double>("icp_cov.std_v", 0.1);
        this->declare_parameter<double>("icp_cov.std_w", 0.2);
        this->declare_parameter<double>("imu_cov.std_w", 0.001);

        /* Quality of Service */
        auto icp_qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

        /* Joint State Subscriber */ 
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(get_parameter("joint_states_topic").as_string(), 10, std::bind(&FUSION_NODE::encoder_update, this, std::placeholders::_1));
        
        /* ICP Odometry Subscriber */
        icp_sub = this->create_subscription<nav_msgs::msg::Odometry>(get_parameter("icp_topic").as_string(), icp_qos, std::bind(&FUSION_NODE::icp_update, this, std::placeholders::_1));
      
        /* IMU Data Subscriber */
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), icp_qos, std::bind(&FUSION_NODE::imu_update, this, std::placeholders::_1));

        /* Fused Odometry Publisher */
        fused_pub = this->create_publisher<nav_msgs::msg::Odometry>(get_parameter("output_fusion_topic").as_string(), rclcpp::QoS(1));

        /* Create Timer To Publish Fused Odometry at 30ms*/
        timer_ = this->create_wall_timer(30ms, std::bind(&FUSION_NODE::prediction, this));

        /* Get Covariance Standard Deviations */
        Q(0, 0)     = get_parameter("pred_cov.std_v").as_double();
        Q(1, 1)     = get_parameter("pred_cov.std_w").as_double();
        R_enc(0, 0) = get_parameter("enc_cov.std_v").as_double();
        R_enc(1, 1) = get_parameter("enc_cov.std_w").as_double();
        R_icp(0, 0) = get_parameter("icp_cov.std_v").as_double();
        R_icp(1, 1) = get_parameter("icp_cov.std_w").as_double();
        R_imu       = get_parameter("imu_cov.std_w").as_double();

        /* Get Kinematic Geometric Parameters*/
        L = get_parameter("kinematic_params.l").as_double();
        R = get_parameter("kinematic_params.r").as_double();
        a = get_parameter("kinematic_params.a").as_double();
        
        /* Initial Robot Pose (Optinal just for Debugging/Visualization Purposes) */
        x_map = get_parameter("initial.x").as_double();
        y_map = get_parameter("initial.y").as_double();
        theta_map = get_parameter("initial.theta").as_double();

        std::cout << "Initial Robot Pose" << std::endl;
        std::cout << "Initial x: "  << x_map << std::endl;
        std::cout << "Initial y: "  << y_map << std::endl;
        std::cout << "Initial Theta: "  << theta_map << std::endl;    

        std::cout << "Q COV\n"      << Q     << std::endl;
        std::cout << "R_enc COV\n"  << R_enc << std::endl;
        std::cout << "R_icp COV\n"  << R_icp << std::endl;
        std::cout << "R_imu COV\n"  << R_imu << std::endl;

        std::cout << "joint_states_topic: "  << get_parameter("joint_states_topic").as_string()  << std::endl;
        std::cout << "icp_topic: "           << get_parameter("icp_topic").as_string()           << std::endl;
        std::cout << "imu_topic: "           << get_parameter("imu_topic").as_string()           << std::endl;
        std::cout << "output_fusion_topic: " << get_parameter("output_fusion_topic").as_string() << std::endl;

        }

        ~FUSION_NODE()
        {

        }

        void publish_fused_odom(double v, double w, double dt = 0.03)
        {
            nav_msgs::msg::Odometry fused_odom_msg;

            /* Compute The pose of the Robot Based on the Fused Linear and Angular velocities */
            x_map = x_map + dt*v*std::cos(theta_map);
            y_map = y_map + dt*v*std::sin(theta_map);
            theta_map = theta_map + dt*w;

            /*Header*/
            fused_odom_msg.header.stamp = this->now();
            fused_odom_msg.header.frame_id = "odom";
            fused_odom_msg.child_frame_id = "base_link";
            /*Position*/
            fused_odom_msg.pose.pose.position.x = x_map ;
            fused_odom_msg.pose.pose.position.y = y_map;
            fused_odom_msg.pose.pose.position.z = 0;
            /*Orientation*/
            fused_odom_msg.pose.pose.orientation.x = 0;
            fused_odom_msg.pose.pose.orientation.y = 0;
            fused_odom_msg.pose.pose.orientation.z = std::sin(theta_map/2);
            fused_odom_msg.pose.pose.orientation.w = std::cos(theta_map/2);
            /*Linear Twist*/
            fused_odom_msg.twist.twist.linear.x = v;
            fused_odom_msg.twist.twist.linear.y = 0;
            fused_odom_msg.twist.twist.linear.z = 0;
            /*Angular Twist*/
            fused_odom_msg.twist.twist.angular.x = 0;
            fused_odom_msg.twist.twist.angular.y = 0;
            fused_odom_msg.twist.twist.angular.z = w;
            /*Publish odom msg*/
            fused_pub->publish(fused_odom_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FUSION_NODE>("fusion_node"));
    rclcpp::shutdown();
    return 0;
}