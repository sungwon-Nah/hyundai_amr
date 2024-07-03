
#ifndef AERO_CONTROLLER_NODE_H
#define AERO_CONTROLLER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "rmw/qos_profiles.h"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <iostream>
#include <math.h>
#include <chrono> 
#include <vector>

struct aero_control_cmd
{
   int id = -1; // 0:FL 1:FR 2:RL 3:RR
   bool b_activate = false;
};



class AeroControlNode : public rclcpp::Node {

public:

    // AeroControlNode(const std::string &node_name_);
    AeroControlNode();

    // ~AeroControlNode() {}

private:
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void VelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void YawAccelCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // void MessageFilteringCallback(
    //     const sensor_msgs::msg::Image::ConstSharedPtr &in_image,
    //     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_pc2);

    std::vector<double> CalculateLoadTransfer();

    std::vector<double> CalculateLatTireForce(const std::vector<double> F_z_estimated);

    std::vector<double> CalculateLonTireForce(); 

    std::vector<aero_control_cmd> AeroControlCmd(const std::vector<double> F_z_estimated, const std::vector<double> F_x_required, const std::vector<double> F_y_required);

    void timerCallback();
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr FL_activate_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr FR_activate_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr RL_activate_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr RR_activate_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr F_z_est_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr F_z_req_pub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_accel_sub;

    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_filtered_Image;
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_filtered_Points;

    // using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
    //     sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;
    // std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> m_sync;

    // rclcpp::Time m_radar_callback_time;

    std::vector<bool> current_state; // To keep track of the current state of each control unit

    double height_of_cg = 0.0; 
    double total_wheelbase_length = 0.;
    double front_wheelbase_length = 0.;
    double rear_wheelbase_length = 0.;
    double track_width = 0.;
    double gravity_acceleration = 0.;
    double total_mass = 0.;
    double moment_of_inertia = 0.;
    double friction_coefficient = 0.;
    double hysteresis_band = 0.;

    bool b_imu_first_callback = false;
    bool b_vel_first_callback = false;
    bool b_yaw_accel_first_callback = false;
    double yaw_rate;
    double accel_x;
    double accel_y;
    double yaw_accel;
    double vel_x;
    double vel_y;
    
};  


#endif // AERO_CONTROLLER_NODE_H