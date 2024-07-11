
#ifndef PND_CONTROLLER_NODE_H
#define PND_CONTROLLER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "rmw/qos_profiles.h"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pnd_msgs/msg/drive_mode.hpp>
#include <pnd_msgs/srv/set_drive_mode.hpp>

#include <iostream>
#include <math.h>
#include <chrono> 
#include <vector>

struct aero_control_cmd
{
   int id = -1; // 0:FL 1:FR 2:RL 3:RR
   bool b_activate = false;
};



class PnDControlNode : public rclcpp::Node {

public:

    // PnDControlNode(const std::string &node_name_);
    PnDControlNode();

    // ~PnDControlNode() {}

private:
    void VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void publish_speed_cmd(double fl_speed, double fr_speed, double rl_speed, double rr_speed);
    void publish_steer_cmd(double fl_speed, double fr_speed, double rl_speed, double rr_speed);
   

    std::vector<double> CalculateLoadTransfer();

    std::vector<double> CalculateLatTireForce(const std::vector<double> F_z_estimated);

    std::vector<double> CalculateLonTireForce(); 

    std::vector<aero_control_cmd> AeroControlCmd(const std::vector<double> F_z_estimated, const std::vector<double> F_x_required, const std::vector<double> F_y_required);

    void timerCallback();

    void service_handler_dvive_mode(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const pnd_msgs::srv::SetDriveMode::Request::SharedPtr request,
        pnd_msgs::srv::SetDriveMode::Response::SharedPtr response);
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pnd_steer_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pnd_speed_pub;
    rclcpp::Publisher<pnd_msgs::msg::DriveMode>::SharedPtr pnd_drivemode_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;

    rclcpp::Service<pnd_msgs::srv::SetDriveMode>::SharedPtr service_drive_mode;



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

    bool b_imu_first_callback = false;
    bool b_vel_first_callback = false;
    bool b_yaw_accel_first_callback = false;
   
    double vel_x;
    double vel_yaw;

    int drive_mode = 2; // Default : Ackerman mode
    
};  


#endif // PND_CONTROLLER_NODE_H