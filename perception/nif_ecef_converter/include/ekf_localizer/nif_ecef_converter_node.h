//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by USRG on 9/14/21.
//

#ifndef ECEF_CONVERTER_NODE_H
#define ECEF_CONVERTER_NODE_H


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "rmw/qos_profiles.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>



#include "amathutils_lib/kalman_filter.hpp"
#include "amathutils_lib/time_delay_kalman_filter.hpp"

#include <chrono>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>

#include <utils/geodetic_conv.h>

using std::placeholders::_1;
using std::placeholders::_2;



class ECEFConverterNode : public rclcpp::Node
{

public:
    ECEFConverterNode();
    // ~ECEFConverterNode();

private:

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gq7_ins_odometry;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ins_sub;

    using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>;

    message_filters::Subscriber<sensor_msgs::msg::Imu> sub_filtered_imu;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> sub_filtered_ins;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> m_sync;

    void GQ7FilteringCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &ins_msg);
  

    void INSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    double m_origin_lat = 0.0;
    double m_origin_lon = 0.0;

    // Geodetic to local Coordinate
    nif::localization::utils::GeodeticConverter conv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
   
    
};

#endif // ECEF_CONVERTER_NODE_H