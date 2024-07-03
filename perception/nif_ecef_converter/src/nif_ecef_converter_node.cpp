#include "ekf_localizer/nif_ecef_converter_node.h"
// #include <random>

ECEFConverterNode::ECEFConverterNode()
    : Node("ECEF_Converter_node") { 

    RCLCPP_INFO(this->get_logger(), "Inititalizing ECEF_Converter_node...");
    // Load Parameters

    this->declare_parameter<double>("m_origin_lat", 36.3660857); // KAIST
    this->declare_parameter<double>("m_origin_lon", 127.3636824);
   
    // this->declare_parameter<bool>("test_mode", true);
   

    this->m_origin_lat = this->get_parameter("m_origin_lat").as_double();
    this->m_origin_lon = this->get_parameter("m_origin_lon").as_double();
    
    // this->test_mode = this->get_parameter("test_mode").as_bool();

    // Set the ltp reference point
    nif::localization::utils::GeodeticConverter::GeoRef ref;
    ref.latitude = m_origin_lat;
    ref.longitude = m_origin_lon;
    ref.altitude = 0.;
    conv_.initializeReference(ref);

    pub_gq7_ins_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "/gq7/odom", rclcpp::SensorDataQoS());

    // ins_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    //   "/ekf/llh_position", rclcpp::SensorDataQoS(),
    //   std::bind(&ECEFConverterNode::INSCallback, this, std::placeholders::_1));

    auto rmw_qos_profile = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    // For message filter
    sub_filtered_imu.subscribe(this, "/ekf/imu/data", rmw_qos_profile);
    sub_filtered_ins.subscribe(this, "/ekf/llh_position", rmw_qos_profile);

    // sub_filtered_imu = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "/ekf/imu/data", rclcpp::SensorDataQoS());
    // sub_filtered_ins = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, "/ekf/llh_position", rclcpp::SensorDataQoS());

    m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_filtered_imu, sub_filtered_ins);

    m_sync->registerCallback(std::bind(&ECEFConverterNode::GQ7FilteringCallback,
                                        this, std::placeholders::_1,
                                        std::placeholders::_2));

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

}

void ECEFConverterNode::GQ7FilteringCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr &ins_msg)
{

  tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  tf2::Quaternion quat_on_global;
  quat_on_global.setRPY(roll, -pitch, -yaw);


  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)ins_msg->latitude;
  currentGPS.longitude = (double)ins_msg->longitude;
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry gq7_ins_odom;

  gq7_ins_odom.header.frame_id = "odom";
  gq7_ins_odom.header.stamp = this->now();
  gq7_ins_odom.child_frame_id = "base_link";

  gq7_ins_odom.pose.pose.position.x = ltp_pt.x;
  gq7_ins_odom.pose.pose.position.y = -ltp_pt.y;
  gq7_ins_odom.pose.pose.position.z = 0.0;
  gq7_ins_odom.pose.pose.orientation.x = quat_on_global.x();
  gq7_ins_odom.pose.pose.orientation.y = quat_on_global.y();
  gq7_ins_odom.pose.pose.orientation.z = quat_on_global.z();
  gq7_ins_odom.pose.pose.orientation.w = quat_on_global.w();


  pub_gq7_ins_odometry->publish(gq7_ins_odom);

  // To Broadcast the TF (ODOM TO BASELINK)
  geometry_msgs::msg::TransformStamped nav_base_tf{};
  nav_base_tf.transform.translation.x = ltp_pt.x;
  nav_base_tf.transform.translation.y = -ltp_pt.y;
  nav_base_tf.transform.translation.z = 0.0;
  nav_base_tf.transform.rotation.w = quat_on_global.x();
  nav_base_tf.transform.rotation.x = quat_on_global.y();
  nav_base_tf.transform.rotation.y = quat_on_global.z();
  nav_base_tf.transform.rotation.z = quat_on_global.w();
  nav_base_tf.header.stamp = this->now();
  nav_base_tf.header.frame_id = "odom";
  nav_base_tf.child_frame_id = "base_link";
  broadcaster_->sendTransform(nav_base_tf);
}

