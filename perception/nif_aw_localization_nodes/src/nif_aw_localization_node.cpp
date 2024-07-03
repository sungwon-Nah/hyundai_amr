#include "ekf_localizer/nif_aw_localization_node.h"
#include "nif_frame_id/frame_id.h"

#include <random>

using namespace message_filters;
using namespace std::placeholders;

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
// #define DEBUG_INFO(...) { if (show_debug_info_) { RCLCPP_INFO(this->get_logger(),  _VA_ARGS__); } }
#define DEBUG_PRINT_MAT(X) { if (show_debug_info_) { std::cout << #X << ": " << X << std::endl; } }

using novatel_oem7_msgs::msg::InertialSolutionStatus;
using novatel_oem7_msgs::msg::PositionOrVelocityType;

using vectornav_msgs::msg::InsStatus;
using vectornav_msgs::msg::GpsGroup;

// clang-format on
AWLocalizationNode::AWLocalizationNode(const std::string &node_name)
    // : IBaseNode(node_name), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
    : IBaseNode(node_name), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)

{
  this->declare_parameter<bool>("use_inspva_heading", bool(true));
  this->declare_parameter<double>("bestvel_heading_update_velocity_thres", double(2.));
  this->declare_parameter<double>("heading_initial_guess_deg", double(-4.));
  this->declare_parameter<bool>("heading_initial_guess_enabled", bool(false));
  this->declare_parameter<double>("heading_lowpass_gain", double(0.90));
  this->declare_parameter<double>("heading_heading2_offset_deg", double(0.0));
  m_origin_lat = this->get_global_parameter<double>("coordinates.ecef_ref_lat");
  m_origin_lon = this->get_global_parameter<double>("coordinates.ecef_ref_lon");

  this->declare_parameter<bool>("show_debug_info", bool(false));
  this->declare_parameter<double>("predict_frequency", double(50.0));
  this->declare_parameter<double>("tf_rate", double(10.0));
  this->declare_parameter<bool>("enable_yaw_bias_estimation", bool(true));
  this->declare_parameter<int>("extend_state_step", int(50));
  this->declare_parameter<std::string>("pose_frame_id", std::string("odom"));

  /* pose measurement */
  this->declare_parameter<double>("pose_additional_delay", double(0.0));
  this->declare_parameter<double>("pose_measure_uncertainty_time", double(0.01));
  this->declare_parameter<double>("pose_rate", double(20.0));      //  used for covariance calculation
  this->declare_parameter<double>("pose_gate_dist", double(30.0)); //  Mahalanobis limit
  this->declare_parameter<double>("pose_stddev_x", double(0.05));
  this->declare_parameter<double>("pose_stddev_y", double(0.05));
  this->declare_parameter<double>("pose_stddev_yaw", double(0.035));
  this->declare_parameter<bool>("use_pose_with_covariance", bool(false));

  /* twist measurement */
  this->declare_parameter<double>("twist_additional_delay", double(0.0));
  this->declare_parameter<double>("twist_rate", double(100.0));        //  used for covariance calculation
  this->declare_parameter<double>("twist_gate_dist", double(10000.0)); //  Mahalanobis limit
  this->declare_parameter<double>("twist_stddev_vx", double(0.2));
  this->declare_parameter<double>("twist_stddev_wz", double(0.03));
  this->declare_parameter<bool>("use_twist_with_covariance", bool(false));

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_yaw_bias_c, proc_stddev_vx_c, proc_stddev_wz_c;
  this->declare_parameter<double>("proc_stddev_yaw_c", double(0.005));
  this->declare_parameter<double>("proc_stddev_yaw_bias_c", double(0.001));
  this->declare_parameter<double>("proc_stddev_vx_c", double(2.0));
  this->declare_parameter<double>("proc_stddev_wz_c", double(0.2));

  this->declare_parameter<double>("top_to_bottom_bias_x", double(0.0));
  this->declare_parameter<double>("top_to_bottom_bias_y", double(0.0));
  this->declare_parameter<double>("bottom_to_top_bias_x", double(0.0));
  this->declare_parameter<double>("bottom_to_top_bias_y", double(0.0));

  this->declare_parameter<bool>("use_bestvel_for_speed", bool(false));
  this->declare_parameter<bool>("enable_tf_publisher", bool(true));

  this->declare_parameter<bool>("publish_reference_points", bool(false));
  this->publish_reference_points =  this->get_parameter("publish_reference_points").as_bool();
  this->declare_parameter<bool>("use_reference_information", bool(false));
  this->use_reference_information = this->get_parameter("use_reference_information").as_bool();

  this->declare_parameter<std::string>("reference_information_filename", "");
	this->reference_information_filename =
      	this->get_parameter("reference_information_filename").as_string();


  this->m_use_inspva_heading =
      this->get_parameter("use_inspva_heading").as_bool();
  this->m_bestvel_heading_update_thres =
      this->get_parameter("bestvel_heading_update_velocity_thres").as_double();
  this->m_heading_initial_guess_enabled =
      this->get_parameter("heading_initial_guess_enabled").as_bool();

  if (this->m_heading_initial_guess_enabled)
  {
    this->m_best_heading_rad =
        this->get_parameter("heading_initial_guess_deg").as_double() * nif::common::constants::DEG2RAD * (-1.0);

    BestPosBottom.yaw = m_best_heading_rad;
    BestPosTop.yaw = m_best_heading_rad;
    m_dGPS_TOP_Heading = m_best_heading_rad; // Hyunwoo   - for arrow direction
    m_dGPS_Heading = m_best_heading_rad;     // Hyunwoo       - for arrow direction
    m_dTOP_INS_Heading = m_best_heading_rad;
    m_dBOTTOM_INS_Heading = m_best_heading_rad;
    m_dGPS_Kinematic_Heading = m_best_heading_rad;
    m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
    m_d_TOP_GPS_Kinematic_Heading = m_best_heading_rad;
    m_last_d_TOP_GPS_Kinematic_Heading = m_d_TOP_GPS_Kinematic_Heading;
    m_dVN1_Heading = m_best_heading_rad;
    m_dVN2_Heading = m_best_heading_rad;
    m_XSENS_GPS_Heading = m_best_heading_rad;
    m_heading_initialized = true;
    m_heading_error = false;
    m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::FORCED_HEADING;
    bForcedHeading = true;
  }
  this->m_heading_lowpass_gain =
      this->get_parameter("heading_lowpass_gain").as_double();

  this->m_heading_heading2_offset_rad =
      this->get_parameter("heading_heading2_offset_deg").as_double() * nif::common::constants::DEG2RAD;

  this->show_debug_info_ = this->get_parameter("show_debug_info").as_bool();
  this->ekf_rate_ = this->get_parameter("predict_frequency").as_double();
  this->tf_rate_ = this->get_parameter("tf_rate").as_double();
  this->enable_yaw_bias_estimation_ = this->get_parameter("enable_yaw_bias_estimation").as_bool();
  this->extend_state_step_ = this->get_parameter("extend_state_step").as_int();

  /* pose measurement */
  this->pose_additional_delay_ = this->get_parameter("pose_additional_delay").as_double();
  this->pose_measure_uncertainty_time_ = this->get_parameter("pose_measure_uncertainty_time").as_double();
  this->pose_rate_ = this->get_parameter("pose_rate").as_double();           //  used for covariance calculation
  this->pose_gate_dist_ = this->get_parameter("pose_gate_dist").as_double(); //  Mahalanobis limit
  this->pose_stddev_x_ = this->get_parameter("pose_stddev_x").as_double();
  this->pose_stddev_y_ = this->get_parameter("pose_stddev_y").as_double();
  this->pose_stddev_yaw_ = this->get_parameter("pose_stddev_yaw").as_double();
  this->use_pose_with_covariance_ = this->get_parameter("use_pose_with_covariance").as_bool();

  /* twist measurement */
  this->twist_additional_delay_ = this->get_parameter("twist_additional_delay").as_double();
  this->twist_rate_ = this->get_parameter("twist_rate").as_double();           //  used for covariance calculation
  this->twist_gate_dist_ = this->get_parameter("twist_gate_dist").as_double(); //  Mahalanobis limit
  this->twist_stddev_vx_ = this->get_parameter("twist_stddev_vx").as_double();
  this->twist_stddev_wz_ = this->get_parameter("twist_stddev_wz").as_double();
  this->use_twist_with_covariance_ = this->get_parameter("use_twist_with_covariance").as_bool();

  /* process noise */
  proc_stddev_yaw_c = this->get_parameter("proc_stddev_yaw_c").as_double();
  proc_stddev_yaw_bias_c = this->get_parameter("proc_stddev_yaw_bias_c").as_double();
  proc_stddev_vx_c = this->get_parameter("proc_stddev_vx_c").as_double();
  proc_stddev_wz_c = this->get_parameter("proc_stddev_wz_c").as_double();

  this->top_to_bottom_bias_x = this->get_parameter("top_to_bottom_bias_x").as_double();
  this->top_to_bottom_bias_y = this->get_parameter("top_to_bottom_bias_y").as_double();
  this->bottom_to_top_bias_x = this->get_parameter("bottom_to_top_bias_x").as_double();
  this->bottom_to_top_bias_y = this->get_parameter("bottom_to_top_bias_y").as_double();
  this->bUseBestVelForSpeed =
      this->get_parameter("use_bestvel_for_speed").as_bool();

  this->enable_tf_publisher = this->get_parameter("enable_tf_publisher").as_bool();

  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  if (!enable_yaw_bias_estimation_)
  {
    proc_stddev_yaw_bias_c = 0.0;
  }
  RCLCPP_INFO(this->get_logger(), "ORIGIN LATITUDE : %f", this->m_origin_lat);
  RCLCPP_INFO(this->get_logger(), "ORIGIN LONGITUDE : %f", this->m_origin_lon);

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c, 2.0) * ekf_dt_;
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c, 2.0) * ekf_dt_;
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c, 2.0) * ekf_dt_;
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c, 2.0) * ekf_dt_;

  /* initialize ros system */
  using namespace std::chrono_literals; // NOLINT
  timer_control_ = this->create_wall_timer(
      10ms, std::bind(&AWLocalizationNode::timerCallback, this));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_ekf_estimated", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_yaw_bias_ = this->create_publisher<std_msgs::msg::Float64>("estimated_yaw_bias", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_bestpos_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_bestpos", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_top_bestpos_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_top_odometry_bestpos", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_bottom_ins_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_bottom_ins", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_top_ins_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_top_ins", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_xsens_odom = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_xsens_odom", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_mahalanobisScore = this->create_publisher<std_msgs::msg::Float64>(
      "out_localization_error", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_localization_status = this->create_publisher<nif_msgs::msg::LocalizationStatus>(
      "out_localization_status", nif::common::constants::QOS_INTERNAL_STATUS);

  // POSE(X, Y)
  subBESTPOS = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_bestpos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::BESTPOSCallback, this, std::placeholders::_1));
  subTOPBESTPOS = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_top_bestpos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPBESTPOSCallback, this,
                std::placeholders::_1));
  // FOR VECTORNAV
  pub_vn1_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_vn_odom1", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_vn2_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_vn_odom2", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_vn_ins_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_vn_ins_odom", nif::common::constants::QOS_EGO_ODOMETRY);

  // For Xsens
  subXSENSPOS = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "in_xsens_pos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::XSENSPOSCallback, this, std::placeholders::_1));

  // HEADING BACK-UP SOLUTION
  subBESTVEL = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "in_bestvel", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::BOTTOMBESTVELCallback, this, std::placeholders::_1));

  subTOPBESTVEL = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "in_bestvel_top", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPBESTVELCallback, this, std::placeholders::_1));

  subHEADING2 = this->create_subscription<novatel_oem7_msgs::msg::HEADING2>(
      "in_heading2", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::HEADING2Callback, this, std::placeholders::_1));

  subTOPHEADING2 = this->create_subscription<novatel_oem7_msgs::msg::HEADING2>(
      "in_top_heading2", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPHEADING2Callback, this, std::placeholders::_1));

  subIMUONLY = this->create_subscription<sensor_msgs::msg::Imu>(
      "in_top_imu", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::IMUCallback, this,
                std::placeholders::_1)); // use this when putting bestvel speed as vehicle velocity.

  subTOPIMUONLY = this->create_subscription<sensor_msgs::msg::Imu>(
      "in_imu", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::IMUCallback, this,
                std::placeholders::_1)); // use this when putting bestvel speed as vehicle velocity.

  // THIS IS FOR TUM DATASET
  subIMUONLY2 = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "in_imu2", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::IMUCallback2, this,
                std::placeholders::_1)); // use this when putting bestvel speed as vehicle velocity.

  subtest = this->create_subscription<std_msgs::msg::Float64>(
      "test_noise", 1,
      std::bind(&AWLocalizationNode::TestCallback, this,
                std::placeholders::_1));
  // FOR VECTORNAV
  subVNATTITUDE = this->create_subscription<vectornav_msgs::msg::AttitudeGroup>(
      "in_vn_attitude", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::VNATTITUDECallback, this,
                std::placeholders::_1));

  subVNGPS1 = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "in_vn_gps1", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::VNGPS1Callback, this,
                std::placeholders::_1));

  subVNGPS2 = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "in_vn_gps2", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::VNGPS2Callback, this,
                std::placeholders::_1));
  subVNINS = this->create_subscription<vectornav_msgs::msg::InsGroup>(
      "in_vn_ins", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::VNINSCallback, this,
                std::placeholders::_1));

  subSystemStatus =
      this->create_subscription<nif::common::msgs::SystemStatus>(
          "/system/status", nif::common::constants::QOS_INTERNAL_STATUS,
          std::bind(&AWLocalizationNode::SystemStatusCallback, this,
                    std::placeholders::_1));

  this->service_forced_heading = this->create_service<nif_msgs::srv::LocalizationForcedHeading>(
      "/aw_localization/forced_heading",
      std::bind(
          &AWLocalizationNode::service_handler_forced_heading,
          this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  this->service_forced_initialize = this->create_service<nif_msgs::srv::LocalizationForcedInitialize>(
      "/aw_localization/forced_initialize",
      std::bind(
          &AWLocalizationNode::service_handler_forced_initialize,
          this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  auto rmw_qos_profile =
      nif::common::constants::QOS_SENSOR_DATA.get_rmw_qos_profile();

  if (this->m_use_inspva_heading){
    sub_filtered_ins_bottom.subscribe(this, "in_inspva", rmw_qos_profile);
    sub_filtered_insstdev_bottom.subscribe(this, "in_insstdev", rmw_qos_profile);
    sub_filtered_ins_top.subscribe(this, "in_top_inspva", rmw_qos_profile);
    sub_filtered_insstdev_top.subscribe(this, "in_top_insstdev", rmw_qos_profile);

    m_ins_bottom_sync = std::make_shared<message_filters::Synchronizer<NovatelINSSyncPolicyT>>(
        NovatelINSSyncPolicyT(10), sub_filtered_ins_bottom, sub_filtered_insstdev_bottom);
    m_ins_bottom_sync->registerCallback(std::bind(&AWLocalizationNode::NovatelBottomINSCallback,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));
    m_ins_top_sync = std::make_shared<message_filters::Synchronizer<NovatelINSSyncPolicyT>>(
        NovatelINSSyncPolicyT(10), sub_filtered_ins_top, sub_filtered_insstdev_top);
    m_ins_top_sync->registerCallback(std::bind(&AWLocalizationNode::NovatelTopINSCallback,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));
  }

  sub_filtered_IMU.subscribe(this, "in_imu", rmw_qos_profile);
  sub_filtered_Wheel.subscribe(this, "in_wheel_speed_report", rmw_qos_profile);

  sub_filtered_TOP_IMU.subscribe(this, "in_top_imu", rmw_qos_profile);
  // FOR VECTORNAV
  sub_filtered_VNCOMMON.subscribe(this, "in_vn_common", rmw_qos_profile);
  // FOR XSENS
  sub_filtered_XSENSIMU.subscribe(this, "in_xsens_imu", rmw_qos_profile);

  m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_filtered_IMU, sub_filtered_Wheel);
  m_sync->registerCallback(std::bind(&AWLocalizationNode::MessageFilteringCallback,
                                     this, std::placeholders::_1,
                                     std::placeholders::_2));

  m_top_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_filtered_TOP_IMU, sub_filtered_Wheel);
  m_top_sync->registerCallback(std::bind(&AWLocalizationNode::TOPMessageFilteringCallback,
                                         this, std::placeholders::_1,
                                         std::placeholders::_2));
  // FOR VECTORNAV
  m_vn_sync = std::make_shared<message_filters::Synchronizer<VNSyncPolicyT>>(
      VNSyncPolicyT(10), sub_filtered_VNCOMMON, sub_filtered_Wheel);
  m_vn_sync->registerCallback(std::bind(&AWLocalizationNode::VNMessageFilteringCallback,
                                        this, std::placeholders::_1,
                                        std::placeholders::_2));
  // FOR XSENS
  m_xsens_sync = std::make_shared<message_filters::Synchronizer<XSENSSyncPolicyT>>(
      XSENSSyncPolicyT(10), sub_filtered_XSENSIMU, sub_filtered_Wheel);
  m_xsens_sync->registerCallback(std::bind(&AWLocalizationNode::XSENSMessageFilteringCallback,
                                           this, std::placeholders::_1,
                                           std::placeholders::_2));

  dim_x_ex_ = dim_x_ * extend_state_step_;
  //  twist_linear_x = 0;

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

	if(this->use_reference_information){
    pub_gnss_reference_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "gnss_reference_points", nif::common::constants::QOS_SENSOR_DATA);
		RCLCPP_INFO(this->get_logger(), "REFERENCE INFORMATION FILE : ", reference_information_filename);
		pcdFileIO();
		ReferencePublisher();
	}

  initEKF();

  /* debug */
  pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("debug", 1);
  pub_measured_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);

  // Set the ltp reference point
  nif::localization::utils::GeodeticConverter::GeoRef ref;
  ref.latitude = m_origin_lat;
  ref.longitude = m_origin_lon;
  ref.altitude = 0.;
  conv_.initializeReference(ref);

  gps_timeout = rclcpp::Duration(1, 0);
};

AWLocalizationNode::~AWLocalizationNode(){};

/*
 * timerCallback
 */
void AWLocalizationNode::timerCallback()
{
  if ((bImuFirstCall || bTOPImuFirstCall || bVNIMUFirstCall || bXSENSIMUFirstCall) && (bBOTTOM_GPS || bTOP_GPS || bVN_GPS1 || bVN_INS || bInitConverged) && (bBOTTOMGPSHeading || bTOPGPSHeading || bVNINSHeadingValid ||m_heading_initialized))
  {
    // FOR SENSOR SAFETY CHECK
    auto node_status = nif::common::NODE_OK;
    auto now = this->now().nanoseconds();
    if ((now - bestpos_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      bBOTTOM_GPS = false;
      if (m_bestvel_bottom_valid)
        bFirstBottomKinematicHeading = true;
      m_bestvel_bottom_valid = false;
      m_heading2_valid = false;
      m_kinematic_heading_bottom_valid = false;
    }
    if ((now - top_bestpos_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      bTOP_GPS = false;
      if (m_bestvel_top_valid)
        bFirstTopKinematicHeading = true;
      m_bestvel_top_valid = false;
      m_top_heading2_valid = false;
      m_kinematic_heading_top_valid = false;
    }
    if ((now - imu_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      bImuFirstCall = false;
    }
    if ((now - top_imu_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      bTOPImuFirstCall = false;
    }
    // FOR NOVATEL INS
    if ((now - top_ins_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      m_bTOPINS_initialized = false;
    }
    if ((now - bottom_ins_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    {
      m_bBOTTOMINS_initialized = false;
    }
    // FOR VECTORNAV
    if ((now - vn_gps1_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds())
    {
      bVN_GPS1 = false;
      bVN1HeadingValid = false;
    }
    if ((now - vn_gps2_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds())
    {
      bVN_GPS2 = false;
      bVN2HeadingValid = false;
    }
    if (((now - vn_imu_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)||((now - vn_ins_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5))
    {
      bVNIMUFirstCall = false;
      m_bVNINS_initialized = false;
      bVNINSHeadingValid = false;
      bVN_INS = false;
      m_localization_status.vectornav_connection = false;
    }
    // FOR XSENS
    // if ((now - xsens_gps_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds())
    // {
    //   bXSENS_GPS = false;
    //   m_XSENS_GPS_Heading = false;
    // }
    // if ((now - xsens_imu_time_last_update.nanoseconds()) > this->gps_timeout.nanoseconds() / 5)
    // {
    //   bXSENSIMUFirstCall = false;
    // }

    predictKinematicsModel();

    VehPose_t BestCorrection;
    VehPose_t CurrentEstimated;
    CurrentEstimated.x = ekf_.getXelement(IDX::X);
    CurrentEstimated.y = ekf_.getXelement(IDX::Y);
    CurrentEstimated.yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);

    double veh_vel_x = 0.0, veh_yaw_rate = 0.0, correction_x = 0.0, correction_y = 0.0, correction_yaw = 0.0;

    veh_vel_x = m_dVelolcity_X;
    veh_yaw_rate = m_dIMU_yaw_rate;
    // FOR VECTORNAV & SENSOR SAFETY CHECK
    if (bImuFirstCall)
    {
      update_twist_time = imu_time_last_update;
      veh_yaw_rate = m_dIMU_yaw_rate;
    }
    else if (bTOPImuFirstCall)
    {
      update_twist_time = top_imu_time_last_update;
      veh_yaw_rate = m_dTOP_IMU_yaw_rate;
    }
    else if (bVNIMUFirstCall)
    {
      update_twist_time = vn_imu_time_last_update;
      veh_yaw_rate = m_dVN_yaw_rate;
    }
    // FOR XSENS
    // else if (bXSENSIMUFirstCall)
    // {
    //   update_twist_time = xsens_imu_time_last_update;
    //   veh_yaw_rate = m_dXSENS_yaw_rate;
    // }

    // standard deviation from novatel bottom/top
    double bottom_noise_total =
        sqrt(BestPosBottom.lat_noise * BestPosBottom.lat_noise +
             BestPosBottom.lon_noise * BestPosBottom.lon_noise);
    double top_noise_total = sqrt(BestPosTop.lat_noise * BestPosTop.lat_noise +
                                  BestPosTop.lon_noise * BestPosTop.lon_noise);
                                  
    m_localization_status.top_error = top_noise_total;
    m_localization_status.bottom_error = bottom_noise_total;
    /**
     * @brief select best correction data
     */
    bool update_pose = false;

    //TODO:: POSE UPDATE WITH VNINS & NOVATEL INS
    if (bBOTTOM_GPS && bTOP_GPS && top_noise_total < 2.0 && bottom_noise_total < 2.0)
    {
      // More cases inside here
      update_pose = CalculateBestCorrection(BestPosBottom, BestPosTop,
                                            CurrentEstimated, BestCorrection);
      correction_x = BestCorrection.x;
      correction_y = BestCorrection.y;
      m_localization_status.top_initialized = true;
      m_localization_status.bottom_initialized = true;
      m_kinematic_heading_bottom_valid = true;
      m_kinematic_heading_top_valid = true;
      bNominalMeasurement = true;
    }
    else if (bBOTTOM_GPS &&           // if top is not received, use only bottom.
            bottom_noise_total < 2.0 // && BestPosBottom.quality_code_ok
            )                        // when top has large noise, but bottom is good, use bottom.
    {
      // No top, Bottom only mode is validated.
      // Also, InertialSolutionStatus is integrated/validated on the bottom only mode.
      correction_x = BestPosBottom.x;
      correction_y = BestPosBottom.y;
      update_pose = true;
      m_localization_status.status = "TOP IS BAD. ONLY BOTTOM";
      m_localization_status.top_initialized = false;
      m_localization_status.bottom_initialized = true;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM;
      update_pose_time = bestpos_time_last_update;

      Eigen::MatrixXd P_curr, P_2by2;
      ekf_.getLatestP(P_curr);
      P_2by2 = P_curr.block(0, 0, 2, 2);

      Eigen::MatrixXd y_est(2, 1);
      y_est << CurrentEstimated.x, CurrentEstimated.y;

      Eigen::MatrixXd y_bottom(2, 1);
      y_bottom << BestPosBottom.x, BestPosBottom.y;
      double bottomError = GetmahalanobisDistance(y_est, y_bottom, P_2by2);

      double bottom_yaw_error =
          fabs(BestPosBottom.yaw -
              (ekf_.getXelement(IDX::YAW) +
                ekf_.getXelement(IDX::YAWB))); // correction - ekf

      if (bottom_yaw_error > M_PI)
        bottom_yaw_error = bottom_yaw_error - 2 * M_PI;
      else if (bottom_yaw_error < -M_PI)
        bottom_yaw_error = bottom_yaw_error + 2 * M_PI;

      bottom_yaw_error = fabs(bottom_yaw_error);

      m_localization_status.bottom_error = bottomError;

      m_dNovatelCorrectionError = m_localization_status.bottom_error;


      if (bottomError < 5.0 && bottom_yaw_error < 0.1 // && BestPosBottom.quality_code_ok
      )
      {
        bInitConverged = true;
      }
      // m_bestvel_bottom_valid = true;
      if (m_bestvel_top_valid)
        bFirstTopKinematicHeading = true;
      m_bestvel_top_valid = false;
      m_kinematic_heading_bottom_valid = true;
      m_kinematic_heading_top_valid = false;
      bNominalMeasurement = true;
    }
    else if (bTOP_GPS &&           // If Bottom is not received, use top
            top_noise_total < 2.0 // && BestPosTop.quality_code_ok
    )
    {
      // No bottom, TOP only mode is validated.
      // Also, InertialSolutionStatus is integrated/validated on the top only
      // mode.
      correction_x = BestPosTop.x;
      correction_y = BestPosTop.y;
      update_pose = true;

      m_localization_status.status = "BOTTOM IS BAD. ONLY TOP";
      m_localization_status.top_initialized = true;
      m_localization_status.bottom_initialized = false;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_TOP;
      update_pose_time = top_bestpos_time_last_update;

      Eigen::MatrixXd P_curr, P_2by2;
      ekf_.getLatestP(P_curr);
      P_2by2 = P_curr.block(0, 0, 2, 2);

      Eigen::MatrixXd y_est(2, 1);
      y_est << CurrentEstimated.x, CurrentEstimated.y;

      Eigen::MatrixXd y_top(2, 1);
      y_top << BestPosTop.x, BestPosTop.y;
      double topError = GetmahalanobisDistance(y_est, y_top, P_2by2);

      double top_yaw_error =
          fabs(BestPosTop.yaw -
              (ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB)));

      m_localization_status.top_error = topError;

      m_dNovatelCorrectionError = m_localization_status.top_error;


      correction_yaw = BestPosTop.yaw;
      if (m_bestvel_bottom_valid)
        bFirstBottomKinematicHeading = true;
      m_bestvel_bottom_valid = false;

      // m_bestvel_top_valid = true;
      m_kinematic_heading_bottom_valid = false;
      m_kinematic_heading_top_valid = true;
      bNominalMeasurement = true;

      if (top_yaw_error > M_PI)
        top_yaw_error = top_yaw_error - 2 * M_PI;
      else if (top_yaw_error < -M_PI)
        top_yaw_error = top_yaw_error + 2 * M_PI;

      top_yaw_error = fabs(top_yaw_error);

      if (topError < 5.0 && top_yaw_error < 0.1 // && BestPosTop.quality_code_ok
      )
      {
        bInitConverged = true;
      }
    }
    else
    {
      m_dNovatelCorrectionError = 3.0;
    }

    if (bVN_INS&&m_bVNINS_initialized)
    {
      bool secondary_update_pose = false;
      
      secondary_update_pose = CalculateSecondaryCorrection(INSVectorNav, BestPosBottom, BestPosTop,
                                                CurrentEstimated, BestCorrection);

      if(secondary_update_pose){
        correction_x = BestCorrection.x;
        correction_y = BestCorrection.y;
        m_localization_status.top_initialized = false;
        m_localization_status.bottom_initialized = false;
        m_kinematic_heading_bottom_valid = true;
        m_kinematic_heading_top_valid = true;
        bNominalMeasurement = false;
        update_pose = secondary_update_pose;
      }
    }

    // From here, we state high GPS error status.
    // 1. check sensor is initialized(started) or not.
    // 2. see the noise or novatel ins status.

    // TODO:: consider other measurements
    if (bNominalMeasurement)
    {
      if (bBOTTOM_GPS && bTOP_GPS &&
          ((bottom_noise_total > 2.0 && top_noise_total > 2.0)))
      {
        // GPS_HIGH_ERROR -> MONZA_TEMP_GPS_HIGH_ERROR
        m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::MONZA_TEMP_GPS_HIGH_ERROR;
        update_pose = false;
        node_status = nif::common::NODE_ERROR;
      }
      else
      {
        // Either one of the two is bad or none.
        if ((bottom_noise_total > 2.0) && bBOTTOM_GPS && !bTOP_GPS)
        {
        // GPS_HIGH_ERROR -> MONZA_TEMP_GPS_HIGH_ERROR
          m_localization_status.status = "NO UPDATE, GPS HIGH ERROR, ONLY BOTTOM";
          m_localization_status.localization_status_code =
              nif_msgs::msg::LocalizationStatus::MONZA_TEMP_GPS_HIGH_ERROR;
          update_pose = false;
          node_status = nif::common::NODE_ERROR;
        }

        if (top_noise_total > 2.0 && bTOP_GPS && !bBOTTOM_GPS)
        {
        // GPS_HIGH_ERROR -> MONZA_TEMP_GPS_HIGH_ERROR
          m_localization_status.status = "NO UPDATE, GPS HIGH ERROR, ONLY TOP";
          m_localization_status.localization_status_code =
              nif_msgs::msg::LocalizationStatus::MONZA_TEMP_GPS_HIGH_ERROR;
          update_pose = false;
          node_status = nif::common::NODE_ERROR;
        }
      }
    }

    // check convergence has higher priority than "GPS HIGH ERROR"
    // even though GPS error is high, keep trying to find the converged value.
    if (!bInitConverged)
    {
      m_localization_status.status = "NO CONVERGED. WAITING FOR CONVERGENCE";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::NO_CONVERGED;

      update_pose = true;
      node_status = nif::common::NODE_ERROR;
    }

    if(!update_pose && bInitConverged && (bImuFirstCall || bTOPImuFirstCall||bVNIMUFirstCall))
    {
      m_localization_status.status = "ONLY IMU";
      m_localization_status.top_initialized = true;
      m_localization_status.bottom_initialized = true;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::MONZA_TEMP_GPS_HIGH_ERROR;
    }

    // correction Heading TODO::Priority
    // Select best heading among top/bottom VN_INS / Novatel_INS / bestvel / kinematic_heading and heading2

    double correction_prev, current_heading_est, correction_diff, heading_diff, pred_heading_change = 0.0;
    correction_prev = m_best_heading_rad;
    current_heading_est = CurrentEstimated.yaw;

    m_stdev_azimuth = 0.015;
    if(bVN_INS && bVNINSHeadingValid){
      m_best_heading_rad = m_dVN_INS_heading;
      m_stdev_azimuth = m_stdev_azimuth;
      m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::VNINS;
      // std::cout << "Heading Source: VN INS, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;
    }
    else if (m_use_inspva_heading && (m_top_inspva_heading_init || m_inspva_heading_init))// USE INSPVA HEADING
    {
      if (m_top_inspva_heading_init)
      {
        m_best_heading_rad = m_dTOP_INS_Heading;
        m_stdev_azimuth = m_top_stdev_azimuth;
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_INS;
      // std::cout << "Heading Source: TOP INS, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;

      }
      else if (m_inspva_heading_init)
      {
        m_best_heading_rad = m_dBOTTOM_INS_Heading;
        m_stdev_azimuth = m_bottom_stdev_azimuth;
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_INS;
      // std::cout << "Heading Source: BOTTOM INS, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;

      }
    }
    else
    {
      if (m_bestvel_bottom_valid)
      {
        m_best_heading_rad = m_dGPS_Heading; // Use only bottom
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_BESTVEL;
        // std::cout << "Heading Source: BOTTOM BESTVEL, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;
      }
      else if (m_bestvel_top_valid)
      {
        m_best_heading_rad = m_dGPS_TOP_Heading; // Use only top
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_BESTVEL;
        // std::cout << "Heading Source: TOP BESTVEL, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;
      }
      else if (m_top_heading2_valid)
      {
        m_best_heading_rad = m_top_heading2_heading_rad; // Use only heading2
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_HEADING2;
        // std::cout << "Heading Source: TOP HEADING2" << std::endl;
      }
      else if (m_heading2_valid)
      {
        m_best_heading_rad = m_heading2_heading_rad; // Use only heading2
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::NOVATEL_HEADING2;
        // std::cout << "Heading Source: BOTTOM HEADING2" << m_best_heading_rad * 180 / 3.14 << std::endl;
      }
      else if (m_use_kinematic_heading)
      {
        if (m_kinematic_heading_bottom_valid)
        {
          m_best_heading_rad = m_dGPS_Kinematic_Heading;
          m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::KINEMATIC;
          // std::cout << "Heading Source: BOTTOM KINEMATIC, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;
        }
        else if (m_kinematic_heading_top_valid)
        {
          m_best_heading_rad = m_d_TOP_GPS_Kinematic_Heading;
          m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::KINEMATIC;
          // std::cout << "Heading Source: TOP KINMATIC, Heading: " << m_best_heading_rad * 180 / 3.14 << std::endl;
        }
      }
      else if (!bBOTTOM_GPS && !bTOP_GPS && bVN1HeadingValid)
      {
        m_best_heading_rad = m_dVN1_Heading;
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::KINEMATIC;
        BestPosVN1.yaw = m_dVN1_Heading;
        m_heading_error = false;
      }
      else if (!bBOTTOM_GPS && !bTOP_GPS && bVN2HeadingValid)
      {
        m_best_heading_rad = m_dVN2_Heading;
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::KINEMATIC;
        BestPosVN2.yaw = m_dVN2_Heading;
        m_heading_error = false;
      }
      else if (!bBOTTOM_GPS && !bTOP_GPS && m_XSENS_Heading_valid)
      {
        m_best_heading_rad = m_XSENS_GPS_Heading;
        m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::KINEMATIC;
        BestPosXSENS.yaw = m_XSENS_GPS_Heading;
        m_heading_error = false;
      }
      // !!!!!!!!!!!!!!! WARN: keep last heading, no error! !!!!!!!!!!!!!!!!!!!!!!!!!!!
      // else {
      // m_heading_error = true;
      // }
      // !!!!!!!!!!!!!!! WARN: keep last heading, no error! !!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    // Heading Correction Validation Check using 1.Currecnt Estimated Yaw, 2.IMU angular rate Z
    // WARN:: Heading STDDEV for Smoothing, it can cause the delay and it can be propagated to the Perception side. 
    double predicted_yaw_change = ekf_.getXelement(IDX::WZ);
    correction_diff = fabs((correction_prev - m_best_heading_rad)*nif::common::constants::RAD2DEG);
    heading_diff = fabs((current_heading_est - m_best_heading_rad)*nif::common::constants::RAD2DEG);
    pred_heading_change = 6.0*fabs((predicted_yaw_change)*nif::common::constants::RAD2DEG);//100hz -> 20hz 5times + 20% margin
    if(pred_heading_change < 2.0)pred_heading_change = 2.0; // room for initialize(assuming heading_stdev=2.0)
    if(!bFirstHeadingConverged || !bInitConverged)
    {
      pred_heading_change = 15.0;
    }
    
    if(m_dVelolcity_X<0.1 && bForcedHeading)
    {
      heading_diff = 0.0;// no problem
      m_stdev_azimuth = 0.0;
      bForcedHeading = false;
    }
    else if((((fabs(correction_diff - 360.0)<pred_heading_change)||(fabs(correction_diff) < pred_heading_change))&& correction_diff!=0.0) || ((fabs(heading_diff - 360.0)<pred_heading_change)||(fabs(heading_diff) < pred_heading_change))) 
    {
      if(fabs(correction_diff - 360.0)<pred_heading_change){
        correction_diff = fabs(correction_diff - 360.0);
      }
      else if((fabs(correction_diff) < pred_heading_change))
      {
        correction_diff = correction_diff;
      }
      if(fabs(heading_diff - 360.0)<pred_heading_change){
        heading_diff = fabs(heading_diff - 360.0);
      }
      else if((fabs(heading_diff) < pred_heading_change))
      {
        heading_diff = heading_diff;
      }
      if(!bFirstHeadingConverged) 
      {
        bFirstHeadingConverged = true;
      }
    }
    else 
    // Just caneling the Heaidng correction maintain uncertainty low, but it's relatively unsafe 
    // No update can cause encreasing uncertainty, But it's relatively safe
    {
      // m_best_heading_rad = CurrentEstimated.yaw;
      update_pose = false;
      m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::HEADING_INVALID;
    }

    m_stdev_azimuth = fabs(0.2*heading_diff*nif::common::constants::DEG2RAD);

    // Update Other Heading Source with Best Heading
    m_dGPS_TOP_Heading = m_best_heading_rad; // Hyunwoo   - for arrow direction
    m_dGPS_Heading = m_best_heading_rad;     // Hyunwoo       - for arrow direction
    BestPosBottom.yaw = m_best_heading_rad;
    BestPosTop.yaw = m_best_heading_rad;
    m_dGPS_Kinematic_Heading = m_best_heading_rad;
    m_last_dGPS_Kinematic_Heading = m_best_heading_rad;
    m_d_TOP_GPS_Kinematic_Heading = m_best_heading_rad;
    m_last_d_TOP_GPS_Kinematic_Heading = m_best_heading_rad;
    m_heading_error = false;

    correction_yaw = m_best_heading_rad;

    m_localization_status.heading = correction_yaw;

    // TODO::when I block the update with the zone, there's no problem in straight, but uncertainty is increased and localization failure triggered.
    // if (zone_type == 40)
    // {
      // std::cout << "OVERPATH EXIST, NO UPDATE POSE" << std::endl;
    //   bottom_gps_update = false;
    //   top_gps_update = false;
    //   bottom_ins_update = false;
    //   top_ins_update = false;
    //   vn_ins_update = false;
    //   vn_gps1_update = false;
    //   vn_gps2_update = false;
    //   xsens_gps_update = false;
    // }
    // else
    // {
      if (bNominalMeasurement && (bottom_gps_update == true || top_gps_update == true) && update_pose)
      {
        measurementUpdatePose(update_pose_time, correction_x,
                              correction_y, correction_yaw);
        bottom_gps_update = false;
        top_gps_update = false;
        bottom_ins_update = false;
        top_ins_update = false;
        vn_ins_update = false;
        vn_gps1_update = false;
        vn_gps2_update = false;
        xsens_gps_update = false;
      }
      else if (!bNominalMeasurement && (vn_ins_update == true) && update_pose)
      { 
        measurementUpdatePose(update_pose_time, correction_x,
                              correction_y, correction_yaw);
        bottom_gps_update = false;
        top_gps_update = false;
        bottom_ins_update = false;
        top_ins_update = false;
        vn_ins_update = false;
        vn_gps1_update = false;
        vn_gps2_update = false;
        xsens_gps_update = false;
      }
    // }

    measurementUpdateTwist(update_twist_time, veh_vel_x, veh_yaw_rate);

    /* set current pose, twist */
    setCurrentResult();

    /* publish ekf result */
    publishEstimateResult();
    
    // THIS HEADING SOURCE IS QUITE ACCURATE EVEN IN THE GPS DEGRADED AREA
    // SO IT IS ALLOWED THE LONG TERM DEAD RECKONING.
    if(m_localization_status.heading_source==nif_msgs::msg::LocalizationStatus::VNINS && bVN_INS)
    {
      if (m_localization_status.uncertainty > 20.0) //large value
      {
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::UNCERTAINTY_TOO_HIGH;
        m_localization_status.status = "UNCERTAINTY_TOO_HIGH";
      }
    }
    else
    {
      if (m_localization_status.uncertainty > 3.0) //no meaning large value
      {
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::UNCERTAINTY_TOO_HIGH;
        m_localization_status.status = "UNCERTAINTY_TOO_HIGH";
      }
    }

    // if (m_localization_status.uncertainty > 20.0) //no meaning large value
    // {
    //   m_localization_status.localization_status_code =
    //       nif_msgs::msg::LocalizationStatus::UNCERTAINTY_TOO_HIGH;
    //   m_localization_status.status = "UNCERTAINTY_TOO_HIGH";
    // }
    // else if (m_heading_error)
    // {
    //   m_localization_status.localization_status_code =
    //       nif_msgs::msg::LocalizationStatus::HEADING_ERROR;
    //   m_localization_status.status = "HEADING_ERROR";
    // }
    pub_localization_status->publish(m_localization_status);
    this->setNodeStatus(node_status);
  }
  else
  {
    m_localization_status.status = "NO SENSOR INITIALIZED";
    m_localization_status.top_initialized = false;
    m_localization_status.bottom_initialized = false;
    m_localization_status.localization_status_code =
        nif_msgs::msg::LocalizationStatus::NO_SENSOR_INITIALIZED;

    auto node_status = nif::common::NODE_ERROR;
    RCLCPP_DEBUG(this->get_logger(), "Waiting for -[/novatel_bottom/bestpos]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/novatel_bottom/rawimux]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/raptor_dbw_interface/wheel_speed_report]");
  }
}

void AWLocalizationNode::showCurrentX()
{
  if (show_debug_info_)
  {
    Eigen::MatrixXd X(dim_x_, 1);
    ekf_.getLatestX(X);
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * setCurrentResult
 */
void AWLocalizationNode::setCurrentResult()
{
  current_ekf_pose_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;

  double roll = 0.;
  double pitch = 0.;
  double yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  q_tf.setRPY(roll, pitch, yaw);
  tf2::convert(q_tf, current_ekf_pose_.pose.orientation);

  current_ekf_twist_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

// HOW TO CORRECT HEADING IN EKF
// 1. Bottom GPS
// 2. Top GPS
// 3. BestVel
void AWLocalizationNode::BESTPOSCallback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
{

  this->bestpos_time_last_update = this->now();
  this->novatel_time_last_update = bestpos_time_last_update;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat;  //- dist(generator) * 1e-5;
  currentGPS.longitude = (double)msg->lon; //- dist(generator) * 1e-5;
  // Currently ignore altitude for the most part and just track x/y
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry ltp_odom;

  ltp_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  ltp_odom.header.stamp = this->now();
  ltp_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_dGPS_X = ltp_pt.x;
  m_dGPS_Y = -ltp_pt.y;

  double prior_noise = 0.0;
  if(use_reference_information)
  {
    prior_noise = 0.5*priorMeasurementUncertainty;
  }

  BestPosBottom.lat_noise = (double)msg->lat_stdev+prior_noise;
  BestPosBottom.lon_noise = (double)msg->lon_stdev+prior_noise;

  // TO MATCH THE BOTTOM NOVATEL TO THE TOP NOVATEL
  tf2::Transform transform_bottom_to_top;
  transform_bottom_to_top.setOrigin(tf2::Vector3(bottom_to_top_bias_x, bottom_to_top_bias_y, 0.0));
  tf2::Quaternion quat_bottom_to_top;
  quat_bottom_to_top.setRPY(0.0, 0.0, 0.0);
  transform_bottom_to_top.setRotation(quat_bottom_to_top);

  tf2::Transform transform_bottom_on_global;
  transform_bottom_on_global.setOrigin(tf2::Vector3(m_dGPS_X, m_dGPS_Y, -ltp_pt.z));
  tf2::Quaternion quat_bottom_on_global;
  quat_bottom_on_global.setRPY(0.0, 0, m_dGPS_Heading);
  transform_bottom_on_global.setRotation(quat_bottom_on_global);

  auto transform_bottom_to_top_sync = transform_bottom_on_global * transform_bottom_to_top;

  tf2::Quaternion q = transform_bottom_to_top_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_bottom_to_top_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  ltp_odom.pose.pose.position.x = transform_bottom_to_top_sync.getOrigin().x();
  ltp_odom.pose.pose.position.y = transform_bottom_to_top_sync.getOrigin().y();
  ltp_odom.pose.pose.position.z = transform_bottom_to_top_sync.getOrigin().z();
  ltp_odom.pose.pose.orientation.x = transform_bottom_to_top_sync.getRotation().x();
  ltp_odom.pose.pose.orientation.y = transform_bottom_to_top_sync.getRotation().y();
  ltp_odom.pose.pose.orientation.z = transform_bottom_to_top_sync.getRotation().z();
  ltp_odom.pose.pose.orientation.w = transform_bottom_to_top_sync.getRotation().w();
  ltp_odom.pose.covariance.at(0) = BestPosBottom.lat_noise;  //  x - x
  ltp_odom.pose.covariance.at(1) = BestPosBottom.lat_noise;  //  x - y
  ltp_odom.pose.covariance.at(6) = BestPosBottom.lon_noise;  //  y - x
  ltp_odom.pose.covariance.at(7) = BestPosBottom.lon_noise;  //  y - y
  ltp_odom.pose.covariance.at(35) = BestPosBottom.yaw_noise; //  yaw - yaw

  pub_bestpos_odometry->publish(ltp_odom);

  if((double)msg->lat == 0. && (double)msg->lon == 0.)
  // if (((double)msg->lat == 0. && (double)msg->lon == 0.) || msg->num_sol_multi_svs < 5 || msg->sol_age > 0.0)
  {
    bBOTTOM_GPS = false;
    return;
  }

  if (!m_bestvel_bottom_valid)
  { // calculating spent computing source, so we use Kinematic Heading only when bestvel is not valid, but, prev is problem when bestvel is malfunctioned
    double distance = sqrt(pow(m_dGPS_X - m_dGPS_X_prev, 2) + pow(m_dGPS_Y - m_dGPS_Y_prev, 2));

    if (distance > 0.5)
    {
      if (bFirstBottomKinematicHeading)
      { // at the first time use initial guess(or previous heading), because prev value is zero(or old one), kinematic Heading is inaccurate
        m_dGPS_Kinematic_Heading = m_best_heading_rad;
        m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
        bFirstBottomKinematicHeading = false;
      }
      else
      {
        m_dGPS_Kinematic_Heading = atan2(m_dGPS_Y - m_dGPS_Y_prev, m_dGPS_X - m_dGPS_X_prev);
        double kinematic_heading_diff_deg = (m_last_dGPS_Kinematic_Heading - m_dGPS_Kinematic_Heading) * nif::common::constants::RAD2DEG;
        if (kinematic_heading_diff_deg > 180.0)
          kinematic_heading_diff_deg -= 360.0;
        if (kinematic_heading_diff_deg < -180.0)
          kinematic_heading_diff_deg += 360.0;
        if (abs(kinematic_heading_diff_deg) > 15.0)
        {
          // std::cout << "bottom kinematic diff too high: " << kinematic_heading_diff_deg << std::endl;
          m_dGPS_Kinematic_Heading = m_last_dGPS_Kinematic_Heading;
        }
        else
        {
          m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
        }
        m_heading_initialized = true;
      }
      m_dGPS_X_prev = m_dGPS_X;
      m_dGPS_Y_prev = m_dGPS_Y;
    }
  }
  bottom_gps_update = true;
  bBOTTOM_GPS = true;

  BestPosBottom.x = transform_bottom_to_top_sync.getOrigin().x();
  BestPosBottom.y = transform_bottom_to_top_sync.getOrigin().y();

  BestPosBottom.novatel_bestpos_status = msg->pos_type.type;
  BestPosBottom.quality_code_ok = BestPosBottom.novatel_bestpos_status >= PositionOrVelocityType::L1_FLOAT;

  m_localization_status.pos_type_0 = msg->pos_type.type;

  ////////////////////////////////////////////////////////////////////////////////////
}

void AWLocalizationNode::TOPBESTPOSCallback(
    const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
{
  // test
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator(seed);
  // double mean = m_testnoise / 10.;
  // double stdev = (int)m_testnoise % 10;
  // std::normal_distribution<double> dist(mean, stdev);

  this->top_bestpos_time_last_update = this->now();
  this->novatel_time_last_update = top_bestpos_time_last_update;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat;  //+ dist(generator) * 1e-5;
  currentGPS.longitude = (double)msg->lon; //+ dist(generator) * 1e-5;
  // Currently ignore altitude for the most part and just track x/y
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry top_best_pos_odom;

  top_best_pos_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  top_best_pos_odom.header.stamp = this->now();
  top_best_pos_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_d_TOP_GPS_X = ltp_pt.x;
  m_d_TOP_GPS_Y = -ltp_pt.y;

  // TO MATCH THE TOP NOVATEL TO THE BOTTOM NOVATEL
  tf2::Transform transform_top_to_bottom;
  transform_top_to_bottom.setOrigin(tf2::Vector3(top_to_bottom_bias_x, top_to_bottom_bias_y, 0.0));
  tf2::Quaternion quat_top_to_bottom;
  quat_top_to_bottom.setRPY(0.0, 0.0, 0.0);
  transform_top_to_bottom.setRotation(quat_top_to_bottom);

  tf2::Transform transform_top_on_global;
  transform_top_on_global.setOrigin(tf2::Vector3(m_d_TOP_GPS_X, m_d_TOP_GPS_Y, -ltp_pt.z));
  tf2::Quaternion quat_top_on_global;
  quat_top_on_global.setRPY(m_dGPS_TOP_roll, 0, m_dGPS_TOP_Heading);
  transform_top_on_global.setRotation(quat_top_on_global);

  auto transform_top_to_bottom_sync = transform_top_on_global * transform_top_to_bottom;

  tf2::Quaternion q = transform_top_to_bottom_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_top_to_bottom_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  top_best_pos_odom.pose.pose.position.x = transform_top_to_bottom_sync.getOrigin().x();
  top_best_pos_odom.pose.pose.position.y = transform_top_to_bottom_sync.getOrigin().y();
  top_best_pos_odom.pose.pose.position.z = transform_top_to_bottom_sync.getOrigin().z();
  top_best_pos_odom.pose.pose.orientation.x = transform_top_to_bottom_sync.getRotation().x();
  top_best_pos_odom.pose.pose.orientation.y = transform_top_to_bottom_sync.getRotation().y();
  top_best_pos_odom.pose.pose.orientation.z = transform_top_to_bottom_sync.getRotation().z();
  top_best_pos_odom.pose.pose.orientation.w = transform_top_to_bottom_sync.getRotation().w();
  top_best_pos_odom.pose.covariance.at(0) = BestPosTop.lat_noise;  //  x - x
  top_best_pos_odom.pose.covariance.at(1) = BestPosTop.lat_noise;  //  x - y
  top_best_pos_odom.pose.covariance.at(6) = BestPosTop.lon_noise;  //  y - x
  top_best_pos_odom.pose.covariance.at(7) = BestPosTop.lon_noise;  //  y - y
  top_best_pos_odom.pose.covariance.at(35) = BestPosTop.yaw_noise; //  yaw - yaw

  pub_top_bestpos_odometry->publish(top_best_pos_odom);

  //////////////////////////////////////////////////////////////////////////////////

  BestPosTop.x = transform_top_to_bottom_sync.getOrigin().x();
  BestPosTop.y = transform_top_to_bottom_sync.getOrigin().y();

  double prior_noise = 0.0;
  if(use_reference_information)
  {
    prior_noise = 0.5*priorMeasurementUncertainty;
  }

  BestPosTop.lat_noise = (double)msg->lat_stdev+prior_noise;
  BestPosTop.lon_noise = (double)msg->lon_stdev+prior_noise;

  BestPosTop.novatel_bestpos_status = msg->pos_type.type;
  BestPosTop.quality_code_ok = BestPosTop.novatel_bestpos_status >= PositionOrVelocityType::L1_FLOAT;
  m_localization_status.pos_type_1 = msg->pos_type.type;

  // We assume not to be in the ocean at (0.0, 0.0)
  if((double)msg->lat == 0. && (double)msg->lon == 0.)
  // if (((double)msg->lat == 0. && (double)msg->lon == 0.) || msg->num_sol_multi_svs < 5 || msg->sol_age > 0.0)
  {
    bTOP_GPS = false;
    return;
  }

  if (!m_bestvel_top_valid)
  { // calculating spent computing source, so we use Kinematic Heading only when bestvel is not valid, but, prev is problem when bestvel is malfunctioned
    double distance = sqrt(pow(m_d_TOP_GPS_X - m_d_TOP_GPS_X_prev, 2) + pow(m_d_TOP_GPS_Y - m_d_TOP_GPS_Y_prev, 2));

    if (distance > 0.5)
    {
      if (bFirstTopKinematicHeading)
      { // at the first time use initial guess(or previous heading), because prev value is zero(or old one), kinematic Heading is inaccurate
        m_d_TOP_GPS_Kinematic_Heading = m_best_heading_rad;
        bFirstTopKinematicHeading = false;
      }
      else
      {
        m_d_TOP_GPS_Kinematic_Heading = atan2(m_d_TOP_GPS_Y - m_d_TOP_GPS_Y_prev, m_d_TOP_GPS_X - m_d_TOP_GPS_X_prev);
        double kinematic_heading_diff_deg = (m_last_d_TOP_GPS_Kinematic_Heading - m_d_TOP_GPS_Kinematic_Heading) * nif::common::constants::RAD2DEG;
        if (kinematic_heading_diff_deg > 180.0)
          kinematic_heading_diff_deg -= 360.0;
        if (kinematic_heading_diff_deg < -180.0)
          kinematic_heading_diff_deg += 360.0;
        if (abs(kinematic_heading_diff_deg) > 15.0)
        {
          // std::cout << "top kinematic diff too high: " << kinematic_heading_diff_deg << std::endl;
          m_d_TOP_GPS_Kinematic_Heading = m_last_d_TOP_GPS_Kinematic_Heading;
        }
        else
        {
          m_last_d_TOP_GPS_Kinematic_Heading = m_d_TOP_GPS_Kinematic_Heading;
        }
        m_heading_initialized = true;
      }
      m_d_TOP_GPS_X_prev = m_d_TOP_GPS_X;
      m_d_TOP_GPS_Y_prev = m_d_TOP_GPS_Y;
    }
  }
  bTOP_GPS = true;
  top_gps_update = true;
}

void AWLocalizationNode::XSENSPOSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  this->xsens_gps_time_last_update = this->now();
  this->secondary_time_last_update = xsens_gps_time_last_update;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;

  currentGPS.latitude = (double)msg->latitude;
  currentGPS.longitude = (double)msg->longitude;
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry xsens_odom;

  xsens_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  xsens_odom.header.stamp = this->now();
  xsens_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_XSENS_GPS_X = ltp_pt.x;
  m_XSENS_GPS_Y = -ltp_pt.y;

  double distance = sqrt(pow(m_XSENS_GPS_X - m_XSENS_GPS_X_prev, 2) + pow(m_XSENS_GPS_Y - m_XSENS_GPS_Y_prev, 2));

  if (distance > 0.5)
  {
    m_XSENS_GPS_Heading = atan2(m_XSENS_GPS_Y - m_XSENS_GPS_Y_prev, m_XSENS_GPS_X - m_XSENS_GPS_X_prev);

    m_XSENS_GPS_X_prev = m_XSENS_GPS_X;
    m_XSENS_GPS_Y_prev = m_XSENS_GPS_Y;
    if (m_dVelolcity_X > m_bestvel_heading_update_thres)
      m_XSENS_Heading_valid = true;
  }

  // We convert from NED to FLU
  xsens_odom.pose.pose.orientation.w = 1;
  xsens_odom.pose.pose.position.x = m_XSENS_GPS_X;
  xsens_odom.pose.pose.position.y = m_XSENS_GPS_Y;
  xsens_odom.pose.pose.position.z = 0;

  tf2::Quaternion quat_ekf;
  geometry_msgs::msg::Quaternion quat_ekf_msg;
  quat_ekf.setRPY(0, 0, m_XSENS_GPS_Heading);
  quat_ekf.normalize();
  quat_ekf_msg = tf2::toMsg(quat_ekf);
  xsens_odom.pose.pose.orientation = quat_ekf_msg;

  pub_xsens_odom->publish(xsens_odom);

  BestPosXSENS.x = m_XSENS_GPS_X;
  BestPosXSENS.y = m_XSENS_GPS_Y;
  // BestPosXSENS.quality_code_ok = BestPosBottom.novatel_bestpos_status >= PositionOrVelocityType::L1_FLOAT;

  if ((double)msg->latitude == 0. && (double)msg->longitude == 0.)
    return;

  xsens_gps_update = true;
  bXSENS_GPS = true;
}

void AWLocalizationNode::NovatelBottomINSCallback(const novatel_oem7_msgs::msg::INSPVA::ConstSharedPtr &inspva_msg,
    const novatel_oem7_msgs::msg::INSSTDEV::ConstSharedPtr &insstdev_msg)
{
  bottom_ins_time_last_update = this->now();
  novatel_ins_time_last_update = bottom_ins_time_last_update;

  double yaw_rad = (-inspva_msg->azimuth) * nif::common::constants::DEG2RAD;
  INSBottom.novatel_ins_status = inspva_msg->status.status;
  if (yaw_rad != 0.0 && yaw_rad != m_prevYaw && (INSBottom.novatel_ins_status == InertialSolutionStatus::INS_SOLUTION_GOOD))
  {
    m_inspva_heading_init = true; //same functionality as m_bestvel_top_valid, but, INSPVA is maintain it's heading estimation quite accurately after initialized once.
    bBOTTOMGPSHeading = true;
    m_bBOTTOMINS_initialized = true;
  }
  else
  {
    m_inspva_heading_init = false;
    bBOTTOMGPSHeading = false;
    m_bBOTTOMINS_initialized = false;
  }

  if (m_use_inspva_heading)
  {
    if (!m_inspva_heading_init) //INS  NOT initialized
    {
      m_prevYaw = yaw_rad;
      return;
    }
    if((double)inspva_msg->latitude == 0. && (double)inspva_msg->longitude == 0.)
    {
      m_bBOTTOMINS_initialized = false;
      return;
    }
    if(m_bBOTTOMINS_initialized) //INS initialized
    {
      m_dBOTTOM_INS_Heading = yaw_rad;
      m_dGPS_roll = inspva_msg->roll * nif::common::constants::DEG2RAD;
      INSBottom.yaw = yaw_rad;

      nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
      currentGPS.latitude = (double)inspva_msg->latitude;  //+ dist(generator) * 1e-5;
      currentGPS.longitude = (double)inspva_msg->longitude; //+ dist(generator) * 1e-5;
      // Currently ignore altitude for the most part and just track x/y
      currentGPS.altitude = 0.;

      nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
      conv_.geodetic2Ned(currentGPS, ltp_pt);

      nav_msgs::msg::Odometry bottom_ins_odom;

      bottom_ins_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
      bottom_ins_odom.header.stamp = this->now();
      bottom_ins_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

      m_d_BOTTOM_INS_X = ltp_pt.x;
      m_d_BOTTOM_INS_Y = -ltp_pt.y;

      // // TO MATCH THE TOP NOVATEL TO THE BOTTOM NOVATEL
      tf2::Transform transform_bottom_ins_to_center;
      transform_bottom_ins_to_center.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2::Quaternion quat_bottom_ins_to_center;
      quat_bottom_ins_to_center.setRPY(0.0, 0.0, 0.0);
      transform_bottom_ins_to_center.setRotation(quat_bottom_ins_to_center);

      tf2::Transform transform_bottom_ins_on_global;
      transform_bottom_ins_on_global.setOrigin(tf2::Vector3(m_d_BOTTOM_INS_X, m_d_BOTTOM_INS_Y, -ltp_pt.z));
      tf2::Quaternion quat_top_on_global;
      quat_top_on_global.setRPY(0.0, 0.0, m_dBOTTOM_INS_Heading);
      transform_bottom_ins_on_global.setRotation(quat_top_on_global);

      auto transform_bottom_ins_to_center_sync = transform_bottom_ins_on_global * transform_bottom_ins_to_center;

      tf2::Quaternion q = transform_bottom_ins_to_center_sync.getRotation();
      Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
      tf2::Vector3 v = transform_bottom_ins_to_center_sync.getOrigin();
      Eigen::Vector3f origin(v.x(), v.y(), v.z());

      INSBottom.x = transform_bottom_ins_to_center_sync.getOrigin().x();
      INSBottom.y = transform_bottom_ins_to_center_sync.getOrigin().y();

      m_bottom_stdev_azimuth = insstdev_msg->azimuth_stdev / 180 * M_PI;
      INSBottom.yaw_noise = m_bottom_stdev_azimuth;

      INSBottom.lat_noise = (double)insstdev_msg->latitude_stdev;
      INSBottom.lon_noise = (double)insstdev_msg->longitude_stdev;

      bottom_ins_odom.pose.pose.position.x = transform_bottom_ins_to_center_sync.getOrigin().x();
      bottom_ins_odom.pose.pose.position.y = transform_bottom_ins_to_center_sync.getOrigin().y();
      bottom_ins_odom.pose.pose.position.z = transform_bottom_ins_to_center_sync.getOrigin().z();
      bottom_ins_odom.pose.pose.orientation.x = transform_bottom_ins_to_center_sync.getRotation().x();
      bottom_ins_odom.pose.pose.orientation.y = transform_bottom_ins_to_center_sync.getRotation().y();
      bottom_ins_odom.pose.pose.orientation.z = transform_bottom_ins_to_center_sync.getRotation().z();
      bottom_ins_odom.pose.pose.orientation.w = transform_bottom_ins_to_center_sync.getRotation().w();
      bottom_ins_odom.pose.covariance.at(0) = INSBottom.lat_noise;  //  x - x
      bottom_ins_odom.pose.covariance.at(1) = INSBottom.lat_noise;  //  x - y
      bottom_ins_odom.pose.covariance.at(6) = INSBottom.lon_noise;  //  y - x
      bottom_ins_odom.pose.covariance.at(7) = INSBottom.lon_noise;  //  y - y
      bottom_ins_odom.pose.covariance.at(35) = INSBottom.yaw_noise; //  yaw - yaw

      pub_bottom_ins_odometry->publish(bottom_ins_odom);

      bottom_ins_update = true;
    }
  }
}

void AWLocalizationNode::NovatelTopINSCallback(const novatel_oem7_msgs::msg::INSPVA::ConstSharedPtr &inspva_msg,
    const novatel_oem7_msgs::msg::INSSTDEV::ConstSharedPtr &insstdev_msg)
{
  top_ins_time_last_update = this->now();
  novatel_ins_time_last_update = top_ins_time_last_update;

  double yaw_rad = (-inspva_msg->azimuth) * nif::common::constants::DEG2RAD;
  INSTop.novatel_ins_status = inspva_msg->status.status;
  if (yaw_rad != 0.0 && yaw_rad != m_prevYaw && (INSTop.novatel_ins_status == InertialSolutionStatus::INS_SOLUTION_GOOD))
  {
    m_top_inspva_heading_init = true; //same functionality as m_bestvel_top_valid, but, INSPVA is maintain it's heading estimation quite accurately after initialized once
    bTOPGPSHeading = true;
    m_bTOPINS_initialized = true;
  }else{
    m_top_inspva_heading_init = false;
    bTOPGPSHeading = false;
    m_bTOPINS_initialized = false;
  }

  if (m_use_inspva_heading)
  {
    if (!m_top_inspva_heading_init)
    {
      m_prevYaw = yaw_rad;
      return;
    }
    if((double)inspva_msg->latitude == 0. && (double)inspva_msg->longitude == 0.)
    {
      m_bTOPINS_initialized = false;
      return;
    }
    if(m_bTOPINS_initialized) //INS initialized
    {
      m_dTOP_INS_Heading = yaw_rad;
      m_dGPS_roll = inspva_msg->roll * nif::common::constants::DEG2RAD;
      INSTop.yaw = yaw_rad;

      nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
      currentGPS.latitude = (double)inspva_msg->latitude;  //+ dist(generator) * 1e-5;
      currentGPS.longitude = (double)inspva_msg->longitude; //+ dist(generator) * 1e-5;
      // Currently ignore altitude for the most part and just track x/y
      currentGPS.altitude = 0.;

      nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
      conv_.geodetic2Ned(currentGPS, ltp_pt);

      nav_msgs::msg::Odometry top_ins_odom;

      top_ins_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
      top_ins_odom.header.stamp = this->now();
      top_ins_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

      m_d_TOP_INS_X = ltp_pt.x;
      m_d_TOP_INS_Y = -ltp_pt.y;

      // // TO MATCH THE TOP NOVATEL TO THE BOTTOM NOVATEL
      tf2::Transform transform_top_ins_to_center;
      transform_top_ins_to_center.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2::Quaternion quat_top_ins_to_center;
      quat_top_ins_to_center.setRPY(0.0, 0.0, 0.0);
      transform_top_ins_to_center.setRotation(quat_top_ins_to_center);

      tf2::Transform transform_top_ins_on_global;
      transform_top_ins_on_global.setOrigin(tf2::Vector3(m_d_TOP_INS_X, m_d_TOP_INS_Y, -ltp_pt.z));
      tf2::Quaternion quat_top_on_global;
      quat_top_on_global.setRPY(0.0, 0.0, m_dTOP_INS_Heading);
      transform_top_ins_on_global.setRotation(quat_top_on_global);

      auto transform_top_ins_to_center_sync = transform_top_ins_on_global * transform_top_ins_to_center;

      tf2::Quaternion q = transform_top_ins_to_center_sync.getRotation();
      Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
      tf2::Vector3 v = transform_top_ins_to_center_sync.getOrigin();
      Eigen::Vector3f origin(v.x(), v.y(), v.z());

      INSTop.x = transform_top_ins_to_center_sync.getOrigin().x();
      INSTop.y = transform_top_ins_to_center_sync.getOrigin().y();

      m_bottom_stdev_azimuth = insstdev_msg->azimuth_stdev / 180 * M_PI;
      INSTop.yaw_noise = m_bottom_stdev_azimuth;

      INSTop.lat_noise = (double)insstdev_msg->latitude_stdev;
      INSTop.lon_noise = (double)insstdev_msg->longitude_stdev;

      top_ins_odom.pose.pose.position.x = transform_top_ins_to_center_sync.getOrigin().x();
      top_ins_odom.pose.pose.position.y = transform_top_ins_to_center_sync.getOrigin().y();
      top_ins_odom.pose.pose.position.z = transform_top_ins_to_center_sync.getOrigin().z();
      top_ins_odom.pose.pose.orientation.x = transform_top_ins_to_center_sync.getRotation().x();
      top_ins_odom.pose.pose.orientation.y = transform_top_ins_to_center_sync.getRotation().y();
      top_ins_odom.pose.pose.orientation.z = transform_top_ins_to_center_sync.getRotation().z();
      top_ins_odom.pose.pose.orientation.w = transform_top_ins_to_center_sync.getRotation().w();
      top_ins_odom.pose.covariance.at(0) = INSTop.lat_noise;  //  x - x
      top_ins_odom.pose.covariance.at(1) = INSTop.lat_noise;  //  x - y
      top_ins_odom.pose.covariance.at(6) = INSTop.lon_noise;  //  y - x
      top_ins_odom.pose.covariance.at(7) = INSTop.lon_noise;  //  y - y
      top_ins_odom.pose.covariance.at(35) = INSTop.yaw_noise; //  yaw - yaw

      pub_top_ins_odometry->publish(top_ins_odom);

      top_ins_update = true;
    }
  }
}

void AWLocalizationNode::BOTTOMBESTVELCallback(
    const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
{
  bestvel_time_last_update = std::move(msg->header.stamp);

  double yaw_rad = (-msg->trk_gnd) * nif::common::constants::DEG2RAD;

  if (m_top_inspva_heading_init)
  { // Always false if use_inspva_heading=false
    return;
  }

  // When we don't use INSPVA HEADING, we use bestvel heading
  // param [use_inspva_heading]
  //       True :  Use bestvel for only back-up solution
  //       False : Use bestvel for heading estimation
  if (m_dVelolcity_X > m_bestvel_heading_update_thres &&
      ((!m_inspva_heading_init && m_use_inspva_heading) || !m_use_inspva_heading)) // INSPVA HEADING BACK UP SOLUTITON
  {
    // m_dGPS_Heading =  this->m_heading_lowpass_gain * yaw_rad +
    //                     (1 - this->m_heading_lowpass_gain) * m_dGPS_Heading_prev;
    m_dGPS_Heading = yaw_rad;
    m_dGPS_Heading_prev = m_dGPS_Heading;
    BestPosBottom.yaw = yaw_rad;
    bBOTTOMGPSHeading = true;
    m_bestvel_bottom_valid = true;
    m_heading_initialized = true;
    m_use_kinematic_heading = false;
    bFirstBottomKinematicHeading = true;
    bFirstXsensKinematicHeading = true;
  }
  else if (m_dVelolcity_X < m_bestvel_heading_update_thres &&
           ((!m_inspva_heading_init && m_use_inspva_heading) || !m_use_inspva_heading))
  {

    if (m_bestvel_bottom_valid)
      bFirstBottomKinematicHeading = true;
    m_use_kinematic_heading = true;
    if (m_bestvel_bottom_valid)
      bFirstBottomKinematicHeading = true;
    m_bestvel_bottom_valid = false;
  }

  else
  {
    if (m_bestvel_bottom_valid)
      bFirstBottomKinematicHeading = true;
    m_bestvel_bottom_valid = false;
  }

  // TODO !WARNING! What happen if velocity < threshold?

  if (bUseBestVelForSpeed)
  {
    m_dVelolcity_X = msg->hor_speed;
  }
}

void AWLocalizationNode::TOPBESTVELCallback(
    const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
{
  top_bestvel_time_last_update = std::move(msg->header.stamp);
  double yaw_rad = (-msg->trk_gnd) * nif::common::constants::DEG2RAD;

  if (m_top_inspva_heading_init)
  { // Always false if use_inspva_heading=false
    return;
  }

  // When we don't use INSPVA HEADING, we use bestvel heading
  // param [use_inspva_heading]
  //       True :  Use bestvel for only back-up solution
  //       False : Use bestvel for heading estimation
  if (m_dVelolcity_X > m_bestvel_heading_update_thres &&
      ((!m_inspva_heading_init && m_use_inspva_heading) || !m_use_inspva_heading)) // INSPVA HEADING BACK UP SOLUTITON
  {
    // m_dGPS_TOP_Heading =  this->m_heading_lowpass_gain * yaw_rad +
    //                     (1 - this->m_heading_lowpass_gain) * m_dGPS_TOP_Heading_prev;
    m_dGPS_TOP_Heading = yaw_rad;
    m_dGPS_TOP_Heading_prev = m_dGPS_TOP_Heading;
    BestPosTop.yaw = yaw_rad;
    bTOPGPSHeading = true;
    m_bestvel_top_valid = true;
    m_heading_initialized = true;
    m_use_kinematic_heading = false;
    bFirstTopKinematicHeading = true;
    bFirstXsensKinematicHeading = true;
  }
  else if (m_dVelolcity_X < m_bestvel_heading_update_thres &&
           ((!m_inspva_heading_init && m_use_inspva_heading) || !m_use_inspva_heading))
  {
    m_use_kinematic_heading = true;
    if (m_bestvel_top_valid)
      bFirstTopKinematicHeading = true;
    m_bestvel_top_valid = false;
  }
  else
  {
    // Neither HEADING2 nor BESTVEL are valid
    if (m_bestvel_top_valid)
      bFirstTopKinematicHeading = true;
    m_bestvel_top_valid = false;
  }
  // TODO !WARNING! What happen if velocity < threshold?

  if (bUseBestVelForSpeed)
  {
    m_dVelolcity_X = msg->hor_speed;
  }
}

void AWLocalizationNode::HEADING2Callback(
    const novatel_oem7_msgs::msg::HEADING2::SharedPtr msg)
{
  heading2_time_last_update = std::move(msg->header.stamp);
  double yaw_rad = (-msg->heading) * nif::common::constants::DEG2RAD;

  if (m_top_inspva_heading_init)
  { // Always false if use_inspva_heading=false
    return;
  }

  // // HEADING2 used only for initial guess (if stddev is low enough)
  if (m_dVelolcity_X <= m_bestvel_heading_update_thres && !m_use_inspva_heading &&
      msg->heading_stdev > 0. && msg->heading_stdev < 2.0)
  {

    m_heading2_heading_rad = yaw_rad + this->m_heading_heading2_offset_rad;
    m_top_heading2_heading_rad = m_heading2_heading_rad;

    m_heading_initialized = true;
    m_heading2_valid = true;
    if (m_bUseHeading2)
    {
      m_dGPS_Kinematic_Heading = m_heading2_heading_rad;
      m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
      m_d_TOP_GPS_Kinematic_Heading = m_heading2_heading_rad;
      m_last_d_TOP_GPS_Kinematic_Heading = m_d_TOP_GPS_Kinematic_Heading;
      m_dVN1_Heading = m_heading2_heading_rad;
      m_dVN2_Heading = m_heading2_heading_rad;
      m_XSENS_GPS_Heading = m_heading2_heading_rad;
      m_bUseHeading2 = false;
    }
  }
  else
  {
    m_heading2_valid = false;
  }
}

void AWLocalizationNode::TOPHEADING2Callback(
    const novatel_oem7_msgs::msg::HEADING2::SharedPtr msg)
{
  top_heading2_time_last_update = std::move(msg->header.stamp);
  double yaw_rad = (-msg->heading) * nif::common::constants::DEG2RAD;

  if (m_top_inspva_heading_init)
  { // Always false if use_inspva_heading=false
    return;
  }

  // // HEADING2 used only for initial guess (if stddev is low enough)
  if (m_dVelolcity_X <= m_bestvel_heading_update_thres && !m_use_inspva_heading &&
      msg->heading_stdev > 0. && msg->heading_stdev < 2.0)
  {

    // WARN: valid only for specific installation
    // TODO: make this param
    // @DEBUG verify with bag
    m_top_heading2_heading_rad = yaw_rad + this->m_heading_heading2_offset_rad;
    m_heading2_heading_rad = m_top_heading2_heading_rad;

    m_heading_initialized = true;
    m_top_heading2_valid = true;
    if (m_bUseHeading2)
    {
      m_dGPS_Kinematic_Heading = m_top_heading2_heading_rad;
      m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
      m_d_TOP_GPS_Kinematic_Heading = m_top_heading2_heading_rad;
      m_last_d_TOP_GPS_Kinematic_Heading = m_d_TOP_GPS_Kinematic_Heading;
      m_dVN1_Heading = m_top_heading2_heading_rad;
      m_dVN2_Heading = m_top_heading2_heading_rad;
      m_XSENS_GPS_Heading = m_top_heading2_heading_rad;
      m_bUseHeading2 = false;
    }
  }
  else
  {
    m_top_heading2_valid = false;
  }

  // // @DEBUG this disables top heading2
  // m_top_heading2_valid = false;
}

void AWLocalizationNode::MessageFilteringCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg)
{

  this->imu_time_last_update = this->now();
  m_dIMU_yaw_rate = imu_msg->angular_velocity.z;

  if (!this->bUseBestVelForSpeed)
  { // TODO:: bUseBestVelForSpeed name change or initial parameter change(true to false)
    m_dVelolcity_X = (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
                     nif::common::constants::KPH2MS * 0.9826;
  }

  if (!bImuFirstCall)
  {
    bImuFirstCall = true;
  }
}

void AWLocalizationNode::TOPMessageFilteringCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr &wheel_speed_msg)
{
  this->top_imu_time_last_update = this->now();
  m_dTOP_IMU_yaw_rate = imu_msg->angular_velocity.z;
  if (!bTOPImuFirstCall)
  {
    bTOPImuFirstCall = true;
  }
  if (bImuFirstCall)
    return;

  if (!this->bUseBestVelForSpeed)
  { // TODO:: bUseBestVelForSpeed name change or initial parameter change(true to false)
    m_dVelolcity_X = (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
                     nif::common::constants::KPH2MS * 0.9826;
  }
}
// FOR VECTORNAV
void AWLocalizationNode::VNMessageFilteringCallback(
    const vectornav_msgs::msg::CommonGroup::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg)
{
  this->vn_imu_time_last_update = this->now();
  this->vn_ins_time_last_update = vn_imu_time_last_update;
  this->secondary_time_last_update = vn_imu_time_last_update;
  m_localization_status.vectornav_connection = true;

  if(imu_msg->insstatus.mode>=InsStatus::MODE_TRACKING && (BestPosVN1.quality_code_ok))
  {
    // std::cout<<"vn_ins initialized"<<std::endl;
    m_bVNINS_initialized = true;
  }
  else
  {
    m_bVNINS_initialized = false;
    // std::cout<<"vn_ins not initialized"<<std::endl;

  }

  m_dVN_roll = imu_msg->yawpitchroll.z;
  double vn_yaw = imu_msg->yawpitchroll.x;
  m_dVN_INS_heading = -vn_yaw  * nif::common::constants::DEG2RAD; 
  INSVectorNav.yaw = m_dVN_INS_heading;

  m_dVN_yaw_rate = -imu_msg->imu_rate.z; // NEED TO CHANGE AFTER CHECKING THE SENSOR DATA
  if (!bVNIMUFirstCall)
  {
    bVNIMUFirstCall = true;
  }

  if (!this->bUseBestVelForSpeed)
  { // TODO:: bUseBestVelForSpeed name change or initial parameter change(true to false)
    m_dVelolcity_X =
        (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
        nif::common::constants::KPH2MS * 0.9826;
  }

  if ((double)imu_msg->position.x == 0. && (double)imu_msg->position.y == 0.)
    return;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)imu_msg->position.x;
  currentGPS.longitude = (double)imu_msg->position.y;
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry vn_ins_odom;

  vn_ins_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  vn_ins_odom.header.stamp = this->now();
  vn_ins_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_d_VN_INS_X = ltp_pt.x;
  m_d_VN_INS_Y = -ltp_pt.y;

  vn_ins_odom.pose.pose.orientation.w = 1;
  vn_ins_odom.pose.pose.position.x = m_d_VN_INS_X;
  vn_ins_odom.pose.pose.position.y = m_d_VN_INS_Y;
  vn_ins_odom.pose.pose.position.z = 0;

  // TO MATCH THE FRONT TO THE CENTER
  tf2::Transform transform_vectornav_ins_to_center;
  transform_vectornav_ins_to_center.setOrigin(tf2::Vector3(0.0, -0.10, 0.0));
  tf2::Quaternion quat_front_to_center;
  quat_front_to_center.setRPY(0.0, 0.0, 0.0);
  transform_vectornav_ins_to_center.setRotation(quat_front_to_center);

  tf2::Transform transform_front_on_global;
  transform_front_on_global.setOrigin(tf2::Vector3(m_d_VN_INS_X, m_d_VN_INS_Y, -ltp_pt.z));
  tf2::Quaternion quat_front_on_global;
  quat_front_on_global.setRPY(0.0, 0.0, m_dVN_INS_heading);
  transform_front_on_global.setRotation(quat_front_on_global);

  auto transform_vectornav_ins_to_center_sync = transform_front_on_global * transform_vectornav_ins_to_center;

  tf2::Quaternion q = transform_vectornav_ins_to_center_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_vectornav_ins_to_center_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  INSVectorNav.x = transform_vectornav_ins_to_center_sync.getOrigin().x();
  INSVectorNav.y = transform_vectornav_ins_to_center_sync.getOrigin().y();

  if(!bPosUncertainty)
  {
    INSVectorNav.lat_noise = 0.4;
    INSVectorNav.lon_noise = 0.4;
  }

  vn_ins_odom.pose.pose.position.x = transform_vectornav_ins_to_center_sync.getOrigin().x();
  vn_ins_odom.pose.pose.position.y = transform_vectornav_ins_to_center_sync.getOrigin().y();
  vn_ins_odom.pose.pose.position.z = transform_vectornav_ins_to_center_sync.getOrigin().z();
  vn_ins_odom.pose.pose.orientation.x = transform_vectornav_ins_to_center_sync.getRotation().x();
  vn_ins_odom.pose.pose.orientation.y = transform_vectornav_ins_to_center_sync.getRotation().y();
  vn_ins_odom.pose.pose.orientation.z = transform_vectornav_ins_to_center_sync.getRotation().z();
  vn_ins_odom.pose.pose.orientation.w = transform_vectornav_ins_to_center_sync.getRotation().w();
  vn_ins_odom.pose.covariance.at(0) = BestPosBottom.lat_noise;  //  x - x
  vn_ins_odom.pose.covariance.at(1) = BestPosBottom.lat_noise;  //  x - y
  vn_ins_odom.pose.covariance.at(6) = BestPosBottom.lon_noise;  //  y - x
  vn_ins_odom.pose.covariance.at(7) = BestPosBottom.lon_noise;  //  y - y
  vn_ins_odom.pose.covariance.at(35) = BestPosBottom.yaw_noise; //  yaw - yaw

  pub_vn_ins_odometry->publish(vn_ins_odom);

  bVN_INS = true;
  vn_ins_update = true;
}

// FOR XSENS
void AWLocalizationNode::XSENSMessageFilteringCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg)
{

  this->xsens_imu_time_last_update = this->now();
  m_dXSENS_yaw_rate = imu_msg->angular_velocity.z;
  ; // NEED TO CHANGE AFTER CHECKING THE SENSOR DATA
  if (!bXSENSIMUFirstCall)
  {
    bXSENSIMUFirstCall = true;
  }
  if (bImuFirstCall || bTOPImuFirstCall)
    return;

  if (!this->bUseBestVelForSpeed)
  { // TODO:: bUseBestVelForSpeed name change or initial parameter change(true to false)
    m_dVelolcity_X =
        (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
        nif::common::constants::KPH2MS * 0.9826;
  }
}

void AWLocalizationNode::IMUCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{

  if (!bUseBestVelForSpeed)
  {
    return;
  }

  this->imu_time_last_update = this->now();
  m_dIMU_yaw_rate = msg->angular_velocity.z;

  if (!bImuFirstCall)
  {
    bImuFirstCall = true;
    return;
  }
}
void AWLocalizationNode::IMUCallback2(
    const novatel_oem7_msgs::msg::RAWIMU::SharedPtr msg)
{

  if (!bUseBestVelForSpeed)
  {
    return;
  }

  this->imu_time_last_update = this->now();
  m_dIMU_yaw_rate = msg->angular_velocity.z;

  if (!bImuFirstCall)
  {
    bImuFirstCall = true;
    return;
  }
}

void AWLocalizationNode::TestCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
  m_testnoise = msg->data;
}

// FOR VECTORNAV
// TODO:: bVNINSHeadingValid logic and thres adjustment
void AWLocalizationNode::VNATTITUDECallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg)
{
  double vn_yaw_uncertainty = msg->ypru.x;
  m_dVN_INS_Heading_Uncertainty = vn_yaw_uncertainty / 180 * M_PI;
  INSVectorNav.yaw_noise = m_dVN_INS_Heading_Uncertainty;
  if(vn_yaw_uncertainty < 0.33 && vn_yaw_uncertainty !=0.0)
  {
    bVNINSHeadingValid = true;
  }
  else
  {
    bVNINSHeadingValid = false;
  }
}

void AWLocalizationNode::VNGPS1Callback(
    const vectornav_msgs::msg::GpsGroup::SharedPtr msg)
{
  this->vn_gps1_time_last_update = this->now();
  this->secondary_time_last_update = vn_gps1_time_last_update;

  // We assume not to be in the ocean at (0.0, 0.0)
  if ((double)msg->poslla.x == 0. && (double)msg->poslla.y == 0.)
    return;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->poslla.x;
  currentGPS.longitude = (double)msg->poslla.y;
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry vn1_odom;

  vn1_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  vn1_odom.header.stamp = this->now();
  vn1_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_d_VN_GPS1_X = ltp_pt.x;
  m_d_VN_GPS1_Y = -ltp_pt.y;

  vn1_odom.pose.pose.orientation.w = 1;
  vn1_odom.pose.pose.position.x = m_d_VN_GPS1_X;
  vn1_odom.pose.pose.position.y = m_d_VN_GPS1_Y;
  vn1_odom.pose.pose.position.z = 0;

  double posu_x = msg->posu.x;
  double posu_y = msg->posu.y;

  double prior_noise = 0.0;

  if(use_reference_information) prior_noise = priorMeasurementUncertainty;

  BestPosVN1.lat_noise = posu_x + prior_noise;
  BestPosVN1.lon_noise = posu_y + prior_noise;



  double distance = sqrt(pow(m_d_VN_GPS1_X - m_d_VN_GPS1_X_prev, 2) + pow(m_d_VN_GPS1_Y - m_d_VN_GPS1_Y_prev, 2));

  if (distance > 0.5)
  {
    m_dVN1_Heading = atan2(m_d_VN_GPS1_Y - m_d_VN_GPS1_Y_prev, m_d_VN_GPS1_X - m_d_VN_GPS1_X_prev);

    m_d_VN_GPS1_X_prev = m_d_VN_GPS1_X;
    m_d_VN_GPS1_Y_prev = m_d_VN_GPS1_Y;
    if (m_dVelolcity_X > m_bestvel_heading_update_thres)
      bVN1HeadingValid = true;
  }

  // TO MATCH THE FRONT TO THE CENTER
  tf2::Transform transform_front_to_center;
  transform_front_to_center.setOrigin(tf2::Vector3(bottom_to_top_bias_x, 0.0, 0.0));
  tf2::Quaternion quat_front_to_center;
  quat_front_to_center.setRPY(0.0, 0.0, 0.0);
  transform_front_to_center.setRotation(quat_front_to_center);

  tf2::Transform transform_front_on_global;
  transform_front_on_global.setOrigin(tf2::Vector3(m_d_VN_GPS1_X, m_d_VN_GPS1_Y, -ltp_pt.z));
  tf2::Quaternion quat_front_on_global;
  quat_front_on_global.setRPY(0.0, 0.0, m_dVN_INS_heading);
  transform_front_on_global.setRotation(quat_front_on_global);

  auto transform_front_to_center_sync = transform_front_on_global * transform_front_to_center;

  tf2::Quaternion q = transform_front_to_center_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_front_to_center_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  vn1_odom.pose.pose.position.x = transform_front_to_center_sync.getOrigin().x();
  vn1_odom.pose.pose.position.y = transform_front_to_center_sync.getOrigin().y();
  vn1_odom.pose.pose.position.z = transform_front_to_center_sync.getOrigin().z();
  vn1_odom.pose.pose.orientation.x = transform_front_to_center_sync.getRotation().x();
  vn1_odom.pose.pose.orientation.y = transform_front_to_center_sync.getRotation().y();
  vn1_odom.pose.pose.orientation.z = transform_front_to_center_sync.getRotation().z();
  vn1_odom.pose.pose.orientation.w = transform_front_to_center_sync.getRotation().w();
  vn1_odom.pose.covariance.at(0) = BestPosBottom.lat_noise;  //  x - x
  vn1_odom.pose.covariance.at(1) = BestPosBottom.lat_noise;  //  x - y
  vn1_odom.pose.covariance.at(6) = BestPosBottom.lon_noise;  //  y - x
  vn1_odom.pose.covariance.at(7) = BestPosBottom.lon_noise;  //  y - y
  vn1_odom.pose.covariance.at(35) = BestPosBottom.yaw_noise; //  yaw - yaw

  pub_vn1_odometry->publish(vn1_odom);

  BestPosVN1.x = m_d_VN_GPS1_X;
  BestPosVN1.y = m_d_VN_GPS1_Y;

  BestPosVN1.novatel_bestpos_status = 16;
  BestPosVN1.quality_code_ok = msg->fix >= GpsGroup::RTK_FLOAT;
  // std::cout<<"BestPosVN1.quality_code_ok:"<<BestPosVN1.quality_code_ok<<std::endl;
  // std::cout<<"BestPosVN1 fix condition:"<<msg->fix<<std::endl;
  // std::cout<<"GpsGroup::RTK_FLOAT:"<<GpsGroup::RTK_FLOAT<<std::endl;

  if (!bBOTTOM_GPS && !bTOP_GPS)
  {
    m_localization_status.pos_type_0 = 0;
    m_localization_status.pos_type_1 = 0;
  }

  bVN_GPS1 = true;
  vn_gps1_update = true;
}

void AWLocalizationNode::VNGPS2Callback(
    const vectornav_msgs::msg::GpsGroup::SharedPtr msg)
{

  this->vn_gps2_time_last_update = this->now();
  this->secondary_time_last_update = vn_gps2_time_last_update;

  // We assume not to be in the ocean at (0.0, 0.0)
  if ((double)msg->poslla.x == 0. && (double)msg->poslla.y == 0.)
    return;

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->poslla.x;
  currentGPS.longitude = (double)msg->poslla.y;
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry vn2_odom;

  vn2_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  vn2_odom.header.stamp = this->now();
  vn2_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_d_VN_GPS2_X = ltp_pt.x;
  m_d_VN_GPS2_Y = -ltp_pt.y;

  vn2_odom.pose.pose.orientation.w = 1;
  vn2_odom.pose.pose.position.x = m_d_VN_GPS2_X;
  vn2_odom.pose.pose.position.y = m_d_VN_GPS2_Y;
  vn2_odom.pose.pose.position.z = 0;

  double posu_x = msg->posu.x;
  double posu_y = msg->posu.y;

  double prior_noise = 0.0;

  if(use_reference_information) prior_noise = priorMeasurementUncertainty;

  BestPosVN2.lat_noise = posu_x + prior_noise;
  BestPosVN2.lon_noise = posu_y + prior_noise;

  double distance = sqrt(pow(m_d_VN_GPS2_X - m_d_VN_GPS2_X_prev, 2) + pow(m_d_VN_GPS2_Y - m_d_VN_GPS2_Y_prev, 2));

  if (distance > 0.5)
  {
    m_dVN2_Heading = atan2(m_d_VN_GPS2_Y - m_d_VN_GPS2_Y_prev, m_d_VN_GPS2_X - m_d_VN_GPS2_X_prev);

    m_d_VN_GPS2_X_prev = m_d_VN_GPS2_X;
    m_d_VN_GPS2_Y_prev = m_d_VN_GPS2_Y;
    if (m_dVelolcity_X > m_bestvel_heading_update_thres)
      bVN2HeadingValid = true;
  }

  // TO MATCH THE FRONT TO THE CENTER
  tf2::Transform transform_front_to_center;
  transform_front_to_center.setOrigin(tf2::Vector3(1.94 + bottom_to_top_bias_x, 0.0, 0.0));
  tf2::Quaternion quat_front_to_center;
  quat_front_to_center.setRPY(0.0, 0.0, 0.0);
  transform_front_to_center.setRotation(quat_front_to_center);

  tf2::Transform transform_front_on_global;
  transform_front_on_global.setOrigin(tf2::Vector3(m_d_VN_GPS2_X, m_d_VN_GPS2_Y, -ltp_pt.z));
  tf2::Quaternion quat_front_on_global;
  quat_front_on_global.setRPY(0.0, 0.0, m_dVN_INS_heading);
  transform_front_on_global.setRotation(quat_front_on_global);

  auto transform_front_to_center_sync = transform_front_on_global * transform_front_to_center;

  tf2::Quaternion q = transform_front_to_center_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_front_to_center_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  vn2_odom.pose.pose.position.x = transform_front_to_center_sync.getOrigin().x();
  vn2_odom.pose.pose.position.y = transform_front_to_center_sync.getOrigin().y();
  vn2_odom.pose.pose.position.z = transform_front_to_center_sync.getOrigin().z();
  vn2_odom.pose.pose.orientation.x = transform_front_to_center_sync.getRotation().x();
  vn2_odom.pose.pose.orientation.y = transform_front_to_center_sync.getRotation().y();
  vn2_odom.pose.pose.orientation.z = transform_front_to_center_sync.getRotation().z();
  vn2_odom.pose.pose.orientation.w = transform_front_to_center_sync.getRotation().w();
  vn2_odom.pose.covariance.at(0) = BestPosBottom.lat_noise;  //  x - x
  vn2_odom.pose.covariance.at(1) = BestPosBottom.lat_noise;  //  x - y
  vn2_odom.pose.covariance.at(6) = BestPosBottom.lon_noise;  //  y - x
  vn2_odom.pose.covariance.at(7) = BestPosBottom.lon_noise;  //  y - y
  vn2_odom.pose.covariance.at(35) = BestPosBottom.yaw_noise; //  yaw - yaw

  pub_vn2_odometry->publish(vn2_odom);

  BestPosVN2.x = m_d_VN_GPS2_X;
  BestPosVN2.y = m_d_VN_GPS2_Y;

  BestPosVN2.novatel_bestpos_status = 16;
  BestPosVN2.quality_code_ok = msg->fix >= GpsGroup::RTK_FLOAT;
  // std::cout<<"BestPosVN2.quality_code_ok:"<<BestPosVN2.quality_code_ok<<std::endl;
  // std::cout<<"BestPosVN2 fix condition:"<<msg->fix<<std::endl;
  // std::cout<<"GpsGroup::RTK_FLOAT:"<<GpsGroup::RTK_FLOAT<<std::endl;

  if (!bBOTTOM_GPS && !bTOP_GPS)
  {
    m_localization_status.pos_type_0 = 0;
    m_localization_status.pos_type_1 = 0;
  }

  bVN_GPS2 = true;
  vn_gps2_update = true;
}

void AWLocalizationNode::SystemStatusCallback(const nif::common::msgs::SystemStatus::SharedPtr msg)
{
  zone_id = msg->mission_status.zone_status.zone_id;
  zone_type = msg->mission_status.zone_status.zone_type;
  mission_number = msg->mission_status.mission_status_code;
}

void AWLocalizationNode::VNINSCallback(
    const vectornav_msgs::msg::InsGroup::SharedPtr msg)
{
  double vn_ins_posu = msg->posu;

  if(vn_ins_posu == 0.0)
  {
    vn_ins_posu = 100000.0;
    return;
  }

  INSVectorNav.lat_noise = vn_ins_posu/sqrt(2);
  INSVectorNav.lon_noise = vn_ins_posu/sqrt(2);

  if(!bPosUncertainty) bPosUncertainty = true;

}

/*
 * initEKF
 */
void AWLocalizationNode::initEKF()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15; //  for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                           //  for yaw
  P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;                         //  for yaw bias
  P(IDX::VX, IDX::VX) = 1000.0;                                           //  for vx
  P(IDX::WZ, IDX::WZ) = 50.0;                                             //  for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void AWLocalizationNode::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k
   *
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  Eigen::MatrixXd X_next(dim_x_, 1); //  predicted state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);

  const int d_dim_x = dim_x_ex_ - dim_x_;
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = ekf_dt_;

  /* Update for latest state */
  X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt; //  dx = v * cos(yaw)
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt; //  dy = v * sin(yaw)
  X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                   //  dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  X_next(IDX::YAW) = std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  const double dvx = std::sqrt(P_curr(IDX::VX, IDX::VX));
  const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  if (dvx < 10.0 && dyaw < 1.0)
  {
    //  auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

    /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
     dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
    Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2); //  coeff of deviation of vx & yaw
    Jp << cos(yaw), -vx * sin(yaw), sin(yaw), vx * cos(yaw);
    Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2); //  cov of vx and yaw

    Q_vx_yaw(0, 0) = P_curr(IDX::VX, IDX::VX) * dt;       //  covariance of vx - vx
    Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt;     //  covariance of yaw - yaw
    Q_vx_yaw(0, 1) = P_curr(IDX::VX, IDX::YAW) * dt;      //  covariance of vx - yaw
    Q_vx_yaw(1, 0) = P_curr(IDX::YAW, IDX::VX) * dt;      //  covariance of yaw - vx
    Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose(); //  for pos_x & pos_y
  }
  else
  {
    //  vx & vy is not converged yet, set constant value.
    Q(IDX::X, IDX::X) = 0.05;
    Q(IDX::Y, IDX::Y) = 0.05;
  }

  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;        //  for yaw
  Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_; //  for yaw bias
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;           //  for vx
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;           //  for wz

  ekf_.predictWithDelay(X_next, A, Q);

  //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void AWLocalizationNode::measurementUpdatePose(rclcpp::Time measurement_time_,
                                               const double &corr_x_,
                                               const double &corr_y_,
                                               const double &corr_yaw_)
{
  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3; //  pos_x, pos_y, yaw, depending on Pose output
  auto t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - measurement_time_).seconds() +
                      (t_curr - measurement_time_).nanoseconds() * 1e-9 +
                      pose_additional_delay_;
  if (delay_time < 0.0)
  {
    delay_time = 0.0;
    //  ROS_WARN_DELAYED_THROTTLE(1.0, "Pose time stamp is inappropriate, set
    //  delay to 0[s]. delay = %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Pose delay exceeds the compensation limit, ignored. delay: %f[s], "
                         " limit= extend_state_step * ekf_dt : %f [s]",
                         delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  // DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  const double yaw_curr =
      ekf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
  double yaw = corr_yaw_;
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error =
      normalizeYaw(yaw - ekf_yaw); //  normalize the error not to exceed 2 pi

  yaw = yaw_error + ekf_yaw;

  Eigen::MatrixXd y(dim_y, 1);
  Eigen::MatrixXd y_correction(2, 1);

  y << corr_x_, corr_y_, yaw;
  y_correction << corr_x_, corr_y_;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  Eigen::MatrixXd y_prediction(2, 1);

  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
      ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;

  y_prediction << ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y);

  Eigen::MatrixXd P_curr, P_y, P_2by2;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  P_2by2 = P_curr.block(0, 0, 2, 2);

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;   //  for pos x
  C(1, IDX::Y) = 1.0;   //  for pos y
  C(2, IDX::YAW) = 1.0; //  for yaw

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_)
  {
    R(0, 0) = current_pose_covariance_.at(0);  //  x - x
    R(0, 1) = current_pose_covariance_.at(1);  //  x - y
    R(0, 2) = current_pose_covariance_.at(5);  //  x - yaw
    R(1, 0) = current_pose_covariance_.at(6);  //  y - x
    R(1, 1) = current_pose_covariance_.at(7);  //  y - y
    R(1, 2) = current_pose_covariance_.at(11); //  y - yaw
    R(2, 0) = current_pose_covariance_.at(30); //  yaw - x
    R(2, 1) = current_pose_covariance_.at(31); //  yaw - y
    R(2, 2) = current_pose_covariance_.at(35); //  yaw - yaw
  }
  else
  {
    const double ekf_yaw = ekf_.getXelement(IDX::YAW);
    const double vx = ekf_.getXelement(IDX::VX);
    const double wz = ekf_.getXelement(IDX::WZ);

    const double cov_pos_x =
        std::pow(pose_measure_uncertainty_time_ * vx * cos(-ekf_yaw), 2.0);
    const double cov_pos_y =
        std::pow(pose_measure_uncertainty_time_ * vx * sin(-ekf_yaw), 2.0);

    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);

    R(0, 0) = m_stdev_latitude + cov_pos_x * 0.2 + std::pow(pose_stddev_x_, 2);  //+ cov_pos_x; //  pos_x
    R(1, 1) = m_stdev_longitude + cov_pos_y * 0.2 + std::pow(pose_stddev_y_, 2); //+ cov_pos_y +  pos_y
    R(2, 2) = m_stdev_azimuth + cov_yaw + std::pow(pose_stddev_yaw_, 2);         // + cov_yaw; //  yaw

    double bottom_measurement_uncertainty = sqrt(m_localization_status.bottom_error);
    double top_measurement_uncertainty = sqrt(m_localization_status.top_error);

    if (bBOTTOM_GPS && bTOP_GPS) // if TOP AND BOTTOM are started, calculated covariance using difference between two sensors.
    {

      if (m_localization_status.localization_status_code ==
              nif_msgs::msg::LocalizationStatus::BEST_STATUS ||
          m_localization_status.localization_status_code ==
              nif_msgs::msg::LocalizationStatus::SENSOR_FUSION)
      {
        R(0, 0) = BestPosTop.lat_noise * m_localization_status.top_weight + BestPosBottom.lat_noise * m_localization_status.bottom_weight + cov_pos_x*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_x_, 2);
        R(1, 1) = BestPosTop.lon_noise * m_localization_status.top_weight + BestPosBottom.lon_noise * m_localization_status.bottom_weight + cov_pos_y*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_y_, 2);
        R(2, 2) = m_stdev_azimuth + cov_yaw +
                  std::pow(pose_stddev_yaw_, 2);
      }
      else if (m_localization_status.localization_status_code ==
              nif_msgs::msg::LocalizationStatus::SECONDARY_SENSOR_FUSION ||
              m_localization_status.localization_status_code ==
              nif_msgs::msg::LocalizationStatus::VECTORNAV_INS)
      {
        R(0, 0) = m_dSecondaryCorrectionError + cov_pos_x*pose_measure_uncertainty_time_ + std::pow(pose_stddev_x_, 2);
        R(1, 1) = m_dSecondaryCorrectionError + cov_pos_y*pose_measure_uncertainty_time_ + std::pow(pose_stddev_y_, 2);
        R(2, 2) = m_stdev_azimuth + cov_yaw + std::pow(pose_stddev_yaw_, 2);
      }
      else if (m_localization_status.localization_status_code ==
               nif_msgs::msg::LocalizationStatus::ONLY_TOP)
      {
        R(0, 0) = m_dNovatelCorrectionError + BestPosTop.lat_noise + cov_pos_x*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_x_, 2);
        R(1, 1) = m_dNovatelCorrectionError + BestPosTop.lon_noise + cov_pos_y*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_y_, 2);
        R(2, 2) = m_stdev_azimuth + cov_yaw +
                  std::pow(pose_stddev_yaw_, 2);
      }
      else if (m_localization_status.localization_status_code ==
               nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM)
      {
        R(0, 0) = m_dNovatelCorrectionError + BestPosBottom.lat_noise + cov_pos_x*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_x_, 2);
        R(1, 1) = m_dNovatelCorrectionError + BestPosBottom.lon_noise + cov_pos_y*pose_measure_uncertainty_time_ +
                  std::pow(pose_stddev_y_, 2);
        R(2, 2) = m_stdev_azimuth + cov_yaw +
                  std::pow(pose_stddev_yaw_, 2);
      }
      else
      {
        R(0, 0) = m_stdev_latitude + cov_pos_x + std::pow(pose_stddev_x_, 2);  //+ cov_pos_x; //  pos_x
        R(1, 1) = m_stdev_longitude + cov_pos_y + std::pow(pose_stddev_y_, 2); //+ cov_pos_y +  pos_y
        R(2, 2) = m_stdev_azimuth + cov_yaw + std::pow(pose_stddev_yaw_, 2);         // + cov_yaw; //  yaw
      }
    }
  }

  /* In order to avoid a large change at the time of updating, measuremeent
   * update is performed by dividing at every step. */
  R *= (ekf_rate_ / pose_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void AWLocalizationNode::measurementUpdateTwist(rclcpp::Time measurement_time_,
                                                const double &pred_vel_x_,
                                                const double &pred_yaw_rate_)
{

  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2; //  vx, wz
  auto t_curr = this->now();

  /* Calculate delay step */
  /* Calculate delay step */
  double delay_time = (t_curr - measurement_time_).seconds() +
                      (t_curr - measurement_time_).nanoseconds() * 1e-9 +
                      twist_additional_delay_;

  if (delay_time < 0.0)
  {
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Twist delay exceeds the compensation limit, ignored. delay: "
                         "%f[s], limit = "
                         "extend_state_step * ekf_dt : %f [s]",
                         delay_time, extend_state_step_ * ekf_dt_);
    return;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pred_vel_x_, pred_yaw_rate_;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    // ROS_WARN("[EKF] twist measurement matrix includes NaN of Inf. ignore
    // update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
      ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0; //  for vx
  C(1, IDX::WZ) = 1.0; //  for wz

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_twist_with_covariance_)
  {
    R(0, 0) = current_twist_covariance_.at(0);  //  vx - vx
    R(0, 1) = current_twist_covariance_.at(5);  //  vx - wz
    R(1, 0) = current_twist_covariance_.at(30); //  wz - vx
    R(1, 1) = current_twist_covariance_.at(35); //  wz - wz
  }
  else
  {
    R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * ekf_dt_; //  for vx
    R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * ekf_dt_; //  for wz
  }

  /* In order to avoid a large change by update, measurement update is performed
   * by dividing at every step. */
  R *= (ekf_rate_ / twist_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
};

/*
 * mahalanobisGate
 */
bool AWLocalizationNode::mahalanobisGate(const double &dist_max, const Eigen::MatrixXd &x, const Eigen::MatrixXd &obj_x,
                                         const Eigen::MatrixXd &cov)
{
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov * (x - obj_x);

  m_mahalanobisScore = std::sqrt(mahalanobis_squared(0));

  if (mahalanobis_squared(0) > dist_max * dist_max)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "speed : %f, yaw_rate : %f", m_dVelolcity_X,
                         m_dIMU_yaw_rate);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "[EKF] Pose measurement update, mahalanobis distance is over "
                         "limit.ignore measurement data.");
    return false;
  }

  return true;
}

double AWLocalizationNode::GetmahalanobisDistance(const Eigen::MatrixXd &x,
                                                  const Eigen::MatrixXd &obj_x,
                                                  const Eigen::MatrixXd &cov)
{
  Eigen::MatrixXd mahalanobis_squared =
      (x - obj_x).transpose() * cov * (x - obj_x);
  double distance = std::sqrt(mahalanobis_squared(0));

  return distance;
}

/**
 * @brief CalculateBestCorrection selects the best correction
 *        using novatel_bottom and novatel_top.
 * @param GPS_pose  novatel_bottom/bestpos, novatel_bottom/bestpos
 * @param Current_estimation  current estimation
 * @return fused correction data.
 */
bool AWLocalizationNode::CalculateBestCorrection(
    const GPSCorrectionData_t &BottomDataIn,
    const GPSCorrectionData_t &TopDataIn, const VehPose_t &CurrentEstIn,
    VehPose_t &BestCorrectionOut)
{
  Eigen::MatrixXd P_curr, P_2by2;
  ekf_.getLatestP(P_curr);
  P_2by2 = P_curr.block(0, 0, 2, 2);

  Eigen::MatrixXd y_est(2, 1);
  y_est << CurrentEstIn.x, CurrentEstIn.y;
  // y_est << BottomDataIn.x, BottomDataIn.y;

  Eigen::MatrixXd y_bottom(2, 1);
  y_bottom << BottomDataIn.x, BottomDataIn.y;
  double bottomError = GetmahalanobisDistance(y_est, y_bottom, P_2by2);

  Eigen::MatrixXd y_top(2, 1);
  y_top << TopDataIn.x, TopDataIn.y;
  double topError = GetmahalanobisDistance(y_est, y_top, P_2by2);

  double diff_top_bottom = sqrt((BottomDataIn.x - TopDataIn.x) * (BottomDataIn.x - TopDataIn.x) +
                                (BottomDataIn.y - TopDataIn.y) * (BottomDataIn.y - TopDataIn.y));

  double bottom_yaw_error = fabs(BottomDataIn.yaw - (ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB))); // correction - ekf
  double top_yaw_error = fabs(TopDataIn.yaw - (ekf_.getXelement(IDX::YAW) +
                                               ekf_.getXelement(IDX::YAWB)));
  if (bottom_yaw_error > M_PI)
    bottom_yaw_error = bottom_yaw_error - 2 * M_PI;
  else if (bottom_yaw_error < -M_PI)
    bottom_yaw_error = bottom_yaw_error + 2 * M_PI;

  bottom_yaw_error = fabs(bottom_yaw_error);

  // TODO !Q: isn't this duplicated?
  if ((bottomError < 5.0 || topError < 5.0) && bottom_yaw_error < 0.1)
  { // Safe also when use_inspva_heading = false;
    bInitConverged = true;
  }

  double norm_weight_bottom, norm_weight_top;

  norm_weight_bottom = 1 - bottomError / (bottomError + topError);
  norm_weight_top = 1 - topError / (bottomError + topError);

  m_localization_status.bottom_error = bottomError;
  m_localization_status.top_error = topError;
  m_localization_status.diff = diff_top_bottom;
  m_localization_status.bottom_weight = norm_weight_bottom;
  m_localization_status.top_weight = norm_weight_top;

  double pose_gate_dist_local;
  if (!bInitConverged)
  {
    pose_gate_dist_local = DBL_MAX;
  }
  else
  {
    pose_gate_dist_local = pose_gate_dist_;
  }

  // if bottom and top are close enough, we regard bottom status is nominal.
  // output : bypass fusion. return novatel bottom directly.
  if (diff_top_bottom < 0.5)
  {
    BestCorrectionOut.x =
        BottomDataIn.x * norm_weight_bottom + TopDataIn.x * norm_weight_top;
    BestCorrectionOut.y =
        BottomDataIn.y * norm_weight_bottom + TopDataIn.y * norm_weight_top;

    m_dNovatelCorrectionError = norm_weight_bottom*m_localization_status.bottom_error + norm_weight_top*m_localization_status.top_error;

    m_localization_status.status = "BEST STATUS";
    m_localization_status.localization_status_code =
        nif_msgs::msg::LocalizationStatus::BEST_STATUS;
    update_pose_time = novatel_time_last_update;

    return true;
  }
  else // one of sensor has drift. we should fuse the sensors.
  {
    // 1. our top priority strategy is weight-sum.
    if (bottomError < pose_gate_dist_local && topError < pose_gate_dist_local &&
        fabs(topError - bottomError) < pose_gate_dist_local)
    {
      BestCorrectionOut.x =
          BottomDataIn.x * norm_weight_bottom + TopDataIn.x * norm_weight_top;
      BestCorrectionOut.y =
          BottomDataIn.y * norm_weight_bottom + TopDataIn.y * norm_weight_top;
    
      m_dNovatelCorrectionError = norm_weight_bottom*m_localization_status.bottom_error + norm_weight_top*m_localization_status.top_error;
      
      m_localization_status.status = "SENSOR FUSION";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::SENSOR_FUSION;
      update_pose_time = novatel_time_last_update;

      return true;
    }
    // 2. if bottom is diverged, and top is nominal, USE NOVATEL TOP
    else if ((bottomError > pose_gate_dist_local &&
              topError < pose_gate_dist_local) ||
             (!BottomDataIn.quality_code_ok &&
              TopDataIn.quality_code_ok))
    {

      BestCorrectionOut.x = TopDataIn.x;
      BestCorrectionOut.y = TopDataIn.y;

      m_dNovatelCorrectionError = m_localization_status.top_error;

      m_localization_status.status = "BOTTOM ERROR, USE TOP";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_TOP;
      update_pose_time = top_bestpos_time_last_update;

      return true;
    }
    // 3. if top is diverged, and bottom is nominal, USE NOVATEL BOTTOM
    else if ((bottomError < pose_gate_dist_local &&
              topError > pose_gate_dist_local) ||
             (BottomDataIn.quality_code_ok &&
              !TopDataIn.quality_code_ok))
    {

      BestCorrectionOut.x = BottomDataIn.x;
      BestCorrectionOut.y = BottomDataIn.y;

      m_dNovatelCorrectionError = m_localization_status.bottom_error;

      m_localization_status.status = "TOP ERROR, USE BOTTOM";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM;
      update_pose_time = bestpos_time_last_update;

      return true;
    }
    // 4. if all the sensors are bad, do not update measurement
    else
    {
      // GPS_HIGH_ERROR -> MONZA_TEMP_GPS_HIGH_ERROR
      m_dNovatelCorrectionError = m_localization_status.bottom_error + m_localization_status.top_error;

      m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::MONZA_TEMP_GPS_HIGH_ERROR;

      return false;
    }
  }
}

bool AWLocalizationNode::CalculateSecondaryCorrection(
    const GPSCorrectionData_t &firstDataIn,
    const GPSCorrectionData_t &secondDataIn, const GPSCorrectionData_t &thirdDataIn, const VehPose_t &CurrentEstIn,
    VehPose_t &BestCorrectionOut)
{
// std::cout<<"si"<<std::endl;
  std::deque<GPSCorrectionData_t> correctionDataQueue;
  std::deque<double> mahalanobisDistanceQueue;

  double firstError,secondError,thirdError = 100.0;

  Eigen::MatrixXd P_curr, P_2by2;
  ekf_.getLatestP(P_curr);
  P_2by2 = P_curr.block(0, 0, 2, 2);

  Eigen::MatrixXd y_est(2, 1);
  y_est << CurrentEstIn.x, CurrentEstIn.y;

  Eigen::MatrixXd y_first(2, 1);
  y_first << firstDataIn.x, firstDataIn.y;
  double first_noise_total =
        sqrt(firstDataIn.lat_noise * firstDataIn.lat_noise +
             firstDataIn.lon_noise * firstDataIn.lon_noise);
  // From the vectornav INS data that we acquired at MONZA Track.
  // VN_POSU is maintained under 0.6(after bridge) in norminal cases,
  // is increased up to 0.9(after bridge) in harsh case.
  // When the TUM makes decisions whether the vectornav ins data is correct or not,
  // they use this posu threshold as 1.4, And we used this parameter more conservative.

  if(first_noise_total < 1.2) firstError = GetmahalanobisDistance(y_est, y_first, P_2by2);

  Eigen::MatrixXd y_second(2, 1);
  y_second << secondDataIn.x, secondDataIn.y;
  double prior_noise = 0.0;
  if(use_reference_information)
  {
    prior_noise = 0.5*priorMeasurementUncertainty;
  }
  double second_noise_total =
        sqrt(secondDataIn.lat_noise * secondDataIn.lat_noise +
             secondDataIn.lon_noise * secondDataIn.lon_noise + prior_noise);
  if(second_noise_total < 2.0) secondError = GetmahalanobisDistance(y_est, y_second, P_2by2);

  Eigen::MatrixXd y_third(2, 1);
  y_third << thirdDataIn.x, thirdDataIn.y;
  double third_noise_total =
        sqrt(thirdDataIn.lat_noise * thirdDataIn.lat_noise +
             thirdDataIn.lon_noise * thirdDataIn.lon_noise + prior_noise);
  if(third_noise_total < 2.0) thirdError = GetmahalanobisDistance(y_est, y_third, P_2by2);

  int numQualifiedMeasurements = 0;
  if (firstError < 3.0 || secondError < 3.0 || thirdError < 3.0)
  {
// std::cout<<"bal"<<std::endl;

    if(!bInitConverged) bInitConverged = true;

    if (firstError < 3.0)
    {
      correctionDataQueue.push_back(firstDataIn);
      mahalanobisDistanceQueue.push_back(firstError);
      numQualifiedMeasurements += 1;
    }
    else
    {
      mahalanobisDistanceQueue.clear();
      correctionDataQueue.clear();
      return false;
    }
    // if (secondError < 3.0)
    // {
    //   if (numQualifiedMeasurements == 1)
    //   {
    //     if (mahalanobisDistanceQueue[0] > secondError)
    //     {
    //       mahalanobisDistanceQueue.push_front(secondError);
    //       correctionDataQueue.push_front(secondDataIn);
    //     }
    //     else
    //     {
    //       mahalanobisDistanceQueue.push_back(secondError);
    //       correctionDataQueue.push_back(secondDataIn);
    //     }
    //   }
    //   else
    //   {
    //     mahalanobisDistanceQueue.push_back(secondError);
    //     correctionDataQueue.push_back(secondDataIn);
    //   }
    //   numQualifiedMeasurements += 1;
    // }
    // if (thirdError < 3.0)
    // {
    //   if (numQualifiedMeasurements == 1)
    //   {
    //     if (mahalanobisDistanceQueue[0] > thirdError)
    //     {
    //       mahalanobisDistanceQueue.push_front(thirdError);
    //       correctionDataQueue.push_front(thirdDataIn);
    //     }
    //     else
    //     {
    //       mahalanobisDistanceQueue.push_back(thirdError);
    //       correctionDataQueue.push_back(thirdDataIn);
    //     }
    //   }
    //   else if (numQualifiedMeasurements == 0)
    //   {
    //     mahalanobisDistanceQueue.push_back(thirdError);
    //     correctionDataQueue.push_back(thirdDataIn);
    //   }
    //   else
    //   {
    //     if (mahalanobisDistanceQueue[0] > thirdError)
    //     {
    //       mahalanobisDistanceQueue.push_front(thirdError);
    //       correctionDataQueue.push_front(thirdDataIn);
    //     }
    //     else if (mahalanobisDistanceQueue[1] > thirdError)
    //     {
    //       mahalanobisDistanceQueue.pop_back();
    //       correctionDataQueue.pop_back();
    //       mahalanobisDistanceQueue.push_back(thirdError);
    //       correctionDataQueue.push_back(thirdDataIn);
    //     }
    //   }
    // }
  }
  else
  {
    mahalanobisDistanceQueue.clear();
    correctionDataQueue.clear();
    return false;
  }

  if (numQualifiedMeasurements >= 2)
  {
    double norm_weight_first, norm_weight_second;

    firstError = mahalanobisDistanceQueue[0];
    secondError = mahalanobisDistanceQueue[1];

    norm_weight_first = 1 - firstError / (firstError + secondError);
    norm_weight_second = 1 - secondError / (firstError + secondError);

    BestCorrectionOut.x =
        correctionDataQueue[0].x * norm_weight_first + correctionDataQueue[1].x * norm_weight_second;
    BestCorrectionOut.y =
        correctionDataQueue[0].y * norm_weight_first + correctionDataQueue[1].y * norm_weight_second;

    m_dSecondaryCorrectionError = norm_weight_first*firstError + norm_weight_second*secondError;

    if(m_dSecondaryCorrectionError<m_dNovatelCorrectionError)
    {
      m_localization_status.status = "SECONDARY SENSOR FUSION";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::SECONDARY_SENSOR_FUSION;
      update_pose_time = secondary_time_last_update;
      mahalanobisDistanceQueue.clear();
      correctionDataQueue.clear();
      return true;
    }
    else
    {
      mahalanobisDistanceQueue.clear();
      correctionDataQueue.clear();
      return false;
    }

  }
  else if (numQualifiedMeasurements == 1)
  {
    BestCorrectionOut.x = correctionDataQueue[0].x;
    BestCorrectionOut.y = correctionDataQueue[0].y;
// std::cout<<"ryun"<<std::endl;

    m_dSecondaryCorrectionError = mahalanobisDistanceQueue[0];

    if(m_dSecondaryCorrectionError<m_dNovatelCorrectionError)
    {
// std::cout<<"ah"<<std::endl;

      m_localization_status.status = "VECTORNAV INS";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::VECTORNAV_INS;
      update_pose_time = secondary_time_last_update;
      mahalanobisDistanceQueue.clear();
      correctionDataQueue.clear();
      return true;
    }
    else
    {
      mahalanobisDistanceQueue.clear();
      correctionDataQueue.clear();
      return false;
    }
  }
  else
  {
    mahalanobisDistanceQueue.clear();
    correctionDataQueue.clear();
    return false;
  }
}

/*
  * GPSIgnoreGate
  * Maximum body slip angle(beta) : 0.2479
      = rad = atan(y_delta /  x_delta)
  */
bool AWLocalizationNode::GPSIgnoreGate(
    const double &dist_max,
    const Eigen::MatrixXd &x,     // estimated
    const Eigen::MatrixXd &obj_x, // correct
    const Eigen::MatrixXd &cov)
{

  double x_delta = (obj_x(0, 0) - x(0, 0)) * cos(x(2, 0)) + (obj_x(1, 0) - x(1, 0)) * sin(x(2, 0));
  double y_delta = -(obj_x(0, 0) - x(0, 0)) * sin(x(2, 0)) + (obj_x(1, 0) - x(1, 0)) * cos(x(2, 0));

  RCLCPP_INFO(this->get_logger(), "x_delta = %f, y_delta = %f", x_delta, y_delta);

  if (x_delta < 0. && atan(fabs(y_delta / x_delta)) > 0.2479)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Non-holonomic model cannot move like this");
    return false;
  }

  return true;
}

/*
 * publishEstimateResult
 */
void AWLocalizationNode::publishEstimateResult()
{
  auto current_time = this->now();
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  current_ekf_odom_.header.stamp = current_time;
  current_ekf_odom_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_odom_.child_frame_id = nif::common::frame_id::localization::BASE_LINK;
  current_ekf_odom_.pose.pose = current_ekf_pose_.pose;
  current_ekf_odom_.pose.covariance[0] = P(IDX::X, IDX::X);
  current_ekf_odom_.pose.covariance[1] = P(IDX::X, IDX::Y);
  current_ekf_odom_.pose.covariance[5] = P(IDX::X, IDX::YAW);
  current_ekf_odom_.pose.covariance[6] = P(IDX::Y, IDX::X);
  current_ekf_odom_.pose.covariance[7] = P(IDX::Y, IDX::Y);
  current_ekf_odom_.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  current_ekf_odom_.pose.covariance[30] = P(IDX::YAW, IDX::X);
  current_ekf_odom_.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  current_ekf_odom_.pose.covariance[35] = P(IDX::YAW, IDX::YAW);

  current_ekf_odom_.twist.twist = current_ekf_twist_.twist;
  current_ekf_odom_.twist.covariance[0] = P(IDX::VX, IDX::VX);
  current_ekf_odom_.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  current_ekf_odom_.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  current_ekf_odom_.twist.covariance[35] = P(IDX::WZ, IDX::WZ);

  if(use_reference_information){
    CalcReferenceStdev(current_ekf_odom_);
  }

  double uncertainty = sqrt(P(IDX::X, IDX::X) * P(IDX::X, IDX::X) + P(IDX::Y, IDX::Y) * P(IDX::Y, IDX::Y));
  m_localization_status.uncertainty = uncertainty;

  pub_odom_->publish(current_ekf_odom_);

  if (this->enable_tf_publisher)
  {
    geometry_msgs::msg::TransformStamped nav_base_tf{};
    nav_base_tf.transform.translation.x = current_ekf_odom_.pose.pose.position.x;
    nav_base_tf.transform.translation.y = current_ekf_odom_.pose.pose.position.y;
    nav_base_tf.transform.translation.z = current_ekf_odom_.pose.pose.position.z;
    nav_base_tf.transform.rotation.w = current_ekf_odom_.pose.pose.orientation.w;
    nav_base_tf.transform.rotation.x = current_ekf_odom_.pose.pose.orientation.x;
    nav_base_tf.transform.rotation.y = current_ekf_odom_.pose.pose.orientation.y;
    nav_base_tf.transform.rotation.z = current_ekf_odom_.pose.pose.orientation.z;
    nav_base_tf.header.stamp = this->now();
    nav_base_tf.header.frame_id = nif::common::frame_id::localization::ODOM;
    nav_base_tf.child_frame_id = nif::common::frame_id::localization::BASE_LINK;
    broadcaster_->sendTransform(nav_base_tf);
  }

  /* publish localization score*/
  std_msgs::msg::Float64 mahalanobisScoreMsg;
  mahalanobisScoreMsg.data = m_mahalanobisScore;
  pub_mahalanobisScore->publish(mahalanobisScoreMsg);

  /* publish yaw bias */
  std_msgs::msg::Float64 yawb;
  yawb.data = X(IDX::YAWB);
  pub_yaw_bias_->publish(yawb);

  /* debug measured pose */
  if (measure_odom_ptr_ != nullptr)
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose = measure_odom_ptr_->pose.pose;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }
}

double AWLocalizationNode::normalizeYaw(const double &yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

/*
 * MUST remain pull-down only, as other checks are made in each callback.
 */
void AWLocalizationNode::headingAgeCheck()
{
  auto now = this->now();

  if (!bBOTTOMGPSHeading || now - bestvel_time_last_update > gps_timeout)
  {
    m_bestvel_bottom_valid = false;
  }

  if (!bTOPGPSHeading || now - top_bestvel_time_last_update > gps_timeout)
  {
    m_bestvel_top_valid = false;
  }

  if (m_heading2_valid || now - heading2_time_last_update > gps_timeout)
  {
    m_heading2_valid = false;
  }
}

void AWLocalizationNode::service_handler_forced_heading(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const nif_msgs::srv::LocalizationForcedHeading::Request::SharedPtr request,
    nif_msgs::srv::LocalizationForcedHeading::Response::SharedPtr response)
{
  if (request->heading_deg == 0)
  {
    this->m_bUseHeading2 = true;

    response->success = true;
    response->message = "OK: Heading source from Heading2.";
    response->heading = 0.5 * (m_top_heading2_heading_rad + m_top_heading2_heading_rad) * nif::common::constants::RAD2DEG * (-1.0);
    return;
  }
  else
  {
    double forced_heading_rad = request->heading_deg * nif::common::constants::DEG2RAD * (-1.0);
    m_dGPS_Heading = forced_heading_rad;
    m_dGPS_TOP_Heading = forced_heading_rad;
    m_dGPS_Kinematic_Heading = forced_heading_rad;
    m_last_dGPS_Kinematic_Heading = m_dGPS_Kinematic_Heading;
    m_d_TOP_GPS_Kinematic_Heading = forced_heading_rad;
    m_last_d_TOP_GPS_Kinematic_Heading = m_d_TOP_GPS_Kinematic_Heading;
    m_top_heading2_heading_rad = forced_heading_rad;
    m_heading2_heading_rad = forced_heading_rad;
    m_dVN1_Heading = forced_heading_rad;
    m_dVN2_Heading = forced_heading_rad;
    m_XSENS_GPS_Heading = forced_heading_rad;
    m_localization_status.heading_source = nif_msgs::msg::LocalizationStatus::FORCED_HEADING;

    bForcedHeading = true;

    response->success = true;
    response->message = "OK: Heading source from Forced Heading.";
    response->heading = request->heading_deg;
    return;
  }
}

void AWLocalizationNode::service_handler_forced_initialize(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const nif_msgs::srv::LocalizationForcedInitialize::Request::SharedPtr request,
    nif_msgs::srv::LocalizationForcedInitialize::Response::SharedPtr response)
{
  if (!request->initialize)
  {
    bInitConverged = false;
    initEKF();

    response->success = true;
    response->message = "OK: EKF REINITIALIZATION START.";
    return;
  }
  else
  {
    if (!bInitConverged)
    {
      bInitConverged = true;
      response->success = true;
      response->message = "OK: EKF FORCED CONVERGE START.";
      return;
    }
    else
    {
      response->success = false;
      response->message = "EKF ALREADY CONVERGED, TRY REINITIALIZE (initialize=false).";
      return;
    }
  }
}

void AWLocalizationNode::pcdFileIO() {
	m_reference_info_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
		new pcl::PointCloud<pcl::PointXYZI>);
	//  read m_reference_info_ptr from a pcd file
	RCLCPP_INFO(this->get_logger(), "LOADING REFERENCE GNSS INFORMATION...");

	pcl::io::loadPCDFile(reference_information_filename, *CloudIn);
	if (CloudIn->size() > 0) {
		RCLCPP_INFO(this->get_logger(), "REFERENCE GNSS INFORMATION IS LOADED!");
		RCLCPP_INFO(this->get_logger(), "POINT SIZE: %d", CloudIn->points.size());
	} else {
		RCLCPP_INFO(this->get_logger(), "...");
	}
	
	m_reference_info_ptr = CloudIn; // assign the loaded point cloud directly to m_reference_info_ptr
	m_reference_info_ptr->header.frame_id = nif::common::frame_id::localization::ODOM;
	bReferenceReady = true;
}
void AWLocalizationNode::ReferencePublisher() {
  sensor_msgs::msg::PointCloud2 GnssReferenceMapCloudMsg;
  pcl::toROSMsg(*m_reference_info_ptr, GnssReferenceMapCloudMsg);
  GnssReferenceMapCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  GnssReferenceMapCloudMsg.header.stamp = this->now();
  pub_gnss_reference_points_->publish(GnssReferenceMapCloudMsg);
}

void AWLocalizationNode::CalcReferenceStdev(const nav_msgs::msg::Odometry& odom_msg)
{
    // Convert the odometry message to a PCL point
    if(!bReferenceReady) return;
    pcl::PointXYZ query_point(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    pcl::PointXYZI max_intensity_point;
    double max_intensity = 0.0;

    // Find the point with the largest intensity within a 10-meter radius of the query point
    double max_radius = 10.0;
    for (const pcl::PointXYZI& point : m_reference_info_ptr->points)
    {
        double dist = std::sqrt(std::pow(point.x - query_point.x, 2) + std::pow(point.y - query_point.y, 2));
        if (dist < max_radius && point.intensity > max_intensity)
        {
            max_intensity = point.intensity;
            max_intensity_point = point;
        }
    }

    // Set the prior measurement uncertainty based on the intensity of the closest point within 10 meters
    if (max_intensity > 0.0)
    {
        this->priorMeasurementUncertainty = 0.2*max_intensity;
        // std::cout << "priorMeasurementUncertainty: " << priorMeasurementUncertainty << std::endl;
    }
    else
    {
        this->priorMeasurementUncertainty = 0.0;
    }
}