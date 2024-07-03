#include "nif_control_joint_lqr_nodes/control_lqr_node.h"

using nif::control::ControlLQRNode;

ControlLQRNode::ControlLQRNode(const std::string &node_name)
    : IControllerNode(node_name)
{
  control_cmd = std::make_shared<nif::common::msgs::ControlCmd>();

  m_camber_manager_ptr = std::make_shared<CamberCompensator>(
      nif::control::CAMBERCOMPESATORMODE::FIRST_ORDER, true);

  // Debug Publishers
  lqr_command_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "control_joint_lqr/tracking_valid", nif::common::constants::QOS_DEFAULT);
  lqr_valid_conditions_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "control_joint_lqr/valid_conditions",
          nif::common::constants::QOS_DEFAULT);
  lqr_steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/lqr_command", nif::common::constants::QOS_DEFAULT);
  lqr_accel_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/accel_command", nif::common::constants::QOS_DEFAULT);
  track_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/track_distance", nif::common::constants::QOS_DEFAULT);
  lqr_tracking_idx_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "control_joint_lqr/track_idx", nif::common::constants::QOS_DEFAULT);
  lqr_tracking_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "control_joint_lqr/track_point", nif::common::constants::QOS_DEFAULT);
  lqr_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "control_joint_lqr/lqr_error", nif::common::constants::QOS_DEFAULT);
  lqr_desired_velocity_mps_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
          "control_joint_lqr/desired_velocity_mps",
          nif::common::constants::QOS_DEFAULT);
  steering_compensate_deg_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/steering_compensate_deg", nif::common::constants::QOS_DEFAULT);
  // Subscribers
  velocity_sub_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report",
          nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&ControlLQRNode::velocityCallback, this,
                    std::placeholders::_1));

  direct_desired_velocity_sub =
      this->create_subscription<std_msgs::msg::Float32>(
          "velocity_planner/des_vel", nif::common::constants::QOS_CONTROL_CMD,
          std::bind(&ControlLQRNode::directDesiredVelocityCallback, this,
                    std::placeholders::_1));

  csl_emergency_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
          "/control_safety_layer/use_local_path",
          nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&ControlLQRNode::cslEmergencyCallback, this,
                    std::placeholders::_1));
  oppo_is_back_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/oppo_is_back",
      nif::common::constants::QOS_PLANNING,
      std::bind(&ControlLQRNode::oppoBackCallback, this,
                std::placeholders::_1));

  sub_system_status_ =
      this->create_subscription<nif::common::msgs::SystemStatus>(
          "/system/status",
          nif::common::constants::QOS_INTERNAL_STATUS,
          std::bind(&ControlLQRNode::SystemStatusCallback, this,
                    std::placeholders::_1));

  // Disable the ACC function
  //   acc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
  //       "control/acc/accel_cmd", nif::common::constants::QOS_CONTROL_CMD,
  //       std::bind(&ControlLQRNode::accCMDCallback, this,
  //       std::placeholders::_1));

  this->declare_parameter("lqr_config_file", "");
  this->declare_parameter("pi_config_file", "");
  // Automatically boot with lat_autonomy_enabled
  //  this->declare_parameter("lat_autonomy_enabled", false);
  // Max Steering Angle in Degrees
  this->declare_parameter("max_steering_angle_deg", 20.0);
  // convert from degress to steering units (should be 1 - 1 ?)
  this->declare_parameter("steering_units_multiplier", 1.0);
  // Minimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_min_dist_m", 3.5);
  // Maximimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_max_dist_m", 8.0);
  // pure_pursuit lookahead distance 1st velocity theshold (65 kph)
  this->declare_parameter("pure_pursuit_1st_vel_ms", 15.0);
  // Maximimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_max_max_dist_m", 13.0);
  // Factor to increase the pure pursuit tracking distance as a function of
  // speed (m/s)
  this->declare_parameter("pure_pursuit_k_vel_m_ms", 1.0);
  // Use tire speed instead of gps velocity estimate
  this->declare_parameter("use_tire_velocity", true);
  // Safety timeouts for odometry and the path (set negative to ignore)
  this->declare_parameter("odometry_timeout_sec", 0.1);
  this->declare_parameter("path_timeout_sec", 0.5);
  // Limit the max change in the steering signal over time
  this->declare_parameter("steering_max_ddeg_dt", 7.0);
  // Limit the max change in the des_accel signal over time (acceleration)
  this->declare_parameter("des_accel_max_da_dt", 5.0); // from 5 to 9
  // this->declare_parameter("des_accel_max_da_dt", 0.2); // from 5 to 9
  // Limit the max change in the des_accel signal over time (decceleration)
  this->declare_parameter("des_deccel_max_da_dt", 10.0); // keep 5
  // this->declare_parameter("des_deccel_max_da_dt", 0.3); // keep 5
  // Minimum length of the reference path
  this->declare_parameter("path_min_length_m", 30.0);
  this->declare_parameter("front_car_max_decel", -4.8);
  //  Invert steering command for simulation
  this->declare_parameter("invert_steering", false);
  // Use mission status maximum desired velocity
  this->declare_parameter("use_mission_max_vel", true);
  // Use ACC cmd
  this->declare_parameter("use_acc", false);
  // Use Camber Compensator
  this->declare_parameter("use_camber_compensator", false);

  this->declare_parameter("accel_p_gain", 0.0);
  this->declare_parameter("accel_i_gain", 0.0);
  this->declare_parameter("track_max_m_dt_", 4.0);
  this->declare_parameter("force_steering_limit", 3.0);
  this->declare_parameter("pi_lookup_table", true);
  // Create Joint LQR Controller from yaml file
  std::string lqr_config_file =
      this->get_parameter("lqr_config_file").as_string();
  if (lqr_config_file.empty())
    throw std::runtime_error(
        "Parameter lqr_config_file not declared, or empty.");

  RCLCPP_INFO(get_logger(), "Loading control params: %s",
              lqr_config_file.c_str());
  joint_lqr_ = joint_lqr::lqr::JointLQR::newPtr(lqr_config_file);

  m_pi_config_file_path =
      this->get_parameter("pi_config_file").as_string();
  if (m_pi_config_file_path.empty())
  {
    throw std::runtime_error(
        "Parameter m_pi_config_file_path not declared, or empty.");
  }
  loadConfig(m_pi_config_file_path);

  // Read in misc. parameters
  max_steering_angle_deg_ =
      this->get_parameter("max_steering_angle_deg").as_double();
  steering_units_multiplier_ =
      this->get_parameter("steering_units_multiplier").as_double();
  pure_pursuit_min_dist_m_ =
      this->get_parameter("pure_pursuit_min_dist_m").as_double();
  pure_pursuit_max_dist_m_ =
      this->get_parameter("pure_pursuit_max_dist_m").as_double();
  pure_pursuit_1st_vel_ms_ =
      this->get_parameter("pure_pursuit_1st_vel_ms").as_double();
  pure_pursuit_max_max_dist_m_ =
      this->get_parameter("pure_pursuit_max_max_dist_m").as_double();
  pure_pursuit_k_vel_m_ms_ =
      this->get_parameter("pure_pursuit_k_vel_m_ms").as_double();
  use_tire_velocity_ = this->get_parameter("use_tire_velocity").as_bool();
  odometry_timeout_sec_ =
      this->get_parameter("odometry_timeout_sec").as_double();
  path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
  steering_max_ddeg_dt_ =
      this->get_parameter("steering_max_ddeg_dt").as_double();
  des_accel_max_da_dt_ = this->get_parameter("des_accel_max_da_dt").as_double();
  des_deccel_max_da_dt_ =
      this->get_parameter("des_deccel_max_da_dt").as_double();
  front_car_max_decel_ =
      this->get_parameter("front_car_max_decel").as_double();
  invert_steering_ = this->get_parameter("invert_steering").as_bool();
  m_use_mission_max_vel_ = this->get_parameter("use_mission_max_vel").as_bool();
  m_path_min_length_m = this->get_parameter("path_min_length_m").as_double();
  // m_use_acc = this->get_parameter("use_acc").as_bool();
  m_use_acc = false;
  use_camber_compensator_ = this->get_parameter("use_camber_compensator").as_bool();
  m_accel_p_gain = this->get_parameter("accel_p_gain").as_double();
  m_accel_i_gain = this->get_parameter("accel_i_gain").as_double();
  m_track_max_m_dt_ = this->get_parameter("track_max_m_dt_").as_double();
  m_force_steering_limit = this->get_parameter("force_steering_limit").as_double();
  m_pi_lookup_table = this->get_parameter("pi_lookup_table").as_bool();


  if (odometry_timeout_sec_ <= 0. || path_timeout_sec_ <= 0.)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "path and ego_odometry timeouts must be greater than zero. "
                 "Got odometry_timeout_sec_: %f; path_timeout_sec_: %f",
                 odometry_timeout_sec_, path_timeout_sec_);
    throw std::range_error("Parameter out of range.");
  }

  this->parameters_callback_handle =
      this->add_on_set_parameters_callback(std::bind(
          &ControlLQRNode::parametersCallback, this, std::placeholders::_1));
}

void ControlLQRNode::publishSteerAccelDiagnostics(
    bool lqr_command_valid, bool valid_path, bool valid_odom,
    bool valid_wpt_distance, bool valid_target_position,
    double lqr_steering_command, double lqr_accel_command,
    double track_distance, unsigned int lqr_tracking_idx,
    geometry_msgs::msg::PoseStamped lqr_track_point,
    joint_lqr::lqr::JointLQR::ErrorMatrix lqr_err_cog,
    joint_lqr::lqr::JointLQR::ErrorMatrix lqr_err,
    double desired_velocity_mps)
{
  std_msgs::msg::Bool command_valid_msg;
  command_valid_msg.data = lqr_command_valid;
  lqr_command_valid_pub_->publish(command_valid_msg);

  std_msgs::msg::Float32MultiArray valid_conditions_msg;
  valid_conditions_msg.data.push_back(lqr_command_valid);
  valid_conditions_msg.data.push_back(valid_path);
  valid_conditions_msg.data.push_back(valid_odom);
  valid_conditions_msg.data.push_back(valid_wpt_distance);
  valid_conditions_msg.data.push_back(valid_target_position);

  lqr_valid_conditions_pub_->publish(valid_conditions_msg);

  std_msgs::msg::Float32 steering_command_msg;
  steering_command_msg.data = lqr_steering_command;
  lqr_steering_command_pub_->publish(steering_command_msg);

  std_msgs::msg::Float32 desired_accel_command_msg;
  desired_accel_command_msg.data = lqr_accel_command;
  lqr_accel_command_pub_->publish(desired_accel_command_msg);

  std_msgs::msg::Float32 track_distance_msg;
  track_distance_msg.data = track_distance;
  track_distance_pub_->publish(track_distance_msg);

  std_msgs::msg::Int32 lqr_tracking_idx_msg;
  lqr_tracking_idx_msg.data = lqr_tracking_idx;
  lqr_tracking_idx_pub_->publish(lqr_tracking_idx_msg);

  lqr_track_point.header.frame_id = "odom";
  lqr_tracking_point_pub_->publish(lqr_track_point);
  auto error_cog_array_msg =
      joint_lqr::utils::ROSError(lqr_err_cog); // Float32MultiArray
  auto error_array_msg =
      joint_lqr::utils::ROSError(lqr_err); // Float32MultiArray
  error_cog_array_msg.data.push_back(error_array_msg.data[0]);
  error_cog_array_msg.data.push_back(error_array_msg.data[1]);
  error_cog_array_msg.data.push_back(error_array_msg.data[2]);
  error_cog_array_msg.data.push_back(error_array_msg.data[3]);
  error_cog_array_msg.data.push_back(error_array_msg.data[4]);
  lqr_error_pub_->publish(error_cog_array_msg);

  std_msgs::msg::Float32 des_vel_msg{};
  des_vel_msg.data = desired_velocity_mps;
  lqr_desired_velocity_mps_pub_->publish(des_vel_msg);
}

nif::common::msgs::ControlCmd::SharedPtr ControlLQRNode::solve()
{
  auto now = this->now();

  double period_double_s =
          nif::common::utils::time::secs(this->getGclockPeriodDuration());

  
  

  // std::cout << "speed : " << current_speed_ms_ << " p_gain : " << m_accel_p_gain << " i_gain : " << m_accel_i_gain << std::endl;
  nif::common::NodeStatusCode node_status = common::NODE_ERROR;
  nav_msgs::msg::Odometry empty_odom = nav_msgs::msg::Odometry();
  //  Check whether we have updated data
  bool valid_path =
      this->hasReferenceTrajectory() &&
      !this->getReferenceTrajectory()->trajectory_path.poses.empty() &&
      this->getReferenceTrajLastPointDistance() > m_path_min_length_m &&
      nif::common::utils::time::secs(
          now - this->getReferenceTrajectoryUpdateTime()) < path_timeout_sec_;
  bool valid_perception_path =
      this->hasReferencePath() &&
      !this->getReferencePath()->poses.empty() &&
      nif::common::utils::time::secs(now - this->getReferencePathUpdateTime()) < path_timeout_sec_;

  bool valid_odom =
      this->hasEgoOdometry() &&
      nif::common::utils::time::secs(now - this->getEgoOdometryUpdateTime()) <
          odometry_timeout_sec_;

  // Check valid waypoint starting distance & valid target waypoint
  // position(front)
  // - starting wpt should be around ego.
  bool valid_wpt_distance =
      valid_path && valid_odom &&
      pure_pursuit_max_max_dist_m_ >
          joint_lqr::utils::pursuit_dist(
              this->getReferenceTrajectory()->trajectory_path.poses[0],
              this->getEgoOdometry());

  // - initialize valid target position
  bool valid_target_position = false;

  bool valid_tracking_result = false;

  double steering_angle_deg = 0.0;
  double desired_accel = 0.0;
  joint_lqr::lqr::JointLQR::ErrorMatrix error;
  joint_lqr::lqr::JointLQR::ErrorMatrix error_COG; // error at center of gravity

  bool is_local_path = (csl_use_local_path_ || !valid_path || !valid_odom) && valid_perception_path && this->hasReferencePath() && (this->getReferencePath()->header.frame_id == this->getBodyFrameId());
  // std::cout << valid_path << valid_odom << valid_perception_path << is_local_path << csl_use_local_path_ << std::endl;

  // Perform Tracking if path is good
  if ((valid_path && valid_odom) || (valid_perception_path && is_local_path))
  {
    // Check whether path is global/local
    // bool is_local_path = this->getReferenceTrajectory()->header.frame_id == this->getBodyFrameId()

    // if (!valid_odom || !valid_path) is_local_path = true;
    joint_lqr::lqr::JointLQR::StateMatrix state;
    if (is_local_path)
    {
      state = joint_lqr::utils::LQRState(empty_odom);
      // x, y, yaw are zeros in local coordinate
      state(0, 0) = 0.0;
      state(1, 0) = 0.0;
      state(4, 0) = 0.0;
      valid_wpt_distance = pure_pursuit_max_max_dist_m_ >
                           joint_lqr::utils::pursuit_dist(
                               this->getReferencePath()->poses[0], empty_odom);
    }
    else
      state = joint_lqr::utils::LQRState(this->getEgoOdometry());

    if (use_tire_velocity_)
      state(2, 0) = current_speed_ms_;
    // std::cout << "debug1" << std::endl;
    // Compute the tracking distance (and ensure it is within a valid range)
    double track_distance =
        pure_pursuit_min_dist_m_ + pure_pursuit_k_vel_m_ms_ * state(2, 0);

    if (state(2, 0) < 17.0)
    {
      track_distance = 4.5;
    }

    if (track_distance > pure_pursuit_max_dist_m_)
    {
      if (state(2, 0) < pure_pursuit_1st_vel_ms_)
      {
        track_distance = pure_pursuit_max_dist_m_;
      }
      else
      {
        track_distance =
            pure_pursuit_max_dist_m_ +
            pure_pursuit_k_vel_m_ms_ * (state(2, 0) - pure_pursuit_1st_vel_ms_);
      }
    }

    if (track_distance < pure_pursuit_min_dist_m_)
      track_distance = pure_pursuit_min_dist_m_;
    if (track_distance > pure_pursuit_max_max_dist_m_)
      track_distance = pure_pursuit_max_max_dist_m_;

    track_distance = joint_lqr::utils::smoothSignal(last_track_distance, track_distance,
                                     m_track_max_m_dt_, period_double_s);
    
    last_track_distance = track_distance;


    // Track on the trajectory
    double target_distance = 0.0;
    bool target_reached_end = false;
    double target_point_azimuth;
    // std::cout << "debug2" << std::endl;
    if (is_local_path)
    {
      joint_lqr::utils::track(
          this->getReferencePath()->poses,
          empty_odom,
          track_distance, // inputs
          lqr_tracking_idx_, target_distance,
          target_reached_end); // outputs

      target_point_azimuth = joint_lqr::utils::pursuit_azimuth(
          this->getReferencePath()->poses[lqr_tracking_idx_],
          empty_odom);
    }
    else
    {
      joint_lqr::utils::track(
          this->getReferenceTrajectory()->trajectory_path.poses,
          this->getEgoOdometry(),
          track_distance, // inputs
          lqr_tracking_idx_, target_distance,
          target_reached_end); // outputs

      target_point_azimuth = joint_lqr::utils::pursuit_azimuth(
          this->getReferenceTrajectory()
              ->trajectory_path.poses[lqr_tracking_idx_],
          this->getEgoOdometry());
    }

    // - target point should be ahead.
    valid_target_position = M_PI * 3 / 4. > std::abs(target_point_azimuth);
    // std::cout << valid_target_position << valid_wpt_distance << std::endl;
    double l_desired_velocity = 0.0;
    if (valid_wpt_distance && valid_target_position)
    {
      // std::cout << "debug4-1" << std::endl;
      valid_tracking_result = true;
      // Run LQR :)
      // Desired velocity check

      // deprecated
      //   if (this->hasDesiredVelocity() &&
      //       (this->now() - this->getDesiredVelocityUpdateTime() <=
      //        rclcpp::Duration(1, 0))) {
      //     l_desired_velocity = this->getDesiredVelocity()->data;
      //   }

      // if (this->hasReferenceTrajectory() &&
      //     (this->now() - this->getReferenceTrajectoryUpdateTime() <=
      //      rclcpp::Duration(1, 0))) {
      // TODO: Review this with Hyunki
      // FIXME:
      // l_desired_velocity = this->getReferenceTrajectory()
      //                          ->trajectory_velocity[lqr_tracking_idx_];

      //   ////////////////////////////////////
      //   // Look-ahead time implementation //
      //   ////////////////////////////////////
      //   double test_lookahead_time = 0.8;
      //   // step 1. Search the nearest time within the trajectory's timestamp
      //   // array
      //   auto closest_time_idx = nif::common::utils::closestIndex(
      //       this->getReferenceTrajectory()->trajectory_timestamp_array,
      //       test_lookahead_time);

      //   // step 2. Safety feature
      //   auto time_differ =
      //       abs(this->getReferenceTrajectory()
      //               ->trajectory_timestamp_array[closest_time_idx] -
      //           test_lookahead_time);

      //   if (time_differ < 2) {
      //     l_desired_velocity = this->getReferenceTrajectory()
      //                              ->trajectory_velocity[closest_time_idx];
      //     if (l_desired_velocity < 1.5)
      //       l_desired_velocity = 0.0;
      //   } else {
      //     l_desired_velocity = 0.0;
      //   }
      // }

      // if (!m_use_mission_max_vel_) {
      //   // if not using mission status maximum velocity,
      //   // directly use des_vel from velocity planner
      //   l_desired_velocity = direct_desired_velocity_;
      // }

      l_desired_velocity = direct_desired_velocity_;

      joint_lqr::lqr::JointLQR::GoalMatrix goal;
      joint_lqr::lqr::JointLQR::GoalMatrix goal_COG;

      if (is_local_path)
      {
        goal = joint_lqr::utils::LQRGoal(
            this->getReferencePath()->poses[lqr_tracking_idx_],
            l_desired_velocity);
        goal_COG = joint_lqr::utils::LQRGoal(
            this->getReferencePath()->poses[0],
            l_desired_velocity);
      }
      else
      {
        goal = joint_lqr::utils::LQRGoal(
            this->getReferenceTrajectory()
                ->trajectory_path.poses[lqr_tracking_idx_],
            l_desired_velocity);
        goal_COG = joint_lqr::utils::LQRGoal(
            this->getReferenceTrajectory()->trajectory_path.poses[0],
            l_desired_velocity);
      }
      // std::cout << "debug4-2" << std::endl;

      error = joint_lqr_->computeError(state, goal);
      error_COG = joint_lqr_->computeError(state, goal_COG);
      auto cmd = joint_lqr_->process(state, goal);
      steering_angle_deg = cmd(0, 0) * nif::common::constants::RAD2DEG;
      desired_accel = cmd(1, 0);
      // std::cout << "debug5" << std::endl;
      // Make sure steering angle is within range
      // std::cout << "m_zone_type : " << m_zone_type << std::endl;
      if(m_zone_type == 40)
      {
        final_max_steering_angle_deg_ = m_force_steering_limit;
      }
      else
      {
        final_max_steering_angle_deg_ = max_steering_angle_deg_;
      }
      
      // std::cout << "final_max_steering_angle_deg_ " << final_max_steering_angle_deg_ << std::endl;
      if (steering_angle_deg > final_max_steering_angle_deg_)
        steering_angle_deg = final_max_steering_angle_deg_;
      if (steering_angle_deg < -final_max_steering_angle_deg_)
        steering_angle_deg = -final_max_steering_angle_deg_;

      // ----------------------------------------------------------------------------
      /*
      // APPLY CAMBER COMPENSATION
      */
      double camber_compensatation_deg = 0.0;
      if (this->use_camber_compensator_)
      {

        try
        {
          m_camber_manager_ptr->setVehSpeed(this->current_speed_ms_);
          double tmp_bank_ = 0.0;
          m_camber_manager_ptr->setBankAngle(tmp_bank_);
          camber_compensatation_deg =
              m_camber_manager_ptr->getCamberCompensation();
        }
        catch (std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "CAMBER COMPENSATION EXCEPTION. %s",
                       e.what());
          camber_compensatation_deg = 0.0;
        }
      }
      steering_angle_deg = steering_angle_deg + camber_compensatation_deg;
      
      // publish compensatation_deg
      std_msgs::msg::Float32 camber_compensatation_deg_msg{};
      camber_compensatation_deg_msg.data = camber_compensatation_deg;
      steering_compensate_deg_pub_->publish(camber_compensatation_deg_msg);
      // ----------------------------------------------------------------------------

      // Smooth and publish diagnostics
      // double period_double_s =
      //     nif::common::utils::time::secs(this->getGclockPeriodDuration());
      RCLCPP_DEBUG(this->get_logger(), "Smoothing with dt: [s] %f",
                   period_double_s);
      steering_angle_deg = joint_lqr::utils::smoothSignal(last_steering_command_, steering_angle_deg,
                                                          steering_max_ddeg_dt_, period_double_s);
      // if (std::abs((last_steering_command_ - steering_angle_deg)/period_double_s) > 5.0){
      // std::cout << (last_steering_command_ - steering_angle_deg)/period_double_s << std::endl;
      //   std::cout << period_double_s << std::endl;
      //   }
      if (desired_accel > 0)
      {
        desired_accel = joint_lqr::utils::smoothSignal(last_accel_command_, desired_accel,
                                                       des_accel_max_da_dt_, period_double_s);
      }
      else
      {

        if (oppo_is_back_ && ((this->now() - oppo_is_back_last_update) < rclcpp::Duration(2, 0)))
        {
          if (desired_accel < -std::abs(front_car_max_decel_))
          {
            desired_accel = -std::abs(front_car_max_decel_);
          }
        }
        desired_accel = joint_lqr::utils::smoothSignal(last_accel_command_, desired_accel,
                                                       des_deccel_max_da_dt_, period_double_s);
      }
      // std::cout << (last_accel_command_ - desired_accel)/period_double_s << std::endl;
    }

    geometry_msgs::msg::PoseStamped lqr_track_point;
    if (is_local_path)
    {
      lqr_track_point = this->getReferencePath()->poses[lqr_tracking_idx_];
    }
    else
      lqr_track_point = this->getReferenceTrajectory()->trajectory_path.poses[lqr_tracking_idx_];
    // Publish diagnostic message
    publishSteerAccelDiagnostics(valid_tracking_result, valid_path, valid_odom,
                                 valid_wpt_distance, valid_target_position,
                                 steering_angle_deg, desired_accel,
                                 track_distance, lqr_tracking_idx_,
                                 lqr_track_point,
                                 error_COG, error, l_desired_velocity);
  }
  // std::cout << "debug6" << std::endl;
  if (!this->hasSystemStatus() ||
      (this->getSystemStatus().autonomy_status.lateral_autonomy_enabled ||
       this->getSystemStatus().autonomy_status.longitudinal_autonomy_enabled) &&
          !(((valid_path && valid_odom) || valid_perception_path) && valid_wpt_distance &&
            valid_target_position))
  {
    // std::cout<<valid_path<<valid_odom<<valid_wpt_distance<<valid_target_position<<std::endl;
    node_status = common::NODE_ERROR;
    this->setNodeStatus(node_status);
    return nullptr;
  }

  last_steering_command_ = steering_angle_deg;
  last_accel_command_ = desired_accel;
  // for steering command
  this->control_cmd->steering_control_cmd.data =
      invert_steering_ ? -last_steering_command_ : last_steering_command_;
  // for acceleration command
  // std::cout << "======================" << std::endl;
  if(m_pi_lookup_table)
  {
    if(desired_accel > 0)  //acceleration case
    {
      // std::cout << "acceleration" << std::endl;

      if(current_speed_ms_ < 10.0)
      {
        m_accel_p_gain = accel_p_gain_10mps;
        m_accel_i_gain = accel_i_gain_10mps;
      } else if(current_speed_ms_ < 20.0)
      {
        m_accel_p_gain = accel_p_gain_20mps;
        m_accel_i_gain = accel_i_gain_20mps;
      } else if(current_speed_ms_ < 30.0)
      {
        m_accel_p_gain = accel_p_gain_30mps;
        m_accel_i_gain = accel_i_gain_30mps;
      } else if(current_speed_ms_ < 40.0)
      {
        m_accel_p_gain = accel_p_gain_40mps;
        m_accel_i_gain = accel_i_gain_40mps;
      } else if(current_speed_ms_ < 50.0)
      {
        m_accel_p_gain = accel_p_gain_50mps;
        m_accel_i_gain = accel_i_gain_50mps;
      } else if(current_speed_ms_ < 60.0)
      {
        m_accel_p_gain = accel_p_gain_60mps;
        m_accel_i_gain = accel_i_gain_60mps;
      } else if(current_speed_ms_ < 70.0)
      {
        m_accel_p_gain = accel_p_gain_70mps;
        m_accel_i_gain = accel_i_gain_70mps;
      } else if(current_speed_ms_ < 80.0)
      {
        m_accel_p_gain = accel_p_gain_80mps;
        m_accel_i_gain = accel_i_gain_80mps;
      } else if(current_speed_ms_ < 90.0)
      {
        m_accel_p_gain = accel_p_gain_90mps;
        m_accel_i_gain = accel_i_gain_90mps;
      } 
    } else  //deceleration case
    {
      // std::cout << "deceleration" << std::endl;
      if(current_speed_ms_ < 10.0)
      {
        m_accel_p_gain = decel_p_gain_10mps;
        m_accel_i_gain = decel_i_gain_10mps;
      } else if(current_speed_ms_ < 20.0)
      {
        m_accel_p_gain = decel_p_gain_20mps;
        m_accel_i_gain = decel_i_gain_20mps;
      } else if(current_speed_ms_ < 30.0)
      {
        m_accel_p_gain = decel_p_gain_30mps;
        m_accel_i_gain = decel_i_gain_30mps;
      } else if(current_speed_ms_ < 40.0)
      {
        m_accel_p_gain = decel_p_gain_40mps;
        m_accel_i_gain = decel_i_gain_40mps;
      } else if(current_speed_ms_ < 50.0)
      {
        m_accel_p_gain = decel_p_gain_50mps;
        m_accel_i_gain = decel_i_gain_50mps;
      } else if(current_speed_ms_ < 60.0)
      {
        m_accel_p_gain = decel_p_gain_60mps;
        m_accel_i_gain = decel_i_gain_60mps;
      } else if(current_speed_ms_ < 70.0)
      {
        m_accel_p_gain = decel_p_gain_70mps;
        m_accel_i_gain = decel_i_gain_70mps;
      } else if(current_speed_ms_ < 80.0)
      {
        m_accel_p_gain = decel_p_gain_80mps;
        m_accel_i_gain = decel_i_gain_80mps;
      } else if(current_speed_ms_ < 90.0)
      {
        m_accel_p_gain = decel_p_gain_90mps;
        m_accel_i_gain = decel_i_gain_90mps;
      } 
    }
  }

  // std::cout << "m_accel_p_gain : " << m_accel_p_gain << std::endl;
  // std::cout << "m_accel_i_gain : " << m_accel_i_gain << std::endl;
  error_v = direct_desired_velocity_ - current_speed_ms_;
  double accel_residual = feedback_controller(error_v, current_speed_ms_);

  if (m_use_acc)
  {
    this->control_cmd->desired_accel_cmd.data =
        std::min(desired_accel, acc_accel_cmd_mpss);
  }
  else
  {
    if(std::fabs(this->control_cmd->steering_control_cmd.data) < 2.0)
    {
      this->control_cmd->desired_accel_cmd.data = desired_accel + accel_residual;
    }
    else if(std::fabs(this->control_cmd->steering_control_cmd.data) > 2.0 
            && std::fabs(error_COG(0)) < 0.5)
    {
      this->control_cmd->desired_accel_cmd.data = desired_accel + accel_residual;
      this->control_cmd->desired_accel_cmd.data = 
        0.5*this->control_cmd->desired_accel_cmd.data;
    }
    else
    {
      this->control_cmd->desired_accel_cmd.data = desired_accel + accel_residual;
    }

  }
  // std::cout << "debug7" << std::endl;

  node_status = common::NODE_OK;
  this->setNodeStatus(node_status);
  return this->control_cmd;
}

double ControlLQRNode::feedback_controller(double error, double current_speed_ms_)
{
  error_stack += error;
  if (std::isnan(error_stack))
    error_stack = 0.0;

  if (std::isnan(error))
    error = 0.0;

  error_stack = std::min(error_stack, 30.0);

  if (current_speed_ms_ < 1.0)
  {
    error_stack = 0.;
  }

  if (error < 0.05)
  {
    // P_gain = 0;
    error_stack = 0.;
  }
  double feedback_cmd = m_accel_p_gain * error + m_accel_i_gain * error_stack;

  prev_speed = current_speed_ms_;
  // std::cout << "feedback_cmd : " << feedback_cmd << std::endl;
  return feedback_cmd;

}

void ControlLQRNode::loadConfig(const std::string &config_file_)
{
  RCLCPP_INFO(get_logger(), "Loading planning params: %s",
              config_file_.c_str());
  
  YAML::Node config = YAML::LoadFile(config_file_);

  // if (!config["pi_controller_param"]) {
  //     throw std::runtime_error("pi_controller_param field not defined in config file.");
  // }

  // Load velocity planner params
  YAML::Node config_params_10mps = config["10mps"];
  accel_p_gain_10mps = config_params_10mps["accel_p_gain"].as<double>();
  accel_i_gain_10mps = config_params_10mps["accel_i_gain"].as<double>();
  decel_p_gain_10mps = config_params_10mps["decel_p_gain"].as<double>();
  decel_i_gain_10mps = config_params_10mps["decel_i_gain"].as<double>();

  YAML::Node config_params_20mps = config["20mps"];
  accel_p_gain_20mps = config_params_20mps["accel_p_gain"].as<double>();
  accel_i_gain_20mps = config_params_20mps["accel_i_gain"].as<double>();
  decel_p_gain_20mps = config_params_20mps["decel_p_gain"].as<double>();
  decel_i_gain_20mps = config_params_20mps["decel_i_gain"].as<double>();

  YAML::Node config_params_30mps = config["30mps"];
  accel_p_gain_30mps = config_params_30mps["accel_p_gain"].as<double>();
  accel_i_gain_30mps = config_params_30mps["accel_i_gain"].as<double>();
  decel_p_gain_30mps = config_params_30mps["decel_p_gain"].as<double>();
  decel_i_gain_30mps = config_params_30mps["decel_i_gain"].as<double>();

  YAML::Node config_params_40mps = config["40mps"];
  accel_p_gain_40mps = config_params_40mps["accel_p_gain"].as<double>();
  accel_i_gain_40mps = config_params_40mps["accel_i_gain"].as<double>();
  decel_p_gain_40mps = config_params_40mps["decel_p_gain"].as<double>();
  decel_i_gain_40mps = config_params_40mps["decel_i_gain"].as<double>();

  YAML::Node config_params_50mps = config["50mps"];
  accel_p_gain_50mps = config_params_50mps["accel_p_gain"].as<double>();
  accel_i_gain_50mps = config_params_50mps["accel_i_gain"].as<double>();
  decel_p_gain_50mps = config_params_50mps["decel_p_gain"].as<double>();
  decel_i_gain_50mps = config_params_50mps["decel_i_gain"].as<double>();

  YAML::Node config_params_60mps = config["60mps"];
  accel_p_gain_60mps = config_params_60mps["accel_p_gain"].as<double>();
  accel_i_gain_60mps = config_params_60mps["accel_i_gain"].as<double>();
  decel_p_gain_60mps = config_params_60mps["decel_p_gain"].as<double>();
  decel_i_gain_60mps = config_params_60mps["decel_i_gain"].as<double>();

  YAML::Node config_params_70mps = config["70mps"];
  accel_p_gain_70mps = config_params_70mps["accel_p_gain"].as<double>();
  accel_i_gain_70mps = config_params_70mps["accel_i_gain"].as<double>();
  decel_p_gain_70mps = config_params_70mps["decel_p_gain"].as<double>();
  decel_i_gain_70mps = config_params_70mps["decel_i_gain"].as<double>();

  YAML::Node config_params_80mps = config["80mps"];
  accel_p_gain_80mps = config_params_80mps["accel_p_gain"].as<double>();
  accel_i_gain_80mps = config_params_80mps["accel_i_gain"].as<double>();
  decel_p_gain_80mps = config_params_80mps["decel_p_gain"].as<double>();
  decel_i_gain_80mps = config_params_80mps["decel_i_gain"].as<double>();

  YAML::Node config_params_90mps = config["90mps"];
  accel_p_gain_90mps = config_params_90mps["accel_p_gain"].as<double>();
  accel_i_gain_90mps = config_params_90mps["accel_i_gain"].as<double>();
  decel_p_gain_90mps = config_params_90mps["decel_p_gain"].as<double>();
  decel_i_gain_90mps = config_params_90mps["decel_i_gain"].as<double>();
  
}

void ControlLQRNode::SystemStatusCallback(const nif::common::msgs::SystemStatus::SharedPtr msg)
{
  m_zone_id = msg->mission_status.zone_status.zone_id;
  m_zone_type = msg->mission_status.zone_status.zone_type;
  // mission_number = msg->mission_status.mission_status_code;
  // is_Localized = !(msg->health_status.localization_failure);
}


rcl_interfaces::msg::SetParametersResult
nif::control::ControlLQRNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &vector)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto &param : vector)
  {
    if (param.get_name() == "pure_pursuit_max_max_dist_m")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (param.as_double() >= 8.0 && param.as_double() <= 20.0)
        {
          this->pure_pursuit_max_max_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    }
    else if (param.get_name() == "pure_pursuit_max_dist_m")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (param.as_double() >= 3.0 && param.as_double() <= 20.0)
        {
          this->pure_pursuit_max_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    }
    else if (param.get_name() == "pure_pursuit_min_dist_m")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (param.as_double() >= 3.0 && param.as_double() <= 20.0)
        {
          this->pure_pursuit_min_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    }
    else if (param.get_name() == "pure_pursuit_1st_vel_ms")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (param.as_double() >= 8.0 && param.as_double() <= 40.0)
        {
          this->pure_pursuit_1st_vel_ms_ = param.as_double();
          result.successful = true;
        }
      }
    }
    else if (param.get_name() == "pure_pursuit_k_vel_m_ms")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (param.as_double() >= 0.0 && param.as_double() <= 1.0)
        {
          this->pure_pursuit_k_vel_m_ms_ = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "accel_p_gain") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 40.0) {
          this->m_accel_p_gain = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "accel_i_gain") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 40.0) {
          this->m_accel_i_gain = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "force_steering_limit") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 40.0) {
          this->m_force_steering_limit = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "pi_lookup_table") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) {
          this->m_pi_lookup_table = param.as_bool();
          result.successful = true;
        }
      }
    }
  }
  return result;
}
