#include "aero_controller_nodes/aero_controller_node.h"

AeroControlNode::AeroControlNode()
    : Node("Aero_Control_node") { 

    RCLCPP_INFO(this->get_logger(), "Inititalizing Aero Controller node...");
    // Load Parameters

    this->declare_parameter<double>("height_of_cg", 0.7);
    this->declare_parameter<double>("total_wheelbase_length", 3.0);
    this->declare_parameter<double>("front_wheelbase_length", 1.155);
    this->declare_parameter<double>("rear_wheelbase_length", 1.815);
    this->declare_parameter<double>("track_width", 1.64);
    this->declare_parameter<double>("gravity_acceleration", 9.80665);
    this->declare_parameter<double>("total_mass", 2065.03);
    this->declare_parameter<double>("moment_of_inertia", 2900.3);
    this->declare_parameter<double>("friction_coefficient", 1.0);
    this->declare_parameter<double>("hysteresis_band", 1.0);
    // this->declare_parameter<bool>("test_mode", true);
   

    this->height_of_cg = this->get_parameter("height_of_cg").as_double();
    this->total_wheelbase_length = this->get_parameter("total_wheelbase_length").as_double();
    this->front_wheelbase_length = this->get_parameter("front_wheelbase_length").as_double();
    this->rear_wheelbase_length = this->get_parameter("rear_wheelbase_length").as_double();
    this->track_width = this->get_parameter("track_width").as_double();
    this->gravity_acceleration = this->get_parameter("gravity_acceleration").as_double();
    this->total_mass = this->get_parameter("total_mass").as_double();
    this->moment_of_inertia = this->get_parameter("moment_of_inertia").as_double();
    this->friction_coefficient = this->get_parameter("friction_coefficient").as_double();
    this->hysteresis_band = this->get_parameter("hysteresis_band").as_double();
    // this->test_mode = this->get_parameter("test_mode").as_bool();

    FL_activate_pub = this->create_publisher<std_msgs::msg::Bool>("/FL_activate", 10);
    FR_activate_pub = this->create_publisher<std_msgs::msg::Bool>("/FR_activate", 10);
    RL_activate_pub = this->create_publisher<std_msgs::msg::Bool>("/RL_activate", 10);
    RR_activate_pub = this->create_publisher<std_msgs::msg::Bool>("/RR_activate", 10);

    F_z_est_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/F_z_est", 10);
    F_z_req_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/F_z_req", 10);
  
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/ekf/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&AeroControlNode::ImuCallback, this, std::placeholders::_1));

    vel_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/ekf/velocity", rclcpp::SensorDataQoS(),
      std::bind(&AeroControlNode::VelCallback, this, std::placeholders::_1));

    // yaw_accel_sub = this->create_subscription<std_msgs::msg::Float64>(
    //   "/yaw_acceleration_combined", rclcpp::SensorDataQoS(),
    //   std::bind(&AeroControlNode::YawAccelCallback, this, std::placeholders::_1));

    yaw_accel_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/yaw_acceleration_kalman", rclcpp::SensorDataQoS(),
      std::bind(&AeroControlNode::YawAccelCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AeroControlNode::timerCallback, this));

    // auto rmw_qos_profile = nif::common::constants::QOS_SENSOR_DATA.get_rmw_qos_profile();

    // sub_filtered_Image.subscribe(this, imgTopic, rmw_qos_profile);
    // sub_filtered_Points.subscribe(this, pcTopic, rmw_qos_profile);

    // m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
    //   SyncPolicyT(10), sub_filtered_Image, sub_filtered_Points);
    // m_sync->registerCallback(std::bind(&AeroControlNode::MessageFilteringCallback,
    //                                  this, std::placeholders::_1,
    //                                  std::placeholders::_2));

}

///////////////////////////////////////callback

void AeroControlNode::timerCallback()
{
    double curr_time = static_cast<double>(this->now().seconds());

    if(b_imu_first_callback && b_vel_first_callback && b_yaw_accel_first_callback)
    {
      // double F_z_fl = 0.0, F_z_fr = 0.0, F_z_rl = 0.0, F_z_rr = 0.0;

      std::vector<double> F_z_estimated = CalculateLoadTransfer(); // F_z_fl[0], F_z_fr[1], F_z_rl[2], F_z_rr[3]

      std::vector<double> F_y_required = CalculateLatTireForce(F_z_estimated);

      std::vector<double> F_x_required = CalculateLonTireForce();

      std::vector<aero_control_cmd> control_cmd = AeroControlCmd(F_z_estimated, F_x_required, F_y_required);

      for(int i = 0; i < control_cmd.size(); i++)
      {
        if(i == 0 && control_cmd.at(i).id == i)
        {
          std_msgs::msg::Bool b_FL_activate;
          b_FL_activate.data = control_cmd.at(i).b_activate;
          FL_activate_pub->publish(b_FL_activate);
        }
        else if(i == 1 && control_cmd.at(i).id == i)
        {
          std_msgs::msg::Bool b_FR_activate;
          b_FR_activate.data = control_cmd.at(i).b_activate;
          FR_activate_pub->publish(b_FR_activate);
        }
        else if(i == 2 && control_cmd.at(i).id == i)
        {
          std_msgs::msg::Bool b_RL_activate;
          b_RL_activate.data = control_cmd.at(i).b_activate;
          RL_activate_pub->publish(b_RL_activate);
        }
        else if(i == 3 && control_cmd.at(i).id == i)
        {
          std_msgs::msg::Bool b_RR_activate;
          b_RR_activate.data = control_cmd.at(i).b_activate;
          RR_activate_pub->publish(b_RR_activate);
        }
        else{

        }
      }
    }

}

void AeroControlNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if(!b_imu_first_callback)
      b_imu_first_callback = true;

    yaw_rate = -msg->angular_velocity.z;
    accel_x = msg->linear_acceleration.x;
    accel_y = -msg->linear_acceleration.y;
     
}

void AeroControlNode::VelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if(!b_vel_first_callback)
      b_vel_first_callback = true;

    vel_x = msg->twist.twist.linear.x;
    vel_y = -msg->twist.twist.linear.y;
     
}

void AeroControlNode::YawAccelCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if(!b_yaw_accel_first_callback)
      b_yaw_accel_first_callback = true;

    yaw_accel = msg->data;
}

std::vector<double> AeroControlNode::CalculateLoadTransfer()
{
  std::vector<double> output;

  double F_z_fl = (total_mass * gravity_acceleration * rear_wheelbase_length / (2.0 * total_wheelbase_length))
         - (total_mass * accel_x * height_of_cg / (2.0 * total_wheelbase_length))
         - (total_mass * accel_y * height_of_cg / (2.0 * track_width));

  output.push_back(F_z_fl);

  double F_z_fr = (total_mass * gravity_acceleration * rear_wheelbase_length / (2.0 * total_wheelbase_length))
         - (total_mass * accel_x * height_of_cg / (2.0 * total_wheelbase_length))
         + (total_mass * accel_y * height_of_cg / (2.0 * track_width));

  output.push_back(F_z_fr);

  double F_z_rl = (total_mass * gravity_acceleration * front_wheelbase_length / (2.0 * total_wheelbase_length))
         + (total_mass * accel_x * height_of_cg / (2.0 * total_wheelbase_length))
         - (total_mass * accel_y * height_of_cg / (2.0 * track_width));

  output.push_back(F_z_rl);

  double F_z_rr = (total_mass * gravity_acceleration * front_wheelbase_length / (2.0 * total_wheelbase_length))
         + (total_mass * accel_x * height_of_cg / (2.0 * total_wheelbase_length))
         + (total_mass * accel_y * height_of_cg / (2.0 * track_width));

  output.push_back(F_z_rr);

  return output;
}

std::vector<double> AeroControlNode::CalculateLatTireForce(const std::vector<double> F_z_estimated)
{
  std::vector<double> output;

  double F_y_f = (total_mass * rear_wheelbase_length * accel_y + moment_of_inertia * yaw_accel) / (front_wheelbase_length + rear_wheelbase_length);
  double F_y_r = (total_mass * front_wheelbase_length * accel_y - moment_of_inertia * yaw_accel) / (front_wheelbase_length + rear_wheelbase_length);

  // Add Centripetal Accel term
  // double F_y_f = (total_mass * rear_wheelbase_length * accel_y + total_mass * rear_wheelbase_length * vel_x * yaw_rate + moment_of_inertia * yaw_accel) / (front_wheelbase_length + rear_wheelbase_length);
  // double F_y_r = (total_mass * front_wheelbase_length * accel_y + total_mass * rear_wheelbase_length * vel_x * yaw_rate - moment_of_inertia * yaw_accel) / (front_wheelbase_length + rear_wheelbase_length);

  double F_y_fl = (F_z_estimated.at(0) / (F_z_estimated.at(0) + F_z_estimated.at(1))) * F_y_f;
  output.push_back(F_y_fl);

  double F_y_fr = (F_z_estimated.at(1) / (F_z_estimated.at(0) + F_z_estimated.at(1))) * F_y_f;
  output.push_back(F_y_fr);

  double F_y_rl = (F_z_estimated.at(2) / (F_z_estimated.at(2) + F_z_estimated.at(3))) * F_y_r;
  output.push_back(F_y_rl);

  double F_y_rr = (F_z_estimated.at(3) / (F_z_estimated.at(2) + F_z_estimated.at(3))) * F_y_r;
  output.push_back(F_y_rr);

  return output;
}

std::vector<double> AeroControlNode::CalculateLonTireForce()
{
  std::vector<double> output;

  double F_x_fl = 0.0;
  output.push_back(F_x_fl);

  double F_x_fr = 0.0;
  output.push_back(F_x_fr);

  double F_x_rl = total_mass * accel_x / 2.0;
  output.push_back(F_x_rl);

  double F_x_rr = total_mass * accel_x / 2.0;
  output.push_back(F_x_rr);

  return output;
}

std::vector<aero_control_cmd> AeroControlNode::AeroControlCmd(const std::vector<double> F_z_estimated, const std::vector<double> F_x_required, const std::vector<double> F_y_required)
{
  std::vector<aero_control_cmd> output;
  std::vector<double> F_z_req_array;

  // for(int i = 0; i < F_z_estimated.size(); i++)
  // {
  //   double F_z_req = sqrt(pow(F_x_required.at(i)/friction_coefficient, 2) + pow(F_y_required.at(i)/friction_coefficient, 2));
  //   std::cout<<"F_z_req: "<<F_z_req<<std::endl;
  //   F_z_req_array.push_back(F_z_req);

  //   aero_control_cmd tmp; 

  //   if(F_z_req > F_z_estimated.at(i))
  //     tmp.b_activate = true;
  //   else
  //     tmp.b_activate = false;

  //   tmp.id = i;

  //   output.push_back(tmp);
  // }

  if (current_state.size() != F_z_estimated.size()) {
        current_state.resize(F_z_estimated.size(), false); // Initialize current state to false if not already initialized
  }

  for (int i = 0; i < F_z_estimated.size(); i++) {
      double F_z_req = sqrt(pow(F_x_required.at(i) / friction_coefficient, 2) + pow(F_y_required.at(i) / friction_coefficient, 2));
      F_z_req_array.push_back(F_z_req);
      aero_control_cmd tmp;
      if (!current_state[i] && F_z_req > F_z_estimated.at(i) + hysteresis_band) {
          tmp.b_activate = true;
      } else if (current_state[i] && F_z_req < F_z_estimated.at(i) - hysteresis_band) {
          tmp.b_activate = false;
      } else {
          tmp.b_activate = current_state[i]; // Maintain current state
      }
      current_state[i] = tmp.b_activate;
      tmp.id = i;
      output.push_back(tmp);
  }

  std_msgs::msg::Float32MultiArray F_z_est_results;
  F_z_est_results.data.push_back(F_z_estimated.at(0));
  F_z_est_results.data.push_back(F_z_estimated.at(1));
  F_z_est_results.data.push_back(F_z_estimated.at(2));
  F_z_est_results.data.push_back(F_z_estimated.at(3));
  F_z_est_pub->publish(F_z_est_results);

  std_msgs::msg::Float32MultiArray F_z_req_results;
  F_z_req_results.data.push_back(F_z_req_array.at(0));
  F_z_req_results.data.push_back(F_z_req_array.at(1));
  F_z_req_results.data.push_back(F_z_req_array.at(2));
  F_z_req_results.data.push_back(F_z_req_array.at(3));
  F_z_req_pub->publish(F_z_req_results);

  return output;
}