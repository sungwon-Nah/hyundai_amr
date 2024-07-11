#include "pnd_controller_nodes/pnd_controller_node.h"

PnDControlNode::PnDControlNode()
    : Node("PnD_Control_node") { 

    RCLCPP_INFO(this->get_logger(), "Inititalizing PnD Controller node...");
    // Load Parameters

    this->declare_parameter<double>("height_of_cg", 0.7);
    this->declare_parameter<double>("total_wheelbase_length", 3.0);
    this->declare_parameter<double>("front_wheelbase_length", 1.155);
    this->declare_parameter<double>("rear_wheelbase_length", 1.815);
    this->declare_parameter<double>("track_width", 0.2);
    this->declare_parameter<double>("gravity_acceleration", 9.80665);
    this->declare_parameter<double>("total_mass", 2065.03);
    this->declare_parameter<double>("moment_of_inertia", 2900.3);
    this->declare_parameter<double>("friction_coefficient", 1.0);
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
    // this->test_mode = this->get_parameter("test_mode").as_bool();


    pnd_steer_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pnd_steer_cmd", 10); //FL, FR, RL, RR
    pnd_speed_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pnd_speed_cmd", 10); //FL, FR, RL, RR
    pnd_drivemode_pub = this->create_publisher<pnd_msgs::msg::DriveMode>("/pnd_drive_mode", 10);

    vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      std::bind(&PnDControlNode::VelCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PnDControlNode::timerCallback, this));

    this->service_drive_mode = this->create_service<pnd_msgs::srv::SetDriveMode>(
      "/pnd_controller/drive_mode",
      std::bind(
          &PnDControlNode::service_handler_dvive_mode,
          this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

}

///////////////////////////////////////callback

void PnDControlNode::timerCallback()
{
    pnd_msgs::msg::DriveMode drivemode_msg;
    drivemode_msg.mode = drive_mode;
    pnd_drivemode_pub->publish(drivemode_msg);

}

void PnDControlNode::publish_speed_cmd(double fl_speed, double fr_speed, double rl_speed, double rr_speed)
{
  std_msgs::msg::Float32MultiArray pnd_speed_array;
  pnd_speed_array.data.push_back(fl_speed);
  pnd_speed_array.data.push_back(fr_speed);
  pnd_speed_array.data.push_back(rl_speed);
  pnd_speed_array.data.push_back(rr_speed);
  pnd_speed_pub->publish(pnd_speed_array);
}

void PnDControlNode::publish_steer_cmd(double fl_steer, double fr_steer, double rl_steer, double rr_steer)
{
  std_msgs::msg::Float32MultiArray pnd_steer_array;
  pnd_steer_array.data.push_back(fl_steer);
  pnd_steer_array.data.push_back(fr_steer);
  pnd_steer_array.data.push_back(rl_steer);
  pnd_steer_array.data.push_back(rr_steer);
  pnd_steer_pub->publish(pnd_steer_array);
}


void PnDControlNode::VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(!b_vel_first_callback)
      b_vel_first_callback = true;

    vel_x = msg->linear.x;
    vel_yaw = msg->angular.z;

    if(drive_mode == pnd_msgs::msg::DriveMode::ZERO_TURN)
    {
      // SJ
    }
    else if(drive_mode == pnd_msgs::msg::DriveMode::DIFFERENTIAL)
    {
      double ref_R = vel_x + vel_yaw * track_width / 2.0;
      double ref_L = vel_x - vel_yaw * track_width / 2.0;

      double fl_speed = 0.0;
      double fr_speed = 0.0;
      double rl_speed = 0.0;
      double rr_speed = 0.0;

      if(ref_R < 0.0)
      {
        fr_speed = 0.0;
        rr_speed = 0.0;
      }
      else
      {
        fr_speed = ref_R;
        rr_speed = ref_R;
      }

      if(ref_L < 0.0)
      {
        fl_speed = 0.0;
        rl_speed = 0.0;
      }
      else
      {
        fl_speed = ref_L;
        rl_speed = ref_L;
      }

      double fl_steer = 0.0;
      double fr_steer = 0.0;
      double rl_steer = 0.0;
      double rr_steer = 0.0;

      publish_speed_cmd(fl_speed, fr_speed, rl_speed, rr_speed);
      publish_steer_cmd(fl_steer, fr_steer, rl_steer, rr_steer);

      
    }
    else if(drive_mode == pnd_msgs::msg::DriveMode::ACKERMANN)
    {
      //JH
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Wrong Drive Mode = %d", drive_mode);

      double fl_speed = 0.0;
      double fr_speed = 0.0;
      double rl_speed = 0.0;
      double rr_speed = 0.0;

      double fl_steer = 0.0;
      double fr_steer = 0.0;
      double rl_steer = 0.0;
      double rr_steer = 0.0;

      publish_speed_cmd(fl_speed, fr_speed, rl_speed, rr_speed);
      publish_steer_cmd(fl_steer, fr_steer, rl_steer, rr_steer);

    }
     
}



void PnDControlNode::service_handler_dvive_mode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const pnd_msgs::srv::SetDriveMode::Request::SharedPtr request,
    pnd_msgs::srv::SetDriveMode::Response::SharedPtr response)
{
  drive_mode = request->mode_request;

  response->success = true;
  response->message = "OK: Drive Mode is Set!";
  response->current_mode = request->mode_request;
  return;
  
}