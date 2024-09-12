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

    pnd_four_wheel_cmd_pub = this->create_publisher<pnd_msgs::msg::FourWheelDrive>("/pnd_four_wheel_cmd", 10); //speed: FL, FR, RL, RR, steer: FL, FR, RL, RR
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

void PnDControlNode::publish_four_wheel_cmd(double fl_speed, double fr_speed, double rl_speed, double rr_speed, double fl_steer, double fr_steer, double rl_steer, double rr_steer)
{
  auto msg = pnd_msgs::msg::FourWheelDrive();
  msg.speed.data.push_back(fl_speed);
  msg.speed.data.push_back(fr_speed);
  msg.speed.data.push_back(rl_speed);
  msg.speed.data.push_back(rr_speed);
  msg.steer.data.push_back(fl_steer);
  msg.steer.data.push_back(fr_steer);
  msg.steer.data.push_back(rl_steer);
  msg.steer.data.push_back(rr_steer);
  pnd_four_wheel_cmd_pub->publish(msg);
}

void PnDControlNode::VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(!b_vel_first_callback)
      b_vel_first_callback = true;

    vel_x = msg->linear.x;
    vel_yaw = msg->angular.z;

    if(drive_mode == pnd_msgs::msg::DriveMode::ZERO_TURN) // assuming positive steering angle meaning the servo turning left.
    {
      double fl_steer = 0.0;
      double fr_steer = 0.0;
      double rl_steer = 0.0;
      double rr_steer = 0.0;

      double fl_speed = 0.0;
      double fr_speed = 0.0;
      double rl_speed = 0.0;
      double rr_speed = 0.0;

      if (vel_x != 0.0) {
        RCLCPP_WARN(this->get_logger(), "Linear velocity must be 0. Commnaded linear velocity: %f", vel_x);
        fl_steer = 0.0;
        fr_steer = 0.0;
        rl_steer = 0.0;
        rr_steer = 0.0;

        fl_speed = 0.0;
        fr_speed = 0.0;
        rl_speed = 0.0;
        rr_speed = 0.0;
      }
      else {
        double direction = 0.0;
        double ref_speed = vel_yaw * track_width / sqrt(2);
        double wheel_orientation = 1.0;

        // if (prev_fl_steer > (1/4)*M_PI_2 && prev_fl_steer < (-3/4)*M_PI_2) {
        //   fl_steer = (3/4)*M_PI;
        //   wheel_orientation = -1.0;
        // }
        // else {
        //   fl_steer = -M_PI/4;
        //   wheel_orientation = 1.0;
        // }
        // if (prev_fr_steer > (3/4)*M_PI_2 && prev_fr_steer < (-1/4)*M_PI_2) {
        //   fr_steer = (-3/4)*M_PI;
        //   wheel_orientation = -1.0;
        // }
        // else {
        //   fr_steer = M_PI/4;
        //   wheel_orientation = 1.0;
        // }
        // if (prev_rl_steer > (3/4)*M_PI_2 && prev_rl_steer < (-1/4)*M_PI_2) {
        //   rl_steer = (-3/4)*M_PI;
        //   wheel_orientation = -1.0;
        // }
        // else {
        //   rl_steer = M_PI/4;
        //   wheel_orientation = 1.0;
        // }
        // if (prev_rr_steer > (1/4)*M_PI_2 && prev_rr_steer < (-3/4)*M_PI_2) {
        //   rr_steer = (3/4)*M_PI;
        //   wheel_orientation = -1.0;
        // }
        // else {
        //   rr_steer = -M_PI/4;
        //   wheel_orientation = 1.0;
        // }

        fl_steer = -M_PI/4;
        fr_steer = M_PI/4;
        rl_steer = M_PI/4;
        rr_steer = -M_PI/4;

        if (vel_yaw < 0.0) {
          direction = -1.0;
        }
        else {
          direction = 1.0;
        }

        fl_speed = -direction*wheel_orientation*ref_speed;
        fr_speed = direction*wheel_orientation*ref_speed;
        rl_speed = -direction*wheel_orientation*ref_speed;
        rr_speed = direction*wheel_orientation*ref_speed;

        publish_four_wheel_cmd(fl_speed, fr_speed, rl_speed, rr_speed, fl_steer, fr_steer, rl_steer, rr_steer);

        // prev_fl_steer = fl_steer;
        // prev_fr_steer = fr_steer;
        // prev_rl_steer = rl_steer;
        // prev_rr_steer = rr_steer; 
      }
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
      
      publish_four_wheel_cmd(fl_speed, fr_speed, rl_speed, rr_speed, fl_steer, fr_steer, rl_steer, rr_steer);

      prev_fl_steer = fl_steer;
      prev_fr_steer = fr_steer;
      prev_rl_steer = rl_steer;
      prev_rr_steer = rr_steer; 
    }
    else if (drive_mode == pnd_msgs::msg::DriveMode::ACKERMANN)
    {
      double fl_steer = 0.0;
      double fr_steer = 0.0;
      double rl_steer = 0.0;
      double rr_steer = 0.0;

      double fl_speed = 0.0;
      double fr_speed = 0.0;
      double rl_speed = 0.0;
      double rr_speed = 0.0;

      double L = total_wheelbase_length;
      double W = track_width;
    
      double v = vel_x; 
      double omega = vel_yaw;

      // Assume no slip
      double delta = atan2(L * omega, v);

      // Constrain delta within (-90, 90)
      if (delta > M_PI_2 || delta < -M_PI_2) {
        RCLCPP_WARN(this->get_logger(), "Calculated delta angle is out of range: %f", delta);
      }

      // Constrain delta within (-30, 30) degrees
      if (delta > M_PI / 6) {
        delta = M_PI / 6;
      } else if (delta < -M_PI / 6) {
        delta = -M_PI / 6;
      }

      // Update steering angle at each wheel
      fl_steer = atan2(L * tan(delta), (L + 0.5 * W * tan(delta)));
      fr_steer = atan2(L * tan(delta), (L - 0.5 * W * tan(delta)));
      rl_steer = 0.0; // Ackermann
      rr_steer = 0.0; // Ackermann

      // Update velocity at each wheel
      rl_speed = 0; // RWD
      rr_speed = 0; // RWD
      rl_speed = (L + 0.5 * W * tan(delta)) / L * v;
      rr_speed = (L - 0.5 * W * tan(delta)) / L * v;

      publish_four_wheel_cmd(fl_speed, fr_speed, rl_speed, rr_speed, fl_steer, fr_steer, rl_steer, rr_steer);

      prev_fl_steer = fl_steer;
      prev_fr_steer = fr_steer;
      prev_rl_steer = rl_steer;
      prev_rr_steer = rr_steer;
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

    publish_four_wheel_cmd(fl_speed, fr_speed, rl_speed, rr_speed, fl_steer, fr_steer, rl_steer, rr_steer);
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