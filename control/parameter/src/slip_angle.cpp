#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

class SlipAngleCalculator : public rclcpp::Node
{
public:
    SlipAngleCalculator() : Node("slip_angle_calculator"), lf_(1.155), lr_(1.815) // Set lf, lr value [m] - IONNIQ 5 : 1.155 , 1.815 / G80EV : ? , ?
    {
        odom_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/ekf/velocity", 10,
            std::bind(&SlipAngleCalculator::odomCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10,
            std::bind(&SlipAngleCalculator::imuCallback, this, std::placeholders::_1));

        slip_angle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/slip_angles", 10);
    }

private:
    void odomCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        vx_ = msg->twist.twist.linear.x;
        vy_ = -msg->twist.twist.linear.y;  // For coordinate change
        calculateAndPublishSlipAngles();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        gamma_ = -msg->angular_velocity.z;  // For coordinate change
        calculateAndPublishSlipAngles();
    }

    void calculateAndPublishSlipAngles()
    {
        if (vx_ == 0) return;  // Avoid division by zero

        double theta_vf = std::atan((vy_ + (lf_ * gamma_)) / vx_);
        double theta_vr = std::atan((vy_ - (lr_ * gamma_)) / vx_);

        std_msgs::msg::Float64MultiArray slip_angles;
        slip_angles.data.push_back(theta_vf);
        slip_angles.data.push_back(theta_vr);

        slip_angle_pub_->publish(slip_angles);

        RCLCPP_INFO(this->get_logger(), "Published slip angles: Front = %f, Rear = %f", theta_vf, theta_vr);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr slip_angle_pub_;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double gamma_ = 0.0;
    double lf_;
    double lr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlipAngleCalculator>());
    rclcpp::shutdown();
    return 0;
}
