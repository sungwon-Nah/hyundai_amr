#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

class IMUProcessor : public rclcpp::Node
{
public:
    IMUProcessor() : Node("yaw_accel_calculator")
    {
        // Subscribe to IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&IMUProcessor::imuCallback, this, std::placeholders::_1));

        // Publisher for yaw acceleration
        yaw_acc_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration", 10);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double current_yaw_rate = -msg->angular_velocity.z;  // For coordinate change
        rclcpp::Time current_time = msg->header.stamp;

        if (first_measurement_)
        {
            last_yaw_rate_ = current_yaw_rate;
            last_time_ = current_time;
            first_measurement_ = false;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt > 0)
        {
            double yaw_acceleration = (current_yaw_rate - last_yaw_rate_) / dt;

            // Publish yaw acceleration
            auto yaw_acc_msg = std_msgs::msg::Float64();
            yaw_acc_msg.data = yaw_acceleration;
            yaw_acc_pub_->publish(yaw_acc_msg);

            // RCLCPP_INFO(this->get_logger(), "Yaw acceleration: %f rad/s^2", yaw_acceleration);
        }

        // Update the last measurements
        last_yaw_rate_ = current_yaw_rate;
        last_time_ = current_time;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acc_pub_;
    double last_yaw_rate_ = 0.0;
    rclcpp::Time last_time_;
    bool first_measurement_ = true;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUProcessor>());
    rclcpp::shutdown();
    return 0;
}
