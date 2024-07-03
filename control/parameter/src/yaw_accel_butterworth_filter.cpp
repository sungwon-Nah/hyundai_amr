#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include "butterworth.h"

class YawAccCalculator : public rclcpp::Node {
public:
    YawAccCalculator() : Node("yaw_acceleration_butterworth_filter"), prev_time_(0), prev_yaw_rate_(0) {
        this->declare_parameter<int>("filter_order", 2);
        this->declare_parameter<double>("cutoff_frequency", 5.0);
        this->declare_parameter<double>("sampling_rate", 100.0);
        int filter_order = this->get_parameter("filter_order").as_int();
        double cutoff_frequency = this->get_parameter("cutoff_frequency").as_double();
        double sampling_rate = this->get_parameter("sampling_rate").as_double();

        bf_.init(filter_order, cutoff_frequency, sampling_rate);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&YawAccCalculator::imuCallback, this, std::placeholders::_1));

        yaw_acc_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_butterworth", 10);
        filtered_yaw_rate_pub_ = this->create_publisher<std_msgs::msg::Float64>("/filtered_yaw_rate_butterworth", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double yaw_rate = -msg->angular_velocity.z;

        double filtered_yaw_rate = bf_.filter(yaw_rate);

        // Publish the filtered yaw rate
        std_msgs::msg::Float64 filtered_yaw_rate_msg;
        filtered_yaw_rate_msg.data = filtered_yaw_rate;
        filtered_yaw_rate_pub_->publish(filtered_yaw_rate_msg);

        if (prev_time_ != 0) {
            double dt = current_time - prev_time_;
            if (dt > 0) {
                double yaw_acceleration = (filtered_yaw_rate - prev_yaw_rate_) / dt;

                std_msgs::msg::Float64 yaw_acc_msg;
                yaw_acc_msg.data = yaw_acceleration;
                yaw_acc_pub_->publish(yaw_acc_msg);
            }
        }

        prev_time_ = current_time;
        prev_yaw_rate_ = filtered_yaw_rate;
    }

    ButterworthFilter bf_;
    double prev_time_;
    double prev_yaw_rate_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_yaw_rate_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawAccCalculator>());
    rclcpp::shutdown();
    return 0;
}
