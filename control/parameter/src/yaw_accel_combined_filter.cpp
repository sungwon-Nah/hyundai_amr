#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>
#include <memory>
#include "butterworth.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t window_size) : window_size_(window_size) {}

    double filter(double new_sample) {
        if (samples_.size() == window_size_) {
            total_sum_ -= samples_.front();
            samples_.pop_front();
        }
        samples_.push_back(new_sample);
        total_sum_ += new_sample;
        return total_sum_ / samples_.size();
    }

private:
    std::deque<double> samples_;
    double total_sum_ = 0.0;
    size_t window_size_;
};

class ImuFilterNode : public rclcpp::Node {
public:
    ImuFilterNode() : Node("yaw_acceleration_combined_filter"), moving_average_filter_(std::make_unique<MovingAverageFilter>(10)), last_yaw_rate_(0.0) {
        this->declare_parameter<int>("filter_order", 2);
        this->declare_parameter<double>("cutoff_frequency", 5.0);
        this->declare_parameter<double>("sampling_rate", 100.0);
        this->declare_parameter<int>("window_size", 10);

        int filter_order = this->get_parameter("filter_order").as_int();
        double cutoff_frequency = this->get_parameter("cutoff_frequency").as_double();
        double sampling_rate = this->get_parameter("sampling_rate").as_double();
        int window_size = this->get_parameter("window_size").as_int();

        moving_average_filter_ = std::make_unique<MovingAverageFilter>(window_size);
        butterworth_filter_.init(filter_order, cutoff_frequency, sampling_rate);

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&ImuFilterNode::imu_callback, this, _1));

        filtered_yaw_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/filtered_yaw_rate_combined", 10);
        yaw_acceleration_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_combined", 10);

        last_time_ = this->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        if (last_time_ == rclcpp::Time(0)) {
            last_time_ = current_time;
            return;
        }

        rclcpp::Duration dt = current_time - last_time_;
        if (dt.seconds() <= 0) {
            RCLCPP_WARN(this->get_logger(), "Non-positive dt computed, skipping this iteration.");
            return;
        }

        double yaw_rate = -msg->angular_velocity.z;

        double butter_filtered = butterworth_filter_.filter(yaw_rate);
        double combined_filtered = moving_average_filter_->filter(butter_filtered);

        std_msgs::msg::Float64 filtered_yaw_rate_msg;
        filtered_yaw_rate_msg.data = combined_filtered;
        filtered_yaw_rate_publisher_->publish(filtered_yaw_rate_msg);

        double yaw_acceleration = (combined_filtered - last_yaw_rate_) / dt.seconds();
        std_msgs::msg::Float64 yaw_acc_msg;
        yaw_acc_msg.data = yaw_acceleration;
        yaw_acceleration_publisher_->publish(yaw_acc_msg);

        last_yaw_rate_ = combined_filtered;
        last_time_ = current_time;
    }

    ButterworthFilter butterworth_filter_;
    std::unique_ptr<MovingAverageFilter> moving_average_filter_;
    double last_yaw_rate_;
    rclcpp::Time last_time_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_yaw_rate_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acceleration_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFilterNode>());
    rclcpp::shutdown();
    return 0;
}
