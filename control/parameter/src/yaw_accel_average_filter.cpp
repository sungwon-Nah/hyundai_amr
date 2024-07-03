#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>

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

class ImuYawAccelerationFilter : public rclcpp::Node {
public:
    ImuYawAccelerationFilter() : Node("yaw_acceleration_average_filter"), last_yaw_rate_(0.0) {
        this->declare_parameter<int>("window_size", 10);
        int window_size = this->get_parameter("window_size").as_int();
        filter_ = std::make_unique<MovingAverageFilter>(window_size);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&ImuYawAccelerationFilter::imu_callback, this, _1));
        
        yaw_acc_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_average", 10);
        filtered_yaw_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/filtered_yaw_rate_average", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_yaw_rate = -msg->angular_velocity.z;
        rclcpp::Time current_time = msg->header.stamp;

        double filtered_yaw_rate = filter_->filter(current_yaw_rate);

        // Publish the filtered yaw rate
        std_msgs::msg::Float64 filtered_yaw_rate_msg;
        filtered_yaw_rate_msg.data = filtered_yaw_rate;
        filtered_yaw_rate_publisher_->publish(filtered_yaw_rate_msg);

        if (last_time_.nanoseconds() != 0) {
            double dt = (current_time - last_time_).seconds();
            if (dt > 0) {
                double yaw_acceleration = (filtered_yaw_rate - last_yaw_rate_) / dt;
                std_msgs::msg::Float64 output;
                output.data = yaw_acceleration;
                yaw_acc_publisher_->publish(output);
            }
        }

        last_yaw_rate_ = filtered_yaw_rate;
        last_time_ = current_time;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acc_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_yaw_rate_publisher_;
    std::unique_ptr<MovingAverageFilter> filter_;
    double last_yaw_rate_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuYawAccelerationFilter>());
    rclcpp::shutdown();
    return 0;
}
