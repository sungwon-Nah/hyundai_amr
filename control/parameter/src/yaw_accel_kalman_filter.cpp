#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>

using std::vector;

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("yaw_acceleration_kalman_filter") {
        this->declare_parameter<double>("R", 0.01);
        this->declare_parameter<double>("x0", 0.0);
        this->declare_parameter<double>("P0", 1.0);
        this->declare_parameter<double>("Q", 1e-5);

        R = this->get_parameter("R").as_double();
        x0 = this->get_parameter("x0").as_double();
        P0 = this->get_parameter("P0").as_double();
        Q = this->get_parameter("Q").as_double();

        yaw_rate_original_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_original", 10);
        yaw_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_kalman", 10);
        yaw_acceleration_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_kalman", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&KalmanFilterNode::imuCallback, this, std::placeholders::_1));

        first_measurement_ = true;
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_yaw_rate = -msg->angular_velocity.z;  // Adjusting for coordinate changes
        std_msgs::msg::Float64 yaw_rate_orig_msg;
        yaw_rate_orig_msg.data = current_yaw_rate;
        yaw_rate_original_pub_->publish(yaw_rate_orig_msg);
        
        rclcpp::Time current_time = msg->header.stamp;

        if (first_measurement_) {
            last_yaw_rate_ = current_yaw_rate;
            last_time_ = current_time;
            first_measurement_ = false;
            return;
        }

        // double dt = (current_time - last_time_).seconds();
        double dt = 0.01;
        // RCLCPP_INFO(this->get_logger(), "dt: %f", dt);
        if (dt <= 0) {
            RCLCPP_INFO(this->get_logger(), "dt: %f", dt);
        }
        if (dt > 0) {
            vector<double> measurement{current_yaw_rate};
            vector<double> estimates = runKalmanFilter(measurement, R, x0, P0, Q);
            double filtered_yaw_rate = !estimates.empty() ? estimates.back() : current_yaw_rate;

            double yaw_acceleration = (filtered_yaw_rate - last_yaw_rate_) / dt;

            std_msgs::msg::Float64 yaw_rate_msg;
            yaw_rate_msg.data = filtered_yaw_rate;
            yaw_rate_publisher_->publish(yaw_rate_msg);

            std_msgs::msg::Float64 yaw_acc_msg;
            yaw_acc_msg.data = yaw_acceleration;
            yaw_acceleration_publisher_->publish(yaw_acc_msg);

            // RCLCPP_INFO(this->get_logger(), "Published yaw acceleration: %f rad/s^2", yaw_acceleration);

            // Update the last measurements
            last_yaw_rate_ = filtered_yaw_rate;
            last_time_ = current_time;
        }
    }

    vector<double> runKalmanFilter(const vector<double>& measurements, double R, double x0, double P0, double Q) {
        static double x_estimate = x0;
        static double P_estimate = P0;
        vector<double> x_estimates;

        // 이건 low pass filter -> 칼만 필터 사용하도록 바꿔야 함. (prediction을 전 estimation을 사용 안하게)
        for (double z : measurements) {
            double x_prediction = x_estimate;
            double P_prediction = P_estimate + Q;
            double K = P_prediction / (P_prediction + R);
            x_estimate = x_prediction + K * (z - x_prediction);
            P_estimate = (1 - K) * P_prediction;
            x_estimates.push_back(x_estimate);
        }
        return x_estimates;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_original_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acceleration_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    double R, x0, P0, Q;
    double last_yaw_rate_;
    rclcpp::Time last_time_;
    bool first_measurement_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
