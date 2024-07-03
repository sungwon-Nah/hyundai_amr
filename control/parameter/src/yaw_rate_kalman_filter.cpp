#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>

using std::vector;

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("yaw_rate_kalman_filter") {
        this->declare_parameter<double>("R", 0.01);
        this->declare_parameter<double>("x0", 0.0);
        this->declare_parameter<double>("P0", 1.0);
        this->declare_parameter<double>("Q", 1e-5);

        R = this->get_parameter("R").as_double();
        x0 = this->get_parameter("x0").as_double();
        P0 = this->get_parameter("P0").as_double();
        Q = this->get_parameter("Q").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_Kalman", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&KalmanFilterNode::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double yaw_rate = -msg->angular_velocity.z;
        vector<double> measurement{yaw_rate};
        vector<double> estimates = runKalmanFilter(measurement, R, x0, P0, Q);
        double filtered_yaw_rate = estimates.back();

        std_msgs::msg::Float64 yaw_rate_msg;
        yaw_rate_msg.data = filtered_yaw_rate;
        publisher_->publish(yaw_rate_msg);
    }

    vector<double> runKalmanFilter(const vector<double>& measurements, double R, double x0, double P0, double Q) {
        static double x_estimate = x0;
        static double P_estimate = P0;
        vector<double> x_estimates;

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

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    double R, x0, P0, Q;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
