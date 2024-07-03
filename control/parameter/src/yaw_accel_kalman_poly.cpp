#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>

using std::vector;

class YawRateSmootherNode : public rclcpp::Node {
public:
    YawRateSmootherNode() : Node("yaw_rate_smooth_node") {
        this->declare_parameter<double>("R", 0.01);
        this->declare_parameter<double>("x0", 0.0);
        this->declare_parameter<double>("P0", 1.0);
        this->declare_parameter<double>("Q", 1e-5);
        this->declare_parameter<int>("polynomial_order", 3);

        R = this->get_parameter("R").as_double();
        x0 = this->get_parameter("x0").as_double();
        P0 = this->get_parameter("P0").as_double();
        Q = this->get_parameter("Q").as_double();
        polynomial_order = this->get_parameter("polynomial_order").as_int();

        yaw_rate_original_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_original", 10);
        yaw_rate_kalman_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_kalman", 10);
        yaw_acceleration_kalman_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_kalman", 10);
        yaw_rate_poly_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_rate_kalman_polynomial", 10);
        yaw_acceleration_poly_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_acceleration_kalman_polynomial", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ekf/imu/data", 10, std::bind(&YawRateSmootherNode::imuCallback, this, std::placeholders::_1));

        first_measurement_ = true;
    }

private:
    struct PolynomialFittingCost {
        PolynomialFittingCost(double x, double y, int degree) : x(x), y(y), degree(degree) {}

        template <typename T>
        bool operator()(T const* const* coeffs, T* residual) const {
            T y_pred = T(0.0);
            T x_pow = T(1.0);

            for (int i = 0; i <= degree; ++i) {
                y_pred += coeffs[0][i] * x_pow;
                x_pow *= T(x);
            }

            residual[0] = y_pred - T(y);
            return true;
        }

        double x;
        double y;
        int degree;
    };

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_yaw_rate = -msg->angular_velocity.z;  // Adjusting for coordinate changes
        std_msgs::msg::Float64 yaw_rate_orig_msg;
        yaw_rate_orig_msg.data = current_yaw_rate;
        yaw_rate_original_pub_->publish(yaw_rate_orig_msg);

        rclcpp::Time current_time = msg->header.stamp;

        if (first_measurement_) {
            last_yaw_rate_ = current_yaw_rate;
            last_smooth_yaw_rate_ = current_yaw_rate;
            last_time_ = current_time;
            first_measurement_ = false;
            return;
        }

        double dt = (current_time - last_time_).seconds();
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
            yaw_rate_kalman_pub_->publish(yaw_rate_msg);

            std_msgs::msg::Float64 yaw_acc_msg;
            yaw_acc_msg.data = yaw_acceleration;
            yaw_acceleration_kalman_pub_->publish(yaw_acc_msg);
            // RCLCPP_INFO(this->get_logger(), "Published yaw acceleration: %f rad/s^2", yaw_acceleration);

            data_.emplace_back(current_yaw_rate);
            time_.emplace_back(current_time.seconds());

            if (data_.size() > 100) {
                data_.erase(data_.begin());
                time_.erase(time_.begin());
            }

            if (data_.size() >= static_cast<size_t>(polynomial_order + 1)) {
                std::vector<double> coeffs = polyfit(time_, data_, polynomial_order);
                double smooth_yaw_rate = polyval(coeffs, time_.back());

                std_msgs::msg::Float64 smooth_yaw_rate_msg;
                smooth_yaw_rate_msg.data = smooth_yaw_rate;
                yaw_rate_poly_pub_->publish(smooth_yaw_rate_msg);

                double smooth_yaw_acceleration = (smooth_yaw_rate - last_smooth_yaw_rate_) / dt;
                std_msgs::msg::Float64 smooth_yaw_acc_msg;
                smooth_yaw_acc_msg.data = smooth_yaw_acceleration;
                yaw_acceleration_poly_pub_->publish(smooth_yaw_acc_msg);
                // RCLCPP_INFO(this->get_logger(), "Published yaw acceleration with poly: %f rad/s^2", smooth_yaw_rate);

                last_smooth_yaw_rate_ = smooth_yaw_rate;
            }

            last_yaw_rate_ = filtered_yaw_rate;
            last_time_ = current_time;
        }
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

    std::vector<double> polyfit(const std::vector<double>& x, const std::vector<double>& y, int degree) {
        ceres::Problem problem;
        std::vector<double> coeffs(degree + 1, 0.0);

        for (size_t i = 0; i < x.size(); ++i) {
            ceres::DynamicAutoDiffCostFunction<PolynomialFittingCost>* cost_function =
                new ceres::DynamicAutoDiffCostFunction<PolynomialFittingCost>(
                    new PolynomialFittingCost(x[i], y[i], degree));
            cost_function->AddParameterBlock(degree + 1);
            cost_function->SetNumResiduals(1);
            problem.AddResidualBlock(cost_function, nullptr, coeffs.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        return coeffs;
    }


    double polyval(const std::vector<double>& coeffs, double x) {
        double result = 0.0;
        for (int i = coeffs.size() - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_original_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_kalman_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acceleration_kalman_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_poly_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_acceleration_poly_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    double R, x0, P0, Q;
    int polynomial_order;
    double last_yaw_rate_;
    double last_smooth_yaw_rate_;
    rclcpp::Time last_time_;
    bool first_measurement_;
    std::vector<double> data_;
    std::vector<double> time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawRateSmootherNode>());
    rclcpp::shutdown();
    return 0;
}
