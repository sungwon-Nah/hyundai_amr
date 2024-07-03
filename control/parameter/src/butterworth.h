#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#include <cmath>
#include <vector>

class ButterworthFilter {
public:
    ButterworthFilter() : order(0), cutoff_frequency(0.0), sampling_rate(0.0) {}

    void init(int order, double cutoff_frequency, double sampling_rate) {
        this->order = order;
        this->cutoff_frequency = cutoff_frequency;
        this->sampling_rate = sampling_rate;

        // Initialize filter coefficients
        a_coefficients.resize(order + 1);
        b_coefficients.resize(order + 1);
        x_history.resize(order + 1, 0.0);
        y_history.resize(order + 1, 0.0);

        computeCoefficients();
    }

    double filter(double input) {
        // Shift history
        for (int i = order; i > 0; --i) {
            x_history[i] = x_history[i - 1];
            y_history[i] = y_history[i - 1];
        }

        // Update current input
        x_history[0] = input;

        // Compute output
        double output = 0.0;
        for (int i = 0; i <= order; ++i) {
            output += b_coefficients[i] * x_history[i];
            if (i > 0) {
                output -= a_coefficients[i] * y_history[i];
            }
        }

        // Update current output in history
        y_history[0] = output;

        return output;
    }

private:
    int order;
    double cutoff_frequency;
    double sampling_rate;
    std::vector<double> a_coefficients;
    std::vector<double> b_coefficients;
    std::vector<double> x_history;
    std::vector<double> y_history;

    void computeCoefficients() {
        // Calculate coefficients with Butterworth Polygon
        // Secondary Butterworth Filter
        double wc = tan(M_PI * cutoff_frequency / sampling_rate);
        double k1 = sqrt(2.0) * wc;
        double k2 = wc * wc;
        double norm = 4 + k1 + k2;

        b_coefficients[0] = k2 / norm;
        b_coefficients[1] = 2 * b_coefficients[0];
        b_coefficients[2] = b_coefficients[0];
        a_coefficients[0] = 1;
        a_coefficients[1] = (2 * (k2 - 4)) / norm;
        a_coefficients[2] = (4 - k1 + k2) / norm;
    }
};

#endif // BUTTERWORTH_H
