#ifndef FILTERVELOCITY_HPP
#define FILTERVELOCITY_HPP
#include <vector>
#include <cstdint>
#include <eigen3/Eigen/Dense>

class FilterVelocity
{
public:
    FilterVelocity();
    double filter(double new_value);
    void median_filter(double derivative, double &new_derivative);
    void smith_filter(double cutoff_frequency, double delta_t, double error, double &new_derivative);

private:
    double filter_size;
    double loop_counter = 0;
    Eigen::Matrix<double, 30,1> filter_coeffs;
    Eigen::Matrix<double, 30,1> filter_input_buffer;
    Eigen::Vector3d median_data;
    Eigen::Vector3d error_at;
    Eigen::Vector3d error_deriv_at;
    Eigen::Vector3d filtered_error_at;
    Eigen::Vector3d filtered_error_deriv_at;
};

#endif // FILTERVELOCITY_HPP
