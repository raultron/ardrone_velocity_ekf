#include "ardrone_velocity_ekf/filtervelocity.hpp"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <cstdio>

FilterVelocity::FilterVelocity()
{
    filter_size = 30.0;

    filter_coeffs << 0.00498902,  0.00567655,  0.00768429,  0.01092849,  0.01526534,  0.02049766,
                        0.02638413,  0.03265082,  0.0390043,   0.04514569,  0.05078516,  0.05565588,
                        0.05952694,  0.06221459,  0.06359114,  0.06359114,  0.06221459,  0.05952694,
                        0.05565588,  0.05078516,  0.04514569,  0.0390043,   0.03265082,  0.02638413,
                        0.02049766,  0.01526534,  0.01092849,  0.00768429,  0.00567655,  0.00498902;

    median_data = Eigen::Vector3d::Zero(3);

}

double FilterVelocity::filter(double new_value){
    double result = 0.0;
    int i;
    filter_input_buffer(0) = new_value;

    for(i=0; i < filter_size; i++){
        result += filter_input_buffer(i)*filter_coeffs[i];
    }

    for(i=filter_size-1; i > 0; i--){
        filter_input_buffer(i) = filter_input_buffer(i-1);
    }
    return result;
}



void FilterVelocity::median_filter(double derivative, double &new_derivative)
{
    double max_d = median_data(0);
    double min_d = median_data(1);
    double previous_derivative = median_data(2);

    // 3 Step Median Filter
    if (derivative > max_d)
    {
        new_derivative = max_d;
    }
    else if (derivative < min_d)
    {
        new_derivative = min_d;
    }
    else
    {
        new_derivative=derivative;
    }
    if (derivative > previous_derivative)
    {
        // for next cycle
        max_d=derivative;
        min_d=previous_derivative;
    }
    else
    {
        max_d=previous_derivative;
        min_d=derivative;
    }
    previous_derivative = derivative;

    median_data(0) = max_d;
    median_data(1) = min_d;
    median_data(2) =  previous_derivative;
}

void FilterVelocity::smith_filter(double cutoff_frequency, double delta_t, double error, double &new_derivative)
{
    error_at(2) = error_at(1);
    error_at(1) = error_at(0);
    error_at(0) = error;
    double c = 1;
    //Filter was designed based on Smith III, Intro. to Digital Filters With Audio Applications. Filter from the ROS PID package.
    if (cutoff_frequency != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      double tan_filt = tan( (cutoff_frequency*6.2832)*delta_t/2 );

      // Avoid tan(0) ==> NaN
      if ( (tan_filt<=0.) && (tan_filt>-0.01) )
        tan_filt = -0.01;
      if ( (tan_filt>=0.) && (tan_filt<0.01) )
        tan_filt = 0.01;

      c = 1/tan_filt;
    }

    filtered_error_at(2) = filtered_error_at(1);
    filtered_error_at(1) = filtered_error_at(0);
    filtered_error_at(0) = (1/(1+c*c+1.414*c))*(error_at(2)+2*error_at(1)+error_at(0)-(c*c-1.414*c+1)*filtered_error_at(2)-(-2*c*c+2)*filtered_error_at(1));

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_at(2) = error_deriv_at(1);
    error_deriv_at(1) = error_deriv_at(0);
    error_deriv_at(0) = (error_at(0)-error_at(1))/delta_t;

    filtered_error_deriv_at(2) = filtered_error_deriv_at(1);
    filtered_error_deriv_at(1) = filtered_error_deriv_at(0);

    if ( loop_counter>2 ) // Let some data accumulate
    {
      filtered_error_deriv_at(0) = (1/(1+c*c+1.414*c))*(error_deriv_at(2)+2*error_deriv_at(1)+error_deriv_at(0)-(c*c-1.414*c+1)*filtered_error_deriv_at(2)-(-2*c*c+2)*filtered_error_deriv_at(1));
      new_derivative = filtered_error_deriv_at(0);
    }
    else
    {
      loop_counter++;
    }


}
