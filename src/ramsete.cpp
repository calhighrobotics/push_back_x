#include "lemlib/api.hpp" // IWYU pragma: keep
#include <array>
#include <cmath>

class ramsete
{
    private:
    std::array<double, 3> desired_values, actual_values;
    double v_d, w_d, damping;
    float b, wheel_diameter = 3.25; //Change wheel diameter according to robot drivetrain

    public:

    ramsete(std::array<double, 3> &desired, std::array<double, 3> &actual, double desired_linear_velocity, double desired_angular_velocity, double damp, float proportional)
    {   
        desired_values = desired;
        actual_values = actual;
        v_d = desired_linear_velocity;
        w_d = desired_angular_velocity;
        damping = damp;
        b = proportional;
    }

    std::array<double, 2> ramsete_controller()
    {
        std::array<double, 2> output_velocities = {0, 0};

        std::array<double , 3> error_computation = {0, 0, 0};

        std::array<std::array<double, 3>, 3> transformation_matrix = {{
            { std::cos(actual_values.at(2)),  std::sin(actual_values.at(2)), 0 },
            {-std::sin(actual_values.at(2)),  std::cos(actual_values.at(2)), 0 },
            {0, 0, 1}
        }};

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                error_computation[i] += transformation_matrix[i][j] * (desired_values.at(i) - actual_values.at(i));
            }
        }
        
        double gain = 2 * damping * std::sqrt((w_d*w_d) + b * (v_d*v_d));

        double linear_velocity = v_d * std::cos(error_computation.at(2)) + gain * error_computation.at(0);
        double angular_velocity = w_d + gain * error_computation.at(2) + (b * v_d * std::sin(error_computation.at(2)) * error_computation.at(1))/(error_computation.at(2)+0.000000001);

        double left = linear_velocity/(2*std::numbers::pi*(wheel_diameter/2)) + angular_velocity;
        double right = linear_velocity/(2*std::numbers::pi*(wheel_diameter/2)) - angular_velocity;

        return {left, right};
    }
};