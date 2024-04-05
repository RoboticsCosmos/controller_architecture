#include <matplot/matplot.h>
#include <vector>
#include <iostream>

// extern "C" 
// {
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

void plant(double *measured_pos,
               double *measured_vel,
               const double *dt,
               const double *control_command,
               bool *within_uncertainty_range)
{
    *measured_pos += 0.1 * *control_command;
    if (*within_uncertainty_range)
    {
        *measured_vel += 0.1 * *control_command;
    }
}

void less_than_monitor(const double *value_to_compare,
                       const double *target_value,
                       bool *detection_flag)
{
    if (*value_to_compare < *target_value)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void greater_than_monitor(const double *value_to_compare,
                          const double *target_value,
                          bool *detection_flag)
{
    if (*value_to_compare > *target_value)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void in_interval_monitor(const double *value_to_compare,
                      const double *target_value,
                      const double *epsilon,
                      bool *detection_flag)
{
    if (abs(*value_to_compare - *target_value) < *epsilon)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void summation(const double *value1, const double *value2, double *result)
{
    *result = *value1 + *value2;
}

void subtraction(const double *start_value, const double *reduction_value, double *difference)
{
    *difference = *start_value - *reduction_value;
}

void multiplication(const double *value1, const double *value2, double *result)
{
    *result = *value1 * *value2;
}

void division(const double *dividend, const double *divisor, double *quotient)
{
    *quotient = *dividend / *divisor;
}

void integrator(const double *input, const double *dt, double *output)
{
    *output += *input * *dt;
}

void differentiator(const double *input, const double *prev_input, const double *dt, double *output)
{
    *output = (*input - *prev_input) / *dt;
}

void saturation(double *input, const double *lower_limit, const double *upper_limit)
{
    double output;
    if (*input < *lower_limit)
    {
        *input = *lower_limit;
    }
    else if (*input > *upper_limit)
    {
        *input = *upper_limit;
    }

}

void get_p_term(const double *kp,
                const double *error,
                double *output)
{
    multiplication(kp, error, output);
}

void get_i_term(const double *ki,
                const double *error,
                double *integral_term,
                const double *dt,
                double *output)
{
    integrator(error, dt, integral_term);
    multiplication(ki, integral_term, output);
}

void get_d_term(const double *kd,
                const double *error,
                const double *prev_error,
                const double *dt,
                double *output)
{
    // *output = *kd * (*error - *prev_error) / *dt;
    subtraction(error, prev_error, output);
    division(output, dt, output);
    multiplication(kd, output, output);
}

void pid_controller(const double *kp,
                    const double *ki,
                    const double *kd,
                    const double *setpoint,
                    const double *measured_x,
                    double *prev_error_x,
                    double *error,
                    double *integral_term,
                    const double *dt,
                    double *output)
{
    // initialization
    double p_out, i_out, d_out;
    subtraction(setpoint, measured_x, error);
    get_p_term(kp, error, &p_out);
    integrator(error, dt, integral_term);
    get_i_term(ki, error, integral_term, dt, &i_out);
    get_d_term(kd, error, prev_error_x, dt, &d_out);

    *prev_error_x = *error;
    summation(&p_out, &i_out, output);
    summation(output, &d_out, output);
}

void pd_controller(double *kp,
                   double *kd,
                   double *setpoint,
                   double *measured_x,
                   double *prev_error_x,
                   double *error,
                   double *dt,
                   double *output)
{
    // initialization
    double p_out, d_out;
    subtraction(setpoint, measured_x, error);
    get_p_term(kp, error, &p_out);
    get_d_term(kd, error, prev_error_x, dt, &d_out);

    *prev_error_x = *error;
    summation(&p_out, &d_out, output);
}

void impedance_controller(double *kp,
                          double *kd,
                          double *pos_error,
                          double *vel_setpoint,
                          double *vel_measurement,
                          double *output)
{
    // initialization
    double p_out, d_out, vel_error;
    multiplication(kp, pos_error, &p_out);
    subtraction(vel_setpoint, vel_measurement, &vel_error);
    multiplication(kd, &vel_error, &d_out);
    summation(&p_out, &d_out, output);
}
// }

int main(void)
{
    using namespace matplot;

    double dt = 0.1;
    double epsilon = 0.1;

    double kp = 0.5;
    double ki = 0.01;
    double kd = 0.1;

    double stiffness_param = 0.05;
    double damping_param = 0.05;

    double prev_error;
    double integral_saturation_ul = .01;
    double integral_saturation_ll = -.3;

    double measured_pos = 20.0;
    double measured_vel = 2.0;

    double setpoint_pos = 4.0;
    // double uncertainty_start_pos = 4.0;
    double setpoint_vel = 0.0;
    double setpoint_force = 0.0;
    double setpoint_accel = 0.0;

    double pos_error;
    double integral_term = 0.0;
    double control_command = 0.0;

    bool detection_flag = false;
    bool within_uncertainty_range = false;
    bool slide_along_surface = false;
    bool move_downwards = true; // as enums or intgers

    // for plotting
    std::vector<float> measured_pos_array;
    std::vector<float> measured_vel_array;
    std::vector<float> kp_array;
    std::vector<float> kd_array;
    std::vector<float> time_array;

    // state machine
    for (int i = 0; i < 200; i++)
    {
        plant(&measured_pos, &measured_vel, &dt, &control_command, &within_uncertainty_range);
        if (move_downwards)
        {
            pid_controller(&kp, &ki, &kd, &setpoint_pos, &measured_pos, &prev_error, &pos_error, &integral_term, &dt, &control_command);
            less_than_monitor(&measured_pos, &setpoint_pos, &detection_flag);
            if (detection_flag)
            {
                within_uncertainty_range = true;
                move_downwards = false;
                setpoint_pos = 2.0;
                std::cout << "reached uncertainty range at time: " << i * dt << "\n";
            }            
        }
        else if (within_uncertainty_range)
        {
            impedance_controller(&stiffness_param, &damping_param, &pos_error, &setpoint_vel, &measured_vel, &control_command);
            in_interval_monitor(&measured_vel, &setpoint_vel, &epsilon, &detection_flag);
            if (detection_flag)
            {
                slide_along_surface = true;
                within_uncertainty_range = false;
                kp = 0.05;
                kd = 0.05;
                std::cout << "contact detected at time: " << i * dt << "\n";
            }
        }
        else if (slide_along_surface)
        {
            pd_controller(&kp, &kd, &setpoint_pos, &measured_pos, 
            &prev_error, &pos_error, &dt, &control_command);
        }

        measured_pos_array.push_back(measured_pos);
        measured_vel_array.push_back(measured_vel);
        kp_array.push_back(kp);
        kd_array.push_back(kd);
        time_array.push_back(i * dt);
    }

    // Plotting
    figure();
    // hold(on);
    plot(time_array, measured_pos_array);
    title("Position Plot");
    xlabel("Time (s)");
    ylabel("Position (m)");
    grid(on);
    auto ax1 = gca();

    figure();
    // hold(on);
    plot(time_array, measured_vel_array);
    title("Velocity Plot");
    xlabel("Time (s)");
    ylabel("Velocity (m/s)");
    grid(on);
    auto ax2 = gca();    

    figure();
    hold(on);
    plot(time_array, kp_array);
    plot(time_array, kd_array);
    xlabel("Time (s)");
    ylabel("Gains Kp, Kd");
    title("Gain Plot");
    legend({"Kp", "Kd"});
    grid(on);
    auto ax3 = gca();

    show();

    return 0;
}