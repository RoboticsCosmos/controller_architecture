#include "controllers/control_blocks.h"

void summation2(const double *value1, const double *value2, double *result)
{
    *result = *value1 + *value2;
}

void summation3(const double *value1, const double *value2,
                const double *value3, double *result)
{
    *result = *value1 + *value2 + *value3;
}

void subtraction(const double *start_value, const double *reduction_value, double *difference)
{
    *difference = *start_value - *reduction_value;
}

void multiply2(const double *value1, const double *value2, double *result)
{
    *result = *value1 * *value2;
}

void multiply3(const double *value1, const double *value2, const double *value3,
               double *result)
{
    *result = *value1 * *value2 * *value3;
}

void division(const double *dividend, const double *divisor, double *quotient)
{
    *quotient = *dividend / *divisor;
}

void integrator(const double *input, const double *dt, double *output)
{
    *output += *input * *dt;
}

void differentiator(const double *current_value, const double *previous_value, const double *dt, double *output)
{
    *output = (*current_value - *previous_value) / *dt;
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

void integrate_when_in_desired_interval(const double *input,
                                        const double *lower_limit,
                                        const double *upper_limit,
                                        const double *dt, double *output)
{
    if (*input > *lower_limit && *input < *upper_limit)
    {
        integrator(input, dt, output);
    }
    else
    {
        *output = 0;
    }
}

void map_control_command(const double *control_command,
                         const double *control_command_lower_limit,
                         const double *control_command_upper_limit,
                         const double *control_command_mapped_lower_limit,
                         const double *control_command_mapped_upper_limit,
                         double *mapped_control_command)
{
    // normalize
    *mapped_control_command =
        (*control_command - *control_command_lower_limit) /
        (*control_command_upper_limit - *control_command_lower_limit);

    // scale
    *mapped_control_command =
        *mapped_control_command * (*control_command_mapped_upper_limit -
                                   *control_command_mapped_lower_limit);
    // translate
    *mapped_control_command += *control_command_mapped_lower_limit;
}

void get_sign(const double *value, double *sign)
{
    if (*value > 0)
    {
        *sign = 1;
    }
    else if (*value < 0)
    {
        *sign = -1;
    }
    else
    {
        *sign = 0;
    }
}

void hside(const double *value, double *hside_output)
{
    if (*value >= 0)
    {
        *hside_output = 1.0;
    }
    else
    {
        *hside_output = 0.0;
    }
}

void get_abs(const double *value, double *abs_value)
{
    if (*value < 0)
    {
        *abs_value = -(*value);
    }
    else
    {
        *abs_value = *value;
    }
}

void set_value_of_first_to_second_variable(const double *first, double *second)
{
    *second = *first;
}
// void get_p_term(const double *kp,
//                 const double *error,
//                 double *output)
// {
//     multiply(kp, error, output);
// }

// void get_i_term(const double *ki,
//                 const double *error,
//                 double *integral_term,
//                 const double *dt,
//                 double *output)
// {
//     integrator(error, dt, integral_term);
//     multiply(ki, integral_term, output);
// }

// void get_d_term(const double *kd,
//                 const double *error,
//                 const double *prev_error,
//                 const double *dt,
//                 double *output)
// {
//     // *output = *kd * (*error - *prev_error) / *dt;
//     subtraction(error, prev_error, output);
//     division(output, dt, output);
//     multiply(kd, output, output);
// }

// void pid_controller(const double *kp,
//                     const double *ki,
//                     const double *kd,
//                     const double *setpoint,
//                     const double *measured_x,
//                     double *prev_error_x,
//                     double *error,
//                     double *integral_term,
//                     const double *dt,
//                     double *output)
// {
//     // initialization
//     double p_out, i_out, d_out;
//     subtraction(setpoint, measured_x, error);
//     get_p_term(kp, error, &p_out);
//     integrator(error, dt, integral_term);
//     get_i_term(ki, error, integral_term, dt, &i_out);
//     get_d_term(kd, error, prev_error_x, dt, &d_out);

//     *prev_error_x = *error;
//     summation(&p_out, &i_out, output);
//     summation(output, &d_out, output);
// }

// void pd_controller(double *kp,
//                    double *kd,
//                    double *setpoint,
//                    double *measured_x,
//                    double *prev_error_x,
//                    double *error,
//                    double *dt,
//                    double *output)
// {
//     // initialization
//     double p_out, d_out;
//     subtraction(setpoint, measured_x, error);
//     get_p_term(kp, error, &p_out);
//     get_d_term(kd, error, prev_error_x, dt, &d_out);

//     *prev_error_x = *error;
//     summation(&p_out, &d_out, output);
// }

// void impedance_controller(double *kp,
//                           double *kd,
//                           double *pos_error,
//                           double *vel_setpoint,
//                           double *vel_measurement,
//                           double *output)
// {
//     // initialization
//     double p_out, d_out, vel_error;
//     multiply(kp, pos_error, &p_out);
//     subtraction(vel_setpoint, vel_measurement, &vel_error);
//     multiply(kd, &vel_error, &d_out);
//     summation(&p_out, &d_out, output);
// }