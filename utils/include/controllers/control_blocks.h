#ifndef CONTROL_BLOCKS_H
#define CONTROL_BLOCKS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h> // eg: for qsort
    /**
     * @file: control_blocks.h
     *
     * This file contains the function declarations for the control blocks.
     */

    /**
     * @brief sum two values
     *
     * @param value1: first value
     * @param value2: second value
     * @param result: sum of the two values
     */
    void summation2(const double *value1, const double *value2, double *result);

    /**
     * @brief sum three values
     *
     * @param value1: first value
     * @param value2: second value
     * @param value3: third value
     * @param result: sum of the three values
     */
    void summation3(const double *value1, const double *value2, const double *value3, double *result);

    /**
     * @brief subtract one value from another
     *
     * @param start_value: value to subtract from
     * @param reduction_value: value to subtract
     * @param difference: difference between the two values
     */
    void subtraction(const double *start_value, const double *reduction_value, double *difference);

    /**
     * @brief multiply two values together
     *
     * @param value1: value to scale
     * @param value2: factor to scale by
     * @param result: scaled value
     */
    void multiply2(const double *value1, const double *value2, double *result);

    /**
     * @brief multiply three values together
     *
     * @param value1: value to scale
     * @param value2: factor to scale by
     * @param value3: factor to scale by
     * @param result: scaled value
     */
    void multiply3(const double *value1, const double *value2,
                   const double *value3, double *result);

    /**
     * @brief divide one value by another
     *
     * @param dividend: value to divide (numerator)
     * @param divisor: value to divide by (denominator)
     * @param quotient: result of the division
     */
    void division(const double *dividend, const double *divisor, double *quotient);

    /**
     * @brief integrate a value
     *
     * @param input: error value to integrate
     * @param dt: time step
     * @param output: integrated value
     */
    void integrator(const double *input, const double *dt, double *output);

    /**
     * @brief differentiate a value
     *
     * @param current_value: value to differentiate
     * @param previous_value: previous value
     * @param dt: time step
     * @param output: differentiated value
     */
    void differentiator(const double *current_value, const double *previous_value, const double *dt, double *output);

    /**
     * @brief saturate a value
     *
     * @param input: value to saturate
     * @param lower_limit: lower limit
     * @param upper_limit: upper limit
     */
    void saturation(double *input, const double *lower_limit, const double *upper_limit);

    /**
     * @brief integrate a value when it is in a desired interval (Clegg integrator)
     *
     * @param input: value to integrate
     * @param lower_limit: lower limit of the interval
     * @param upper_limit: upper limit of the interval
     * @param dt: time step
     * @param output: integrated value
     */
    void integrate_when_in_desired_interval(const double *input,
                                            const double *lower_limit,
                                            const double *upper_limit,
                                            const double *dt,
                                            double *output);

    /**
     * @brief map a control command to a new range
     *
     * @param control_command: control command to map
     * @param control_command_lower_limit: lower limit of the control command
     * @param control_command_upper_limit: upper limit of the control command
     * @param control_command_mapped_lower_limit: lower limit of the mapped control command
     * @param control_command_mapped_upper_limit: upper limit of the mapped control command
     * @param mapped_control_command: mapped control command
     */
    void map_control_command(const double *control_command,
                             const double *control_command_lower_limit,
                             const double *control_command_upper_limit,
                             const double *control_command_mapped_lower_limit,
                             const double *control_command_mapped_upper_limit,
                             double *mapped_control_command);

    /**
     * @brief get the sign of a value
     *
     * @param value: value to get the sign of
     * @param sign: sign of the value
     */
    void get_sign(const double *value, double *sign);

    /**
     * @brief get the hside function output of a value
     *
     * @param value: value to get the hside of
     * @param hside_output: hside of the value
     */
    void hside(const double *value, double *hside_output);

    /**
     * @brief get the absolute value of a value
     *
     * @param value: value to get the absolute value of
     * @param abs_value: absolute value of the value
     */
    void get_abs(const double *value, double *abs_value);

    /**
     * @brief set the value of the first variable as the value of the second variable
     *
     * @param first: first variable which is constant
     * @param second: second variable which is updated
     */
    void set_value_of_first_to_second_variable(const double *first, double *second);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_BLOCKS_H