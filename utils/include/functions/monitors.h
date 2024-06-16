
#ifndef MONITORS_H
#define MONITORS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

    /**
     * @file: monitors.h
     *
     * This file contains the function declarations for the monitors.
     */

    /**
     * @brief monitor if value_to_compare is less than target_value
     *
     * @param value_to_compare: value to compare
     * @param target_value: target value to compare against
     * @param detection_flag: flag to indicate if the condition is met
     */
    void less_than_monitor(const double *value_to_compare,
                           const double *target_value,
                           bool *detection_flag);

    /**
     * @brief monitor if value_to_compare is less than target_value
     *
     * @param value_to_compare: value to compare
     * @param target_value: target value to compare against
     * @param detection_flag: flag to indicate if the condition is met
     */
    void less_than_equal_to_monitor(const double *value_to_compare,
                           const double *target_value,
                           bool *detection_flag);

    /**
     * @brief monitor if value_to_compare is greater than target_value
     *
     * @param value_to_compare: value to compare
     * @param target_value: target value to compare against
     * @param detection_flag: flag to indicate if the condition is met
     */
    void greater_than_monitor(const double *value_to_compare,
                              const double *target_value,
                              bool *detection_flag);

    /**
     * @brief monitor if value_to_compare is greater than target_value
     *
     * @param value_to_compare: value to compare
     * @param target_value: target value to compare against
     * @param detection_flag: flag to indicate if the condition is met
     */
    void greater_than_equal_to_monitor(const double *value_to_compare,
                              const double *target_value,
                              bool *detection_flag);

    /**
     * @brief monitor if value is within desired interval
     *
     * @param value: value to check
     * @param in_interval_upper_bound: upper bound of interval
     * @param in_interval_lower_bound: lower bound of interval
     * @param detection_flag: flag to indicate if the condition is met
     */
    void in_interval_monitor(const double *value,
                             const double *in_interval_upper_bound,
                             const double *in_interval_lower_bound,
                             bool *detection_flag);

    /**
     * @brief monitor if value_to_compare is equal to target_value within epsilon
     *
     * @param value_to_compare: value to compare
     * @param target_value: target value to compare against
     * @param epsilon: tolerance
     * @param detection_flag: flag to indicate if the condition is met
     */
    void equality_monitor(const double *value_to_compare,
                          const double *target_value,
                          const double *epsilon,
                          bool *detection_flag);

    /**
     * @brief check if the event is consistent for n seconds
     *
     * @param time_in_seconds: time in seconds
     * @param event: event
     * @param event_start_time: event start time
     * @param event_started_bool: event started flag
     * @param event_consistent_for_n_sec_flag: flag to indicate if the event is
  consistent for n seconds
    */

    /**
     * @brief check if the event is consistent for n seconds
     *
     * @param time_in_seconds: duration the event must be consistent for
     * @param event: the current state of the event (true/false)
     * @param event_start_time: event start time
     * @param event_started_bool: event started flag
     * @param current_time_sec_temp: current time in seconds
     * @param event_consistent_for_n_sec_flag: flag to indicate if the event is consistently true for 'time_in_seconds'
     */
    void event_consistent_for_n_sec(const double *time_in_seconds,
                                    const bool *event,
                                    double *event_start_time,
                                    bool *event_started_bool,
                                    double *current_time_sec_temp,
                                    bool *event_consistent_for_n_sec_flag);

#ifdef __cplusplus
}
#endif

#endif // MONITORS_H