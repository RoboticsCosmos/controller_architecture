#include "functions/monitors.h"
#include "functions/utilities.h"

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

void less_than_equal_to_monitor(const double *value_to_compare,
                       const double *target_value,
                       bool *detection_flag)
{
    if (*value_to_compare <= *target_value)
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

void greater_than_equal_to_monitor(const double *value_to_compare,
                          const double *target_value,
                          bool *detection_flag)
{
    if (*value_to_compare >= *target_value)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void in_interval_monitor(const double *value_to_compare,
                         const double *in_interval_upper_bound,
                         const double *in_interval_lower_bound,
                         bool *detection_flag)
{
    if (*value_to_compare < *in_interval_upper_bound && *value_to_compare > *in_interval_lower_bound)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void equality_monitor(const double *value_to_compare,
                      const double *target_value,
                      const double *epsilon,
                      bool *detection_flag)
{
    if (fabs(*value_to_compare - *target_value) < *epsilon)
    {
        *detection_flag = true;
    }
    else
    {
        *detection_flag = false;
    }
}

void event_consistent_for_n_sec(const double *time_in_seconds,
                                const bool *event, double *event_start_time,
                                bool *event_started_bool,
                                double *current_time_sec_temp,
                                bool *event_consistent_for_n_sec_flag)
{
    get_current_time_sec(current_time_sec_temp);

    // handling null pointers
    if (!time_in_seconds || !event || !event_start_time || !event_started_bool || !current_time_sec_temp || !event_consistent_for_n_sec_flag)
    {
        return;
    }

    if (!*event)
    {
        *event_started_bool = false;
        *event_consistent_for_n_sec_flag = false;
    }
    else
    {
        if (!*event_started_bool)
        {
            *event_start_time = *current_time_sec_temp;
            *event_started_bool = true;
            *event_consistent_for_n_sec_flag = false;
        }
        else
        {
            if (*current_time_sec_temp - *event_start_time >= *time_in_seconds)
            {
                *event_consistent_for_n_sec_flag = true;
            }
            else
            {
                *event_consistent_for_n_sec_flag = false;
            }
        }
    }
}