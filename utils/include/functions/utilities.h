
#ifndef UTILITIES_H
#define UTILITIES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

  /**
   * @file: utilities.h
   *
   * This file contains the function declarations for the utilities.
   */

  // /**
  //  * @brief circular buffer structure
  //  *
  //  * @param buffer: buffer to store data
  //  * @param size: size of the buffer
  //  * @param head: head of the buffer
  //  * @param tail: tail of the buffer
  //  * @param is_full: flag to indicate if the buffer is full
  //  */
  // typedef struct
  // {
  //   char *buffer;
  //   size_t size;
  //   size_t head;
  //   size_t tail;
  //   int is_full;
  // } CircularBuffer;

  /**
   * @brief get the median of an array
   *
   * @param array: array of values
   * @param array_size: size of the array
   * @param median: median of the array
   */
  void get_median(const double *array, const int *array_size, double *median);

  /**
   * @brief get the current time in sec with micro seconds precision
   *
   * @param current_time: current time
   */
  void get_current_time_sec(double *current_time);

  /**
   * @brief get the measured time period
   *
   * @param previous_time_stamp_sec: previous time stamp
   * @param current_time_sec: current time
   * @param measured_time_period_sec: measured time period
   */
  void update_measured_time_period(const double *previous_time_stamp,
                                   const double *current_time, double *measured_time_period);

  // void append_to_buffer()

#ifdef __cplusplus
}
#endif

#endif // UTILITIES_H