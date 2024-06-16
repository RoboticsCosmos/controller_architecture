#include "functions/utilities.h"

void get_median(const double *array, const int *array_size, double *median)
{
  double temp[*array_size];
  for (int i = 0; i < *array_size; i++)
  {
    temp[i] = array[i];
  }
  for (int i = 0; i < *array_size; i++)
  {
    for (int j = i + 1; j < *array_size; j++)
    {
      if (temp[i] > temp[j])
      {
        double a = temp[i];
        temp[i] = temp[j];
        temp[j] = a;
      }
    }
  }
  if (*array_size % 2 == 0)
  {
    *median = (temp[*array_size / 2] + temp[*array_size / 2 - 1]) / 2;
  }
  else
  {
    *median = temp[*array_size / 2];
  }
}

void get_current_time_sec(double *current_time)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *current_time = (double) tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

void update_measured_time_period(const double *previous_time_stamp,
                                 const double *current_time,
                                 double *measured_time_period) {
  *measured_time_period = *current_time - *previous_time_stamp;
}