
#include <stdio.h>
#include <math.h>

void plant(float *measured_x,
           float *dt,
           float *control_command)
{
    *measured_x += 0.1 * *control_command;
}

void pid_controller(float *kp, 
                    float *ki,
                    float *kd, 
                    float *setpoint,
                    float *measured_x, 
                    float *prev_error_x,
                    float *error, 
                    float *integral_term,
                    float *dt, 
                    float *output)
{
    // initialization
    float p_out, i_out, d_out;

    // error calculation
    *error = *setpoint - *measured_x;

    // output signal calculation
    p_out = *kp * *error;
    *integral_term += *error * *dt;
    i_out = *ki * *integral_term;
    d_out = *kd * (*error - *prev_error_x) / *dt;

    printf("p_out: %7.3f i_out: %7.3f d_out: %7.3f\n", p_out, i_out, d_out);

    *prev_error_x = *error;

    *output = p_out + i_out + d_out;
}

int main(void)
{
    float dt = 0.1;
    float kp = 0.5;
    float ki = 0.01;
    float kd = 0.5;

    float prev_error_x;

    float measured_x = 0.0;
    float setpoint_x = 10.0;

    float error;
    float integral_term;
    float control_command = 0.0;

    for (int i = 0; i < 200; i++)
    {
        plant(&measured_x, &dt, &control_command);
        pid_controller(&kp, &ki, &kd, &setpoint_x, &measured_x, &prev_error_x, &error, &integral_term, &dt, &control_command);
        printf("measured_x: %7.3f error: %7.3f output: %7.3f\n", measured_x, error, control_command);
    }

    return 0;
}