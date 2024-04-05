#include <iostream>
#include <cmath>
#include <matplot/matplot.h>


void plant(std::vector<float>& measured_x_array, float& measured_x, float dt, float& control_command)
{
    measured_x += 0.1 * control_command;
    measured_x_array.push_back(measured_x);
}

void pid_controller(float kp, float ki, float kd, float setpoint,
                    float& measured_x, float& prev_error_x,
                    float& error, float& integral_term,
                    float dt, float& output)
{
    // initialization
    float p_out, i_out, d_out;

    // error calculation
    error = setpoint - measured_x;

    // output signal calculation
    p_out = kp * error;
    integral_term += error * dt;
    i_out = ki * integral_term;
    d_out = kd * (error - prev_error_x) / dt;

    // std::cout << "p_out: " << p_out << " i_out: " << i_out << " d_out: " << d_out << std::endl;

    prev_error_x = error;

    output = p_out + i_out + d_out;
}

int main()
{
    using namespace matplot;

    float dt = 0.1f;
    float kp = 0.5f;
    float ki = 0.01f;
    float kd = 0.5f;

    float prev_error_x = 0.0f;

    float measured_x = 0.0f;
    float setpoint_x = 10.0f;

    // create array to store measured_x
    std::vector<float> measured_x_array;
    std::vector<float> time_array;

    float error = 0.0f;
    float integral_term = 0.0f;
    float control_command = 0.0f;

    for (int i = 0; i < 200; ++i)
    {
        time_array.push_back(i * dt);
        plant(measured_x_array, measured_x, dt, control_command);
        pid_controller(kp, ki, kd, setpoint_x, measured_x, prev_error_x, error, integral_term, dt, control_command);
        // std::cout << "measured_x: " << measured_x << " error: " << error << " output: " << control_command << std::endl;
    }

    // scale measured_x_array by 10 with different name
    std::vector<float> measured_x_array_scaled;
    for (auto& x : measured_x_array)
    {
        measured_x_array_scaled.push_back(x * 1.2);
    }

    // Plotting
    hold(on);
    plot(time_array, measured_x_array);
    plot(time_array, measured_x_array_scaled);
    title("Signal Plot");
    xlabel("Time");

    // std::vector<double> theta = iota(pi / 4, pi / 4, 2 * pi);
    // std::vector<double> rho = {19, 6, 12, 18, 16, 11, 15, 15};
    // polarscatter(measured_x_array, time_array);

    show();

    return 0;
}


// #include <stdio.h>
// #include <math.h>

// void plant(float *measured_x,
//            float *dt,
//            float *control_command)
// {
//     *measured_x += 0.1 * *control_command;
// }

// void pid_controller(float *kp, 
//                     float *ki,
//                     float *kd, 
//                     float *setpoint,
//                     float *measured_x, 
//                     float *prev_error_x,
//                     float *error, 
//                     float *integral_term,
//                     float *dt, 
//                     float *output)
// {
//     // initialization
//     float p_out, i_out, d_out;

//     // error calculation
//     *error = *setpoint - *measured_x;

//     // output signal calculation
//     p_out = *kp * *error;
//     *integral_term += *error * *dt;
//     i_out = *ki * *integral_term;
//     d_out = *kd * (*error - *prev_error_x) / *dt;

//     printf("p_out: %7.3f i_out: %7.3f d_out: %7.3f\n", p_out, i_out, d_out);

//     *prev_error_x = *error;

//     *output = p_out + i_out + d_out;
// }

// int main(void)
// {
//     float dt = 0.1;
//     float kp = 0.5;
//     float ki = 0.01;
//     float kd = 0.5;

//     float prev_error_x;

//     float measured_x = 0.0;
//     float setpoint_x = 10.0;

//     float error;
//     float integral_term;
//     float control_command = 0.0;

//     for (int i = 0; i < 200; i++)
//     {
//         plant(&measured_x, &dt, &control_command);
//         pid_controller(&kp, &ki, &kd, &setpoint_x, &measured_x, &prev_error_x, &error, &integral_term, &dt, &control_command);
//         printf("measured_x: %7.3f error: %7.3f output: %7.3f\n", measured_x, error, control_command);
//     }

//     return 0;
// }