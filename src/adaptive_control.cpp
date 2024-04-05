#include <iostream>
#include <cmath>
#include <matplot/matplot.h>
#include <vector>
#include <deque>

enum ControllerType
{
    P,
    PD,
    PID,
    IMPEDANCE
};

enum MonitorType
{
    INEQUALITY_CONDITION,
    PERSISTANT_INEQUALITY_CONDITION,
    NONE
};

enum InequalityType
{
    GREATER_THAN,
    LESS_THAN,
    EQUAL_TO
};

void summation(const std::vector<float> &u1,
               const std::vector<float> &u2,
               std::vector<float> &u_sum)
{
    // Check if u1 and u2 have the same size
    if (u1.size() != u2.size())
    {
        throw std::invalid_argument("Vectors u1 and u2 must have the same size for element-wise addition.");
    }

    u_sum.resize(u1.size());

    for (size_t i = 0; i < u1.size(); ++i)
    {
        u_sum[i] = u1[i] + u2[i];
    }
}

void summation(const std::vector<float> &u1,
               const std::vector<float> &u2,
               std::vector<float> &u3,
               std::vector<float> &u_sum)
{
    // Check if u1, u2 and u3 have the same size
    if (u1.size() != u2.size() || u1.size() != u3.size())
    {
        throw std::invalid_argument("Vectors u1, u2 and u3 must have the same size for element-wise addition.");
    }

    u_sum.resize(u1.size());

    for (size_t i = 0; i < u1.size(); ++i)
    {
        u_sum[i] = u1[i] + u2[i] + u3[i];
    }
}

void multiply(const std::vector<float> &u1,
              const std::vector<float> &u2,
              std::vector<float> &u_mul)
{
    // Check if u1 and u2 have the same size
    if (u1.size() != u2.size())
    {
        throw std::invalid_argument("Vectors u1 and u2 must have the same size for element-wise multiplication.");
    }

    u_mul.resize(u1.size());

    // Perform element-wise multiplication
    for (size_t i = 0; i < u1.size(); ++i)
    {
        u_mul[i] = u1[i] * u2[i];
    }
}

void multiply(const std::vector<float> &u1,
              const float &u2,
              std::vector<float> &u_mul)
{
    u_mul.resize(u1.size());

    // Perform element-wise multiplication
    for (size_t i = 0; i < u1.size(); ++i)
    {
        u_mul[i] = u1[i] * u2;
    }
}

void difference(const std::vector<float> &desired_vector,
                const std::vector<float> &measurement_vector,
                std::vector<float> &error_vector)
{
    // Check if setpoints and measured_values have the same size
    if (desired_vector.size() != measurement_vector.size())
    {
        throw std::invalid_argument("Vectors setpoints and measured_values must have the same size for error calculation.");
    }

    error_vector.resize(desired_vector.size());

    for (size_t i = 0; i < desired_vector.size(); ++i)
    {
        error_vector[i] = desired_vector[i] - measurement_vector[i];
    }
}

void integrator(const std::vector<float> &pos_error_vector,
                const float &dt,
                std::vector<float> &integral_term_vector)
{
    // Check if error and integral_term have the same size
    if (pos_error_vector.size() != integral_term_vector.size())
    {
        throw std::invalid_argument("Vectors error and integral_term must have the same size for integration.");
    }

    integral_term_vector.resize(pos_error_vector.size());

    for (size_t i = 0; i < pos_error_vector.size(); ++i)
    {
        integral_term_vector[i] += pos_error_vector[i] * dt;
    }
}

void saturation(std::vector<float> &u_sat,
                const std::vector<float> &min,
                const std::vector<float> &max)
{
    for (size_t i = 0; i < u_sat.size(); ++i)
    {
        u_sat[i] = std::min(std::max(u_sat[i], min[i]), max[i]);
    }
}

void plant(const ControllerType &controller_type,
           std::vector<float> &pos_measurement_vector,
           std::vector<float> &vel_measurement_vector,
           const float &dt,
           const std::vector<float> &control_command_vector)
{
    for (size_t i = 0; i < pos_measurement_vector.size(); ++i)
    {
        pos_measurement_vector[i] += 0.1 * control_command_vector[i];
    }

    if (controller_type == IMPEDANCE)
    {
        for (size_t i = 0; i < vel_measurement_vector.size(); ++i)
        {
            vel_measurement_vector[i] += 0.1 * control_command_vector[i];
        }
    }
    // pos_measurement_vector[0] += 0.1 * (rand() % 100) / 100;
}

void data_buffer(std::deque<std::vector<float>> &vector_to_compare_history,
                 const std::vector<float> *vector_to_compare,
                 const std::vector<int> &dim_to_compare,
                 const int &history_length)
{
    std::vector<float> vector_to_compare_at_dim;
    for (size_t j = 0; j < dim_to_compare.size(); ++j)
    {
        vector_to_compare_at_dim.push_back((*vector_to_compare)[dim_to_compare[j]]);
    }
    vector_to_compare_history.push_back(vector_to_compare_at_dim);
    if (vector_to_compare_history.size() > history_length)
    {
        vector_to_compare_history.pop_front();
    }
}

void monitor(const MonitorType &monitor_type,
             const int &time_step_index,
             const float &dt,
             bool &detection_flag,
             const int &history_length,
             std::deque<std::vector<float>> &vector_to_compare_history,
             const std::vector<float> *vector_to_compare,
             const std::vector<float> *desired_setpoint_vector,
             const std::vector<int> &dim_to_compare,
             const std::vector<InequalityType> &inequality_type,
             const float epsilon = 0.1f)
{
    // eg: if both arms are moving towards the box, and one arm gets stuck or deviated, the direction and magnitude of deviation
    // is unknown, and controller should be able to handle it accordingly.

    /* different types of monitors:
    (i) based on instantaneous value
    (ii) based on temporal history of values
    (iii) based on anticipation of future values
    (iv) monitors can also trigger some motions or sensors to get more information about the environment until hypothesis is confirmed
    */
    // if size of desired_setpoint_vector is 1 then set a flag to true

    detection_flag = false;

    switch (monitor_type)
    {

    case INEQUALITY_CONDITION:
        // based on the inequality type, the condition is checked at each dimension to compare the measured value with the desired value
        for (size_t j = 0; j < dim_to_compare.size(); ++j)
        {
            switch (inequality_type[j])
            {
            case GREATER_THAN:
                if ((*vector_to_compare)[dim_to_compare[j]] > (*desired_setpoint_vector)[dim_to_compare[j]])
                    detection_flag = true;
                break;

            case LESS_THAN:
                if ((*vector_to_compare)[dim_to_compare[j]] < (*desired_setpoint_vector)[dim_to_compare[j]])
                    detection_flag = true;
                break;

            case EQUAL_TO:
                if (abs((*vector_to_compare)[dim_to_compare[j]] - (*desired_setpoint_vector)[dim_to_compare[j]]) < epsilon)
                    detection_flag = true;
                break;
            }
            if (!detection_flag)
            {
                break;
            }
        }
        if (detection_flag)
        {
            std::cout << "monitor triggered at time step: " << time_step_index << "\n"
                      << std::endl;
        }
        break;

    case PERSISTANT_INEQUALITY_CONDITION:
        data_buffer(vector_to_compare_history,
                    vector_to_compare,
                    dim_to_compare,
                    history_length);
        if (vector_to_compare_history.size() < history_length)
        {
            detection_flag = false;
        }
        else
        {
            for (size_t j = 0; j < vector_to_compare_history.size(); ++j)
            {
                for (size_t k = 0; k < dim_to_compare.size(); ++k)
                {
                    detection_flag = false;

                    switch (inequality_type[k])
                    {
                    case GREATER_THAN:
                        if (vector_to_compare_history[j][k] > (*desired_setpoint_vector)[dim_to_compare[k]])
                            detection_flag = true;
                        break;

                    case LESS_THAN:
                        if (vector_to_compare_history[j][k] < (*desired_setpoint_vector)[dim_to_compare[k]])
                            detection_flag = true;
                        break;

                    case EQUAL_TO:
                        if (abs(vector_to_compare_history[j][k] - (*desired_setpoint_vector)[dim_to_compare[k]]) < epsilon)
                            detection_flag = true;
                        break;
                    }
                    if (!detection_flag)
                    {
                        break;
                    }
                }
                if (!detection_flag)
                {
                    break;
                }
            }
        }
        if (detection_flag)
        {
            std::cout << "monitor triggered at time step: " << time_step_index << "\n"
                      << std::endl;
        }
        break;

    case NONE:
        detection_flag = false;
        break;

    default:
        std::cerr << "Invalid MonitorType" << std::endl;
        break;
    }
}

void adaptor(ControllerType &controller_type,
             MonitorType &monitor_type,
             std::vector<InequalityType> &inequality_type,
             const bool &detection_flag,
             std::vector<float> &kp,
             std::vector<float> &ki,
             std::vector<float> &kd,
             std::vector<float> &integral_saturation_ll,
             std::vector<float> &integral_saturation_ul,
             std::vector<float> *desired_setpoint_vector,
             std::vector<float> *vector_to_compare,
             const std::vector<float> &pos_setpoint_vector,
             const std::vector<float> &pos_measurement_vector,
             const std::vector<float> &vel_setpoint_vector,
             const std::vector<float> &vel_measurement_vector,
             const int &time_step_index,
             const float &dt)
{
    // switch gains; switch monitor function; transient free switch between controllers (maybe ramp down over certain time period);
    // switch adaptors; switch controllers (select from a library?);
    // also use time to adapt or schedule
    // not considered: when it needs to monitor both or mix up controllers in different directions

    if (detection_flag)
    {
        if (monitor_type == INEQUALITY_CONDITION)
        {
            controller_type = IMPEDANCE;
            kp = {0.1f, 0.1f, 0.1f};
            kd = {0.01f, 0.01f, 0.01f};
            *desired_setpoint_vector = vel_setpoint_vector;
            *vector_to_compare = vel_measurement_vector;
            // monitor_type = NONE;
            monitor_type = PERSISTANT_INEQUALITY_CONDITION;
            inequality_type = {EQUAL_TO, EQUAL_TO, EQUAL_TO};
            std::cout << "Switching to Impedance Controller\n"
                    << std::endl;
        }
        else if (monitor_type == PERSISTANT_INEQUALITY_CONDITION)
        {
            // switch to a different controller
            controller_type = PD;
            // switch to a different setpoint
            // switch to a different gains
            kd = {0.01f, 0.01f, 0.01f};
            kp = {0.01f, 0.01f, 0.01f};
            *desired_setpoint_vector = pos_setpoint_vector;
            *vector_to_compare = pos_measurement_vector;
            std::cout << "Switching to PD Controller\n"
                    << std::endl;
            monitor_type = NONE;
            // switch to a different adaptor
            // switch to a different monitor function (detect contact)
        }
    }
}

void estimator()
{
    // estimation of weight of the box; estimation of friction; estimation of inertia; estimation of velocity; estimation of acceleration
}

void transition_ramp()
{
    // transition between controllers; transition between adaptors; transition between estimators
    // transition between setpoints; transition between gains; transition between monitor functions
}

void controller(const ControllerType type,
                const std::vector<float> &kp,
                const std::vector<float> &ki,
                const std::vector<float> &kd,
                const std::vector<float> &integral_saturation_ll,
                const std::vector<float> &integral_saturation_ul,
                const std::vector<float> &pos_setpoint_vector,
                const std::vector<float> &vel_setpoint_vector,
                const std::vector<float> &pos_measurement_vector,
                const std::vector<float> &vel_measurement_vector,
                std::vector<float> &prev_pos_error_vector,
                std::vector<float> &pos_error_vector,
                std::vector<float> &vel_error_vector,
                std::vector<float> &integral_term_vector,
                const float &dt,
                std::vector<float> &output_vector)
{
    std::vector<float> p_out, i_out, d_out;
    std::vector<float> pos_error_diff;

    difference(pos_setpoint_vector, pos_measurement_vector, pos_error_vector);

    switch (type)
    {
    case P:
        multiply(kp, pos_error_vector, p_out);
        output_vector = p_out;
        break;

    case PD:
        difference(pos_error_vector, prev_pos_error_vector, pos_error_diff);
        multiply(kp, pos_error_vector, p_out);
        multiply(kd, pos_error_diff, d_out);
        multiply(d_out, 1.0 / dt, d_out);
        summation(p_out, d_out, output_vector);
        break;

    case PID:
        difference(pos_error_vector, prev_pos_error_vector, pos_error_diff);
        multiply(kp, pos_error_vector, p_out);
        integrator(pos_error_vector, dt, integral_term_vector);
        saturation(integral_term_vector, integral_saturation_ll, integral_saturation_ul);
        multiply(ki, integral_term_vector, i_out);
        multiply(kd, pos_error_diff, d_out);
        multiply(d_out, 1.0 / dt, d_out);
        summation(p_out, i_out, d_out, output_vector);
        break;

    case IMPEDANCE:
        multiply(kp, pos_error_vector, p_out);
        difference(vel_setpoint_vector, vel_measurement_vector, vel_error_vector);
        multiply(kd, vel_error_vector, d_out);
        summation(p_out, d_out, output_vector);
        break;

    default:
        std::cerr << "Invalid ControllerType" << std::endl;
        break;
    }
    prev_pos_error_vector = pos_error_vector;
}

int main()
{
    using namespace matplot;

    float dt = 0.1f;

    std::vector<float> kp = {0.5f, 0.5f, 0.5f};
    std::vector<float> ki = {0.01f, 0.01f, 0.01f};
    std::vector<float> kd = {0.1f, 0.1f, 0.1f};

    std::vector<float> integral_saturation_ul = {.01f, .01f, .01f};
    std::vector<float> integral_saturation_ll = {-3.f, -3.f, -3.f};

    std::vector<float> prev_pos_error_vector = {0.0f, 0.0f, 0.0f};

    // Measurements
    std::vector<float> pos_measurement_vector = {20.0f, 0.0f, 5.0f};
    std::vector<float> vel_measurement_vector = {2.0f, 0.0f, 2.0f};

    // Setpoints
    std::vector<float> pos_setpoint_vector = {10.0f, 5.0f, -5.0f};
    std::vector<float> vel_setpoint_vector = {0.0f, 0.0f, 0.0f};
    std::vector<float> force_setpoint_vector = {0.0f, 0.0f, 0.1f};
    std::vector<float> acceleration_setpoint_vector = {0.0f, 0.0f, 0.1f};

    std::vector<float> pos_error_vector = {0.0f, 0.0f, 0.0f};
    std::vector<float> vel_error_vector = {0.0f, 0.0f, 0.0f};
    std::vector<float> integral_term_vector = {0.0f, 0.0f, 0.0f};
    std::vector<float> control_command_vector = {0.0f, 0.0f, 0.0f};

    // define flags based on proximity to setpoint
    std::vector<float> error_threshold = {1.f, 1.f, 5.f};
    bool detection_flag = false;
    int history_length = 10;
    std::deque<std::vector<float>> vector_to_compare_history;
    std::vector<float> *vector_to_compare;
    std::vector<float> *desired_setpoint_vector;
    std::vector<int> dim_to_compare = {0, 1, 2};

    // create array to store measured values for plotting
    std::vector<float> measured_x_array;
    std::vector<float> measured_y_array;
    std::vector<float> measured_z_array;
    std::vector<float> kp_x_array;
    std::vector<float> time_array;

    ControllerType controller_type = PID;
    MonitorType monitor_type = INEQUALITY_CONDITION;
    std::vector<InequalityType> inequality_type = {EQUAL_TO, EQUAL_TO, EQUAL_TO};
    int iterations = 200;
    float epsilon = 0.1f;

    desired_setpoint_vector = &pos_setpoint_vector;
    vector_to_compare = &pos_measurement_vector;

    for (int i = 0; i < iterations; ++i)
    {
        monitor(monitor_type,
                i,
                dt,
                detection_flag,
                history_length,
                vector_to_compare_history,
                vector_to_compare,
                desired_setpoint_vector,
                dim_to_compare,
                inequality_type,
                epsilon);

        adaptor(controller_type,
                monitor_type,
                inequality_type,
                detection_flag,
                kp,
                ki,
                kd,
                integral_saturation_ll,
                integral_saturation_ul,
                desired_setpoint_vector,
                vector_to_compare,
                pos_setpoint_vector,
                pos_measurement_vector,
                vel_setpoint_vector,
                vel_measurement_vector,
                i,
                dt);

        plant(controller_type,
              pos_measurement_vector,
              vel_measurement_vector,
              dt,
              control_command_vector);

        measured_x_array.push_back(pos_measurement_vector[0]);
        measured_y_array.push_back(pos_measurement_vector[1]);
        measured_z_array.push_back(pos_measurement_vector[2]);
        kp_x_array.push_back(kp[0]);
        time_array.push_back(i * dt);

        controller(controller_type,
                   kp,
                   ki,
                   kd,
                   integral_saturation_ll,
                   integral_saturation_ul,
                   pos_setpoint_vector,
                   vel_setpoint_vector,
                   pos_measurement_vector,
                   vel_measurement_vector,
                   prev_pos_error_vector,
                   pos_error_vector,
                   vel_error_vector,
                   integral_term_vector,
                   dt,
                   control_command_vector);
        // std::cout << "measured_x: " << measured_x << " error: " << error << " output: " << control_command << std::endl;
    }

    // Plotting
    figure();
    hold(on);
    plot(time_array, measured_x_array);
    plot(time_array, measured_y_array);
    plot(time_array, measured_z_array);
    title("Signal Plot");
    xlabel("Time (s)");
    ylabel("Value (m)");
    grid(on);
    legend({"position (x)", "position (y)", "position (z)"});
    auto ax1 = gca();

    figure();
    plot(time_array, kp_x_array);
    xlabel("Time (s)");
    ylabel("Kp (x)");
    title("P Gain Plot");
    grid(on);
    auto ax2 = gca();

    show();

    return 0;
}
/*
Concerns:
1. input quantity to the plant and the output
2. composition of controllers
3. transition between controllers
4. modification of gains
5. variation of monitors (contact, threshold, etc), adapters, estimators
6. variation of setpoints
7. variation of saturation limits
8. switch b/w control modes (VELOCITY, TORQUE)
9. frames in which the gains are calculated
10.individual axis, different controllers
*/
