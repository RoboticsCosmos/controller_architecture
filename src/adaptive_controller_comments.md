
Adaptive control:
- concrete example: while moving towards the box to pick it, one arm may get stuck or some external force may deviate 
its path. Then keeping the motion specification the same will not work, and assuming it needs to be changed with different
controller architecture, the system should be able to handle it smoothly.

Consideration:
- output type: torque or position or velocity
- input: gains as a matrix
- frame in which the gains are calculated
- inertia matrix in impedance controller is desired inertia matrix and not the actual inertia matrix
- hardware limitation: individual axis, we cnnot have velocity control and torque control at the same time
- low gains: imprecision; high gains: high contact force


Different blocks:
- summing [u1, u2 -> u1+u2]
- integrator [u->integral(u*dt) over time period]
- saturation [u->min(max(u, min), max)]
- gain block (u->f(k, u, error, dt))
- saturation block (u->f(min, max, u))
- sign 
- low pass filter
- adapt bias, adapt gain

Gain scheduling:
- https://uk.mathworks.com/help/slcontrol/ug/implement-gain-scheduled-pid-controllers.html?s_eid=PSM_15028
- https://uk.mathworks.com/help/control/ug/set-up-simulink-models-for-gain-scheduling.html?s_eid=PSM_15028
- https://uk.mathworks.com/help/control/ug/tuning-of-gain-scheduled-three-loop-autopilot.html?s_eid=PSM_15028


// // output signal calculation
// p_out = kp * error;
// integral_term += error * dt;
// i_out = ki * integral_term;
// d_out = kd * (error - prev_error_x) / dt;

// // std::cout << "p_out: " << p_out << " i_out: " << i_out << " d_out: " << d_out << std::endl;

// prev_error_x = error;