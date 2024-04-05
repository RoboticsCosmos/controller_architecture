from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class PID_CONTROLLER(DefinedNamespace):
            
    PIDController: URIRef
    PDController: URIRef
    PIController: URIRef
    PController: URIRef
    ConstantGain: URIRef
    TrajectoryController: URIRef
    RegulationController: URIRef
    ConditionalIntegrationWhenInControllableRange: URIRef
    CleggIntegratorToResetWindup: URIRef
    PreventOvershoot: URIRef
    DeadbandActive: URIRef
    BumplessOperation: URIRef

    output_signal: URIRef
    p_error_term: URIRef
    i_error_term: URIRef
    d_error_term: URIRef
    deadband_value_absolute: URIRef
    integral_windup_limit_absolute: URIRef
    controllable_region_error_absolute: URIRef
    reset_windup_at_zero_crossing: URIRef
    prevent_overshoot: URIRef
    bumpless_operation: URIRef
    p_gain: URIRef
    p_gain_upper_bound: URIRef
    p_gain_lower_bound: URIRef
    i_gain: URIRef
    i_gain_upper_bound: URIRef
    i_gain_lower_bound: URIRef
    d_gain: URIRef
    d_gain_upper_bound: URIRef
    d_gain_lower_bound: URIRef
    init_p_gain: URIRef
    init_i_gain: URIRef
    init_d_gain: URIRef
    time_instance: URIRef
    time_period: URIRef

    _NS = Namespace(
        "https://controller.org/metamodels/controllers/PID/pid_controller#"
    )

class IMPEDANCE_CONTROLLER(DefinedNamespace):

    ImpedanceController: URIRef

    output_torque_signal: URIRef
    pos_error: URIRef
    vel_error: URIRef
    acceleration_error: URIRef
    stiffness_param: URIRef
    stiffness_param_upper_bound: URIRef
    stiffness_param_lower_bound: URIRef
    inertia_param: URIRef
    inertia_param_upper_bound: URIRef
    inertia_param_lower_bound: URIRef
    damping_param: URIRef
    damping_param_upper_bound: URIRef
    damping_param_lower_bound: URIRef
    init_stiffness_param: URIRef
    init_inertia_param: URIRef
    init_damping_param: URIRef
    
    _NS = Namespace(
        "https://controller.org/metamodels/controllers/ForceControl/impedance_controller#"
    )

class FUNCTIONS(DefinedNamespace):

    Functions: URIRef
    LowPassFilteredErrorSignal: URIRef
    GetSign: URIRef
    Saturation: URIRef
    HeavisideStepFunction: URIRef
    Modulus: URIRef
    MapOutputToActuationSignal: URIRef

    low_pass_filter_factor_alpha: URIRef
    low_pass_filter_factor_alpha_limits: URIRef
    low_pass_filter_factor_alpha_limits_inclusive: URIRef
    saturation_bool: URIRef
    saturation_limits: URIRef
    saturation_limits_inclusive: URIRef
    map_output_to_actuation_scaling_limits: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/functions#")

class ERROR_SIGNAL(DefinedNamespace):

    ErrorSignal: URIRef
    InstantaneousErrorSignalPosition: URIRef
    InstantaneousErrorSignalVelocity: URIRef
    PredictFromErrorSignal: URIRef
    PredictFromProcessVariableSignal: URIRef
    HistoryFromStartErrorSignal: URIRef
    HistoryFromRecentTimeWindowErrorSignal: URIRef

    setpoint: URIRef
    measured_value: URIRef
    error: URIRef
    prev_error: URIRef
    history_time_window_length: URIRef
    time_period: URIRef

    _NS = Namespace(
        "https://controller.org/metamodels/controllers/error_signal#"
    )

class CONTROLLER(DefinedNamespace):

    Controller: URIRef
    Plant: URIRef

    setpoint_pos: URIRef
    setpoint_vel: URIRef
    measured_variable_pos: URIRef
    measured_variable_vel: URIRef
    actuation_signal: URIRef
    time_instance: URIRef
    time_period: URIRef
    plant_name: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/controller#")

class PLAN(DefinedNamespace):

    Plan: URIRef
    State: URIRef
    Transition: URIRef
    
    states_set: URIRef
    controller_to_activate: URIRef
    transitions: URIRef
    transition_state: URIRef
    start_state: URIRef
    events: URIRef
    
    _NS = Namespace("https://controller.org/metamodels/architecture_components/plan#")

class MONITOR(DefinedNamespace):

    Monitor: URIRef
    Event: URIRef
    LessThanMonitor: URIRef
    GreaterThanMonitor: URIRef
    InIntervalMonitor: URIRef
    EqualMonitor: URIRef

    time_instance_start_log: URIRef
    time_instance_end_log: URIRef
    in_interval_lower_bound: URIRef
    in_interval_upper_bound: URIRef
    in_interval_epsilon: URIRef
    value_to_monitor: URIRef
    reference_value: URIRef
    desired_bool_value: URIRef
    flags: URIRef

    _NS = Namespace("https://controller.org/metamodels/architecture_components/monitor#")    