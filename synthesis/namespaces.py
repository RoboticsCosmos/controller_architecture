from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class ALGORITHM(DefinedNamespace):

    Algorithm: URIRef
    Data: URIRef
    UncertainQuantity: URIRef
    Schedule: URIRef
    ScheduleCall: URIRef
    EventBasedScheduleCall: URIRef
    FunctionCall: URIRef
    ConstraintController: URIRef
    Activity: URIRef
    algorithm_details: URIRef
    schedules_of_algorithm: URIRef
    schedule_event_associations: URIRef
    data_type: URIRef
    data_name: URIRef
    uncertainty_data: URIRef
    init_value: URIRef
    trigger_chain: URIRef
    associated_controller: URIRef
    constraints_controlled: URIRef
    constraint_controller: URIRef
    schedule_to_call: URIRef
    desired_events: URIRef

    _NS = Namespace("https://controller.org/metamodels/algorithm#")

class ERROR(DefinedNamespace):

    ErrorSignal: URIRef
    ErrorFunction: URIRef
    InstantaneousError: URIRef
    DerivativeFromError: URIRef
    DerivativeFromProcessVariable: URIRef
    IntegralFromStartError: URIRef
    IntegralFromRecentTimeWindowError: URIRef
    ErrorBasedOnConstraint: URIRef
    measured_quantity: URIRef
    reference_quantity: URIRef
    integral_time_window_length: URIRef
    time_period: URIRef
    quantity_to_integrate: URIRef
    differentiated_data: URIRef
    integrated_data: URIRef
    error_data: URIRef
    current_value: URIRef
    previous_value: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/error#")

class FUNCTIONS(DefinedNamespace):

    Functions: URIRef
    LowPassFilteredErrorSignal: URIRef
    GetSign: URIRef
    Saturation: URIRef
    HeavisideStepFunction: URIRef
    Absolute: URIRef
    CopyVariableValue: URIRef
    MapOutputToActuationSignal: URIRef
    Sum: URIRef
    Subtract: URIRef
    Multiply: URIRef
    Scale: URIRef
    Divide: URIRef
    GetCurrentTime: URIRef
    variable_to_copy_to: URIRef
    variable_to_copy_from: URIRef
    filtered_data: URIRef
    sign_data: URIRef
    saturated_data: URIRef
    stepped_data: URIRef
    absolute_data: URIRef
    summed_data: URIRef
    difference_data: URIRef
    product_data: URIRef
    divided_data: URIRef
    mapped_data: URIRef
    input_signal: URIRef
    signal_to_map: URIRef
    signal_to_process: URIRef
    signal_to_saturate: URIRef
    arguments_list: URIRef
    low_pass_filter_factor_alpha: URIRef
    low_pass_filter_factor_alpha_limits: URIRef
    low_pass_filter_factor_alpha_limits_inclusive: URIRef
    saturation_limits: URIRef
    actuation_scaling_limits: URIRef
    current_time_data: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/functions#")

class CONTROLLER(DefinedNamespace):

    Controller: URIRef
    Plant: URIRef
    Signal: URIRef
    ControlCommand: URIRef
    Setpoint: URIRef
    Feedback: URIRef
    TimePeriod: URIRef
    ctr_functions: URIRef
    ctr_data_structures: URIRef
    controller_name: URIRef
    controller_output: URIRef
    desired_time_period_val: URIRef
    measured_time_period_val: URIRef
    measured_pos: URIRef
    measured_vel: URIRef
    control_mode: URIRef
    servoing_mode: URIRef
    actuation_signal: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/controller#")

class ABAG(DefinedNamespace):

    ABAG: URIRef
    adaptive_bias: URIRef
    adaptive_gain: URIRef
    output_signal: URIRef
    bias_adaptation_threshold: URIRef
    bias_adaptation_step_delta: URIRef
    adaptive_bias_limits: URIRef
    gain_adaptation_threshold: URIRef
    gain_adaptation_step_delta: URIRef
    adaptive_gain_limits: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/ABAG/abag#")

class PID_CONTROLLER(DefinedNamespace):

    PIDController: URIRef
    DController: URIRef
    IController: URIRef
    PController: URIRef
    ConstantGain: URIRef
    TrajectoryController: URIRef
    RegulatingController: URIRef
    LimitIntegralTerm: URIRef
    ConditionalIntegrationWhenInControllableRange: URIRef
    CleggIntegratorToResetWindup: URIRef
    DeadbandActive: URIRef
    BumplessOperation: URIRef
    GainSignal: URIRef
    p_error_term: URIRef
    i_error_term: URIRef
    d_error_term: URIRef
    deadband_range_absolute: URIRef
    integral_windup_limit_absolute: URIRef
    controllable_region_error_absolute: URIRef
    reset_windup_at_zero_crossing: URIRef
    bumpless_operation: URIRef
    gain_calculator: URIRef
    p_gain_function: URIRef
    i_gain_function: URIRef
    d_gain_function: URIRef
    init_p_gain: URIRef
    init_i_gain: URIRef
    init_d_gain: URIRef
    p_gain_signal: URIRef
    i_gain_signal: URIRef
    d_gain_signal: URIRef
    gain_value: URIRef
    p_error_signal_calculator: URIRef
    i_error_signal_calculator: URIRef
    d_error_signal_calculator: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/PID/pid_controller#")

class IMPEDANCE_CONTROLLER(DefinedNamespace):

    ImpedanceController: URIRef
    ConstantImpedanceParameters: URIRef
    pos_error: URIRef
    vel_error: URIRef
    acceleration_error: URIRef
    stiffness_param: URIRef
    inertia_param: URIRef
    damping_param: URIRef
    init_stiffness_param: URIRef
    init_inertia_param: URIRef
    init_damping_param: URIRef

    _NS = Namespace("https://controller.org/metamodels/controllers/ForceControl/impedance_controller#")

class MONITOR(DefinedNamespace):

    Monitor: URIRef
    Constraint: URIRef
    LessThanConstraint: URIRef
    LessThanEqualToConstraint: URIRef
    GreaterThanConstraint: URIRef
    GreaterThanEqualToConstraint: URIRef
    InIntervalConstraint: URIRef
    OutOfIntervalConstraint: URIRef
    EqualConstraint: URIRef
    ConstraintMonitor: URIRef
    GreaterThanUpperLimitConstraint: URIRef
    GreaterThanLowerLimitConstraint: URIRef
    LowerThanLowerLimitConstraint: URIRef
    LowerThanUpperLimitConstraint: URIRef
    ConstraintToEvent: URIRef
    ConstraintToFlag: URIRef
    EventMonitor: URIRef
    in_interval_lower_bound: URIRef
    in_interval_upper_bound: URIRef
    tolerance: URIRef
    constraint_to_monitor: URIRef
    quantity_to_compare: URIRef
    quantity_to_compare_with: URIRef
    flag_or_event_to_check: URIRef
    flag_set_by_monitor: URIRef
    event_emitted_by_monitor: URIRef
    operator: URIRef

    _NS = Namespace("https://controller.org/metamodels/architecture_components/monitor#")

class PLAN(DefinedNamespace):

    Plan: URIRef
    State: URIRef
    Transition: URIRef
    Timeout: URIRef
    EmergencyStop: URIRef
    StateParameters: URIRef
    Action: URIRef
    MotionSpecification: URIRef
    pre_condition: URIRef
    post_condition: URIRef
    per_condition: URIRef
    motion_specification: URIRef
    monitors: URIRef
    action: URIRef
    set_of_states: URIRef
    state_parameters: URIRef
    start_state: URIRef
    transitions: URIRef
    termination: URIRef
    triggering_events: URIRef
    state_to_transit_to: URIRef
    timeout_period: URIRef
    emergency_stop_flag_init: URIRef
    setpoint_pos: URIRef
    setpoint_vel: URIRef
    init_setpoint_value: URIRef
    setpoint_function: URIRef
    setpoint_signal: URIRef
    controller_output_composition: URIRef
    arm_name: URIRef
    direction_of_specification: URIRef

    _NS = Namespace("https://controller.org/metamodels/architecture_components/plan#")