// script starts from here ... (not a comment for template, but for c/cpp)

#include <Eigen/Core>
#include <chrono>
#include <controllers/control_blocks.h>
#include <functions/monitors.h>
#include <iostream>
#include <kinova_mediator/mediator.hpp>
#include <math.h>
#include <vector>
#include <yaml-cpp/yaml.h>

// KDL libraries
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

// handling signals: referred from https://github.com/RoboticsCosmos/motion_spec_gen/blob/0b48adc779ae6d6d593c1d7bcdb87e5535482fc3/gen/freddy_uc1_ref_log.cpp
#include <csignal>

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
    flag = 1;
    std::cout << "Received signal: " << sig << std::endl;
}
// extern "C"
// {
namespace k_api = Kinova::Api;

void kinova_feedback(kinova_mediator &kinova_arm_mediator,
                     KDL::JntArray &jnt_positions,
                     KDL::JntArray &jnt_velocities,
                     KDL::JntArray &jnt_torques)
{
    kinova_arm_mediator.get_joint_state(jnt_positions,
                                        jnt_velocities,
                                        jnt_torques);
}

void get_end_effector_pose_and_twist(KDL::JntArrayVel &jnt_velocity,
                                     const KDL::JntArray &jnt_positions,
                                     const KDL::JntArray &jnt_velocities,
                                     KDL::Frame &measured_endEffPose_BL,
                                     KDL::FrameVel &measured_endEffTwist_BL,
                                     KDL::Frame &measured_endEffPose_GF,
                                     KDL::FrameVel &measured_endEffTwist_GF,
                                     std::shared_ptr<KDL::ChainFkSolverPos_recursive> &fkSolverPos,
                                     std::shared_ptr<KDL::ChainFkSolverVel_recursive> &fkSolverVel,
                                     const KDL::Frame &BL_wrt_GF_frame)
{
    // getting the end effector pose and twist
    jnt_velocity.q = jnt_positions;
    jnt_velocity.qdot = jnt_velocities;

    fkSolverPos->JntToCart(jnt_positions, measured_endEffPose_BL);
    fkSolverVel->JntToCart(jnt_velocity, measured_endEffTwist_BL);

    measured_endEffPose_GF = BL_wrt_GF_frame * measured_endEffPose_BL;
    measured_endEffTwist_GF = BL_wrt_GF_frame * measured_endEffTwist_BL;
}

void calculate_joint_torques_RNEA(
    std::shared_ptr<KDL::ChainJntToJacDotSolver> &jacobDotSolver,
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> &ikSolverAcc,
    std::shared_ptr<KDL::ChainIdSolver_RNE> &idSolver,
    KDL::JntArrayVel &jnt_velocity,
    KDL::Twist &jd_qd,
    KDL::Twist &xdd,
    KDL::Twist &xdd_minus_jd_qd,
    KDL::JntArray &jnt_accelerations,
    KDL::JntArray &jnt_positions,
    KDL::JntArray &jnt_velocities,
    KDL::Wrenches &linkWrenches_EE,
    KDL::JntArray &jnt_torques)
{
    // get joint torques
    jacobDotSolver->JntToJacDot(jnt_velocity, jd_qd);
    xdd_minus_jd_qd = xdd - jd_qd;
    ikSolverAcc->CartToJnt(jnt_positions, xdd_minus_jd_qd, jnt_accelerations);
    idSolver->CartToJnt(jnt_positions, jnt_velocities, jnt_accelerations, linkWrenches_EE, jnt_torques);
}

enum robot_controlled
{
    DUAL_KINOVA_GEN3_ARMS = 0,
    KINOVA_GEN3_1_LEFT = 1,
    KINOVA_GEN3_2_RIGHT = 2,
};

// Function to append data to the file with dynamic size of vector
template <size_t N>
void appendDataToFile_dynamic_size(std::ofstream &file, const std::vector<std::array<double, N>> &data)
{
    for (const auto &row : data)
    {
        for (size_t i = 0; i < N; ++i)
        {
            file << row[i];
            if (i < N - 1)
            {
                file << ",";
            }
        }
        file << "\n";
    }
}

int main()
{

    // set robots to control
    robot_controlled robots_to_control = robot_controlled::KINOVA_GEN3_1_LEFT;

    // handling signals
    struct sigaction sa;
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    for (int i = 1; i < NSIG; ++i)
    {
        if (sigaction(i, &sa, NULL) == -1)
        {
            perror("sigaction");
        }
    }

    // initialise data by reading from the config file
    double TIMEOUT_DURATION_TASK; // in seconds
    double WRENCH_THRESHOLD_LINEAR;
    double WRENCH_THRESHOLD_ROTATIONAL;
    double JOINT_TORQUE_THRESHOLD;
    int SAVE_LOG_EVERY_NTH_STEP;
    double DESIRED_TIME_STEP;

    YAML::Node config_file = YAML::LoadFile("parameters.yaml");
    std::vector<float> arm1_home = config_file["arm_1"]["home"].as<std::vector<float>>();
    std::vector<float> arm2_home = config_file["arm_2"]["home"].as<std::vector<float>>();
    TIMEOUT_DURATION_TASK = config_file["TIMEOUT_DURATION_TASK"].as<double>(); // seconds
    WRENCH_THRESHOLD_LINEAR = config_file["WRENCH_THRESHOLD_LINEAR"].as<double>();
    WRENCH_THRESHOLD_ROTATIONAL = config_file["WRENCH_THRESHOLD_ROTATIONAL"].as<double>();
    JOINT_TORQUE_THRESHOLD = config_file["JOINT_TORQUE_THRESHOLD"].as<double>();
    SAVE_LOG_EVERY_NTH_STEP = config_file["SAVE_LOG_EVERY_NTH_STEP"].as<int>();
    DESIRED_TIME_STEP = config_file["DESIRED_TIME_STEP"].as<double>();

    // urdf and KDL
    KDL::Tree kinematic_tree;
    KDL::Chain chain_urdf;

    kdl_parser::treeFromFile("urdf/Kinova_1.urdf", kinematic_tree);
    kinematic_tree.getChain("base_link", "EndEffector_Link", chain_urdf);
    const unsigned int NUM_LINKS = chain_urdf.getNrOfSegments();

    /* KDL solvers */
    std::shared_ptr<KDL::ChainJntToJacDotSolver> jacobDotSolver;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkSolverPos;
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> fkSolverVel;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ikSolverAcc;
    std::shared_ptr<KDL::ChainIdSolver_RNE> idSolver_left;
    std::shared_ptr<KDL::ChainIdSolver_RNE> idSolver_right;

    const KDL::Vector GRAVITY_left(0., 0.0, -9.81);  // [in BL] the negative of this gravity vector is considered as input acceleration for the force controller
    const KDL::Vector GRAVITY_right(0., 0.0, -9.81); // [in BL] the negative of this gravity vector is considered as input acceleration for the force controller

    jacobDotSolver = std::make_shared<KDL::ChainJntToJacDotSolver>(chain_urdf);
    fkSolverPos = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_urdf);
    fkSolverVel = std::make_shared<KDL::ChainFkSolverVel_recursive>(chain_urdf);
    ikSolverAcc = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_urdf);
    idSolver_left = std::make_shared<KDL::ChainIdSolver_RNE>(chain_urdf, GRAVITY_left);
    idSolver_right = std::make_shared<KDL::ChainIdSolver_RNE>(chain_urdf, GRAVITY_right);

    // transformations
    const KDL::Vector BL_x_axis_wrt_GF_left(1.0, 0.0, 0.0);
    const KDL::Vector BL_y_axis_wrt_GF_left(0.0, 1.0, 0.0);
    const KDL::Vector BL_z_axis_wrt_GF_left(0.0, 0.0, 1.0);
    const KDL::Vector BL_position_wrt_GF_left(0., 0.5, 0.0); // left
    const KDL::Vector BL_x_axis_wrt_GF_right(1.0, 0.0, 0.0);
    const KDL::Vector BL_y_axis_wrt_GF_right(0.0, 1.0, 0.0);
    const KDL::Vector BL_z_axis_wrt_GF_right(0.0, 0.0, 1.0);
    const KDL::Vector BL_position_wrt_GF_right(0.0, -0.5, 0.0);                                                  // right
    const KDL::Rotation BL_wrt_GF_left(BL_x_axis_wrt_GF_left, BL_y_axis_wrt_GF_left, BL_z_axis_wrt_GF_left);     // added as columns
    const KDL::Rotation BL_wrt_GF_right(BL_x_axis_wrt_GF_right, BL_y_axis_wrt_GF_right, BL_z_axis_wrt_GF_right); // added as columns
    const KDL::Frame BL_wrt_GF_frame_left(BL_wrt_GF_left, BL_position_wrt_GF_left);
    const KDL::Frame BL_wrt_GF_frame_right(BL_wrt_GF_right, BL_position_wrt_GF_right);

    // end effector Pose
    KDL::Frame measured_endEffPose_BL_left_arm;
    KDL::Frame measured_endEffPose_GF_left_arm;
    KDL::FrameVel measured_endEffTwist_BL_left_arm;
    KDL::FrameVel measured_endEffTwist_GF_left_arm;
    KDL::Frame measured_endEffPose_BL_right_arm;
    KDL::Frame measured_endEffPose_GF_right_arm;
    KDL::FrameVel measured_endEffTwist_BL_right_arm;
    KDL::FrameVel measured_endEffTwist_GF_right_arm;

    // Joint variables
    KDL::JntArray jnt_positions_left(kinova_constants::NUMBER_OF_JOINTS);
    KDL::JntArray jnt_velocities_left(kinova_constants::NUMBER_OF_JOINTS);   // has only joint velocities of all joints
    KDL::JntArray jnt_torques_left_read(kinova_constants::NUMBER_OF_JOINTS); // to read from the robot
    KDL::JntArray jnt_positions_right(kinova_constants::NUMBER_OF_JOINTS);
    KDL::JntArray jnt_velocities_right(kinova_constants::NUMBER_OF_JOINTS);
    KDL::JntArray jnt_torques_right_read(kinova_constants::NUMBER_OF_JOINTS); // to read from the robot

    KDL::JntArray jnt_torques_left_cmd(kinova_constants::NUMBER_OF_JOINTS);  // to send to the robot
    KDL::JntArray jnt_torques_right_cmd(kinova_constants::NUMBER_OF_JOINTS); // to send to the robot
    KDL::JntArrayVel jnt_velocity_left(kinova_constants::NUMBER_OF_JOINTS);  // has both joint position and joint velocity of all joints
    KDL::JntArrayVel jnt_velocity_right(kinova_constants::NUMBER_OF_JOINTS); // has both joint position and joint velocity of all joints

    KDL::JntArray jnt_accelerations_left(kinova_constants::NUMBER_OF_JOINTS);
    KDL::JntArray jnt_accelerations_right(kinova_constants::NUMBER_OF_JOINTS);

    KDL::JntArray zero_jnt_velocities(kinova_constants::NUMBER_OF_JOINTS);
    zero_jnt_velocities.data.setZero();

    // Link wrenches
    KDL::Wrenches linkWrenches_GF_left(NUM_LINKS, KDL::Wrench::Zero());
    KDL::Wrenches linkWrenches_EE_left(NUM_LINKS, KDL::Wrench::Zero());
    KDL::Wrenches linkWrenches_GF_right(NUM_LINKS, KDL::Wrench::Zero());
    KDL::Wrenches linkWrenches_EE_right(NUM_LINKS, KDL::Wrench::Zero());

    // cartesian acceleration
    KDL::Twist xdd_left;
    KDL::Twist xdd_minus_jd_qd_left;
    KDL::Twist jd_qd_left;
    KDL::Twist xdd_right;
    KDL::Twist xdd_minus_jd_qd_right;
    KDL::Twist jd_qd_right;

    // joint torques that will be calculated before setting the control mode
    std::vector<double> rne_output_jnt_torques_vector_to_set_control_mode_left(kinova_constants::NUMBER_OF_JOINTS);
    std::vector<double> rne_output_jnt_torques_vector_to_set_control_mode_right(kinova_constants::NUMBER_OF_JOINTS);

    kinova_mediator kinova_left;  // 192.168.1.10 (KINOVA_GEN3_1)
    kinova_mediator kinova_right; // 192.168.1.12 (KINOVA_GEN3_2)

    // robot communication
    if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_1_LEFT)
    {
        kinova_left.kinova_id = KINOVA_GEN3_1;
        kinova_left.initialize(kinova_environment::REAL, robot_id::KINOVA_GEN3_1,
                               0.0);

        kinova_feedback(kinova_left, jnt_positions_left, jnt_velocities_left,
                        jnt_torques_left_read);

        get_end_effector_pose_and_twist(
            jnt_velocity_left, jnt_positions_left, jnt_velocities_left,
            measured_endEffPose_BL_left_arm, measured_endEffTwist_BL_left_arm,
            measured_endEffPose_GF_left_arm, measured_endEffTwist_GF_left_arm,
            fkSolverPos, fkSolverVel, BL_wrt_GF_frame_left);

        calculate_joint_torques_RNEA(jacobDotSolver, ikSolverAcc, idSolver_left,
                                     jnt_velocity_left, jd_qd_left, xdd_left,
                                     xdd_minus_jd_qd_left, jnt_accelerations_left,
                                     jnt_positions_left, jnt_velocities_left,
                                     linkWrenches_EE_left, jnt_torques_left_cmd);

        // convert JntArray to double array
        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        {
            rne_output_jnt_torques_vector_to_set_control_mode_left[i] =
                jnt_torques_left_cmd(i);
            std::cout << "jnt_torques_left_cmd: [" << i << "]  " << rne_output_jnt_torques_vector_to_set_control_mode_left[i] << std::endl;
        }
        kinova_left.set_control_mode(control_mode::TORQUE, rne_output_jnt_torques_vector_to_set_control_mode_left.data());
    }

    if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_2_RIGHT)
    {
        kinova_right.kinova_id = KINOVA_GEN3_2;
        kinova_right.initialize(kinova_environment::REAL, robot_id::KINOVA_GEN3_2,
                                0.0);

        kinova_feedback(kinova_right, jnt_positions_right, jnt_velocities_right,
                        jnt_torques_right_read);

        get_end_effector_pose_and_twist(
            jnt_velocity_right, jnt_positions_right, jnt_velocities_right,
            measured_endEffPose_BL_right_arm, measured_endEffTwist_BL_right_arm,
            measured_endEffPose_GF_right_arm, measured_endEffTwist_GF_right_arm,
            fkSolverPos, fkSolverVel, BL_wrt_GF_frame_right);

        calculate_joint_torques_RNEA(jacobDotSolver, ikSolverAcc, idSolver_right,
                                     jnt_velocity_right, jd_qd_right, xdd_right,
                                     xdd_minus_jd_qd_right, jnt_accelerations_right,
                                     jnt_positions_right, jnt_velocities_right,
                                     linkWrenches_EE_right, jnt_torques_right_cmd);

        // convert JntArray to double array
        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        {
            rne_output_jnt_torques_vector_to_set_control_mode_right[i] =
                jnt_torques_right_cmd(i);
            std::cout << "jnt_torques_right_cmd: [" << i << "]  " << rne_output_jnt_torques_vector_to_set_control_mode_right[i] << std::endl;
        }
        kinova_right.set_control_mode(control_mode::TORQUE, rne_output_jnt_torques_vector_to_set_control_mode_right.data());
    }

    // initialise all data structures (done)
    double i_term_lin_vel_z_axis_right_arm_data = 0.0;
    bool pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_flag = false;
    double damping_lin_z_axis_right_arm_data = -300.0;
    double d_term_lin_vel_error_z_axis_right_arm_data = 0.0;
    double lin_vel_error_pid_z_axis_right_arm_data = 0.0;
    double lin_pos_sp_equality_tolerance_x_axis_left_arm_data = 0.02;
    double d_gain_lin_vel_z_axis_right_arm_data = -2.5;
    double lin_pos_error_stiffness_z_axis_left_arm_data = 0.0;
    bool pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_flag = false;
    double integral_of_vel_term_error_z_axis_right_arm_data = 0.0;
    double lin_vel_sp_z_axis_left_arm_data = -0.025;
    double apply_ee_force_x_axis_right_arm_data = 0.0;
    double damping_term_z_axis_left_arm_data = 0.0;
    bool pos_constraint_arm_above_pos_sp_z_axis_left_arm_flag = false;
    double p_gain_pos_y_axis_left_arm_data = -50.0;
    bool pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag = false;
    double lin_zero_vel_z_axis_left_arm_data = 0.0;
    double stiffness_lin_z_axis_left_arm_data = -100.0;
    double p_term_lin_vel_z_axis_right_arm_data = 0.0;
    double stiffness_term_lin_z_axis_left_arm_data = 0.0;
    bool pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_event = false;
    double apply_ee_force_y_axis_right_arm_data = 0.0;
    double p_term_lin_vel_z_axis_left_arm_data = 0.0;
    double i_term_saturation_upper_limit_z_axis_left_arm_data = 5.0;
    double lin_pos_sp_equality_tolerance_z_axis_right_arm_data = 0.005;
    bool pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_event = false;
    double i_term_saturation_upper_limit_z_axis_right_arm_data = 5.0;
    double i_term_saturation_lower_limit_z_axis_right_arm_data = -5.0;
    double measured_lin_vel_x_axis_left_arm_data = 0.0;
    double lin_vel_sp_threshold_to_activate_i_z_axis_right_arm_data = 0.005;
    double previous_lin_vel_error_pid_z_axis_right_arm_data = 0.0;
    double damping_lin_z_axis_left_arm_data = -300.0;
    bool pos_out_of_tolerance_about_setpoint_y_axis_left_arm_event = false;
    double apply_ee_force_z_axis_right_arm_data = 0.0;
    double time_period_of_complete_controller_cycle_data = 0.0;
    double lin_vel_error_pid_z_axis_left_arm_data = 0.0;
    double apply_ee_force_x_axis_left_arm_data = 0.0;
    bool states_z_axis_left_arm_is_active = false;
    double p_gain_z_axis_left_arm_data = -80.0;
    double measured_lin_pos_y_axis_left_arm_data = 0.0;
    double apply_ee_force_z_axis_left_arm_data = 0.0;
    bool error_outside_threshold_to_deactivate_i_block_z_axis_left_arm_event = false;
    double lin_pos_sp_z_axis_left_arm_data = 0.03;
    bool states_z_axis_right_arm_is_active = false;
    double stiffness_lin_z_axis_right_arm_data = -100.0;
    bool vel_constraint_zero_vel_z_axis_left_arm_flag = false;
    bool pos_out_of_tolerance_about_setpoint_x_axis_left_arm_event = false;
    double measured_lin_pos_x_axis_left_arm_data = 0.0;
    bool vel_constraint_zero_vel_z_axis_right_arm_flag = false;
    double d_term_lin_vel_z_axis_left_arm_data = 0.0;
    double lin_pos_sp_uncertainty_range_z_axis_right_arm_data = 0.0;
    double i_term_lin_vel_z_axis_left_arm_data = 0.0;
    double lin_pos_sp_y_axis_left_arm_data = 0.3;
    double i_gain_lin_vel_z_axis_right_arm_data = 0.0;
    double lin_pos_sp_x_axis_left_arm_data = 0.5;
    double lin_pos_sp_uncertainty_range_z_axis_left_arm_data = 0.1;
    bool states_y_axis_left_arm_is_active = false;
    double damping_term_z_axis_right_arm_data = 0.0;
    double applied_force_threshold_apply_ee_force_z_axis_right_arm_data = 5.0;
    bool pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag = false;
    bool pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag = false;
    double measured_lin_vel_z_axis_right_arm_data = 0.0;
    double lin_zero_vel_z_axis_right_arm_data = 0.0;
    double stiffness_term_lin_z_axis_right_arm_data = 0.0;
    double measured_lin_pos_z_axis_left_arm_data = 0.0;
    double lin_vel_error_damping_z_axis_left_arm_data = 0.0;
    double lin_vel_sp_z_axis_right_arm_data = -0.025;
    double lin_pos_sp_equality_tolerance_z_axis_left_arm_data = 0.005;
    double integral_of_vel_term_error_z_axis_left_arm_data = 0.0;
    double lin_pos_sp_z_axis_right_arm_data = 0.0;
    double measured_lin_vel_y_axis_right_arm_data = 0.0;
    double apply_ee_force_y_axis_left_arm_data = 0.0;
    double lin_vel_sp_equality_tolerance_z_axis_left_arm_data = 0.005;
    bool net_applied_force_greater_than_threshold_z_axis_right_arm_flag = false;
    bool pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_event = false;
    bool error_within_threshold_to_activate_i_block_z_axis_right_arm_event = false;
    double d_term_lin_vel_z_axis_right_arm_data = 0.0;
    double force_to_apply_to_table_z_axis_left_arm_data = 5.0;
    bool pos_constraint_arm_above_pos_sp_z_axis_left_arm_event = false;
    double measured_lin_vel_x_axis_right_arm_data = 0.0;
    double measured_lin_vel_z_axis_left_arm_data = 0.0;
    double lin_pos_error_stiffness_z_axis_right_arm_data = 0.0;
    double measured_lin_vel_y_axis_left_arm_data = 0.0;
    bool states_x_axis_left_arm_is_active = false;
    double measured_lin_pos_z_axis_right_arm_data = 0.0;
    double measured_lin_pos_y_axis_right_arm_data = 0.0;
    double lin_pos_sp_equality_tolerance_y_axis_left_arm_data = 0.02;
    double i_gain_lin_vel_z_axis_left_arm_data = -100.0;
    double i_term_saturation_lower_limit_z_axis_left_arm_data = -5.0;
    double d_gain_lin_vel_z_axis_left_arm_data = -2.5;
    bool pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event = false;
    double p_gain_z_axis_right_arm_data = -80.0;
    bool pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event = false;
    double p_gain_pos_x_axis_left_arm_data = -50.0;
    bool pos_constraint_arm_below_setpoint_contact_region_z_axis_right_arm_flag = false;
    double lin_vel_error_damping_z_axis_right_arm_data = 0.0;
    double lin_vel_sp_equality_tolerance_z_axis_right_arm_data = 0.005;
    double d_term_lin_vel_error_z_axis_left_arm_data = 0.0;
    double previous_lin_vel_error_z_axis_left_arm_data = -0.025;
    double measured_lin_pos_x_axis_right_arm_data = 0.0;
    bool error_within_threshold_to_activate_i_block_z_axis_left_arm_event = false;
    double lin_vel_sp_threshold_to_activate_i_z_axis_left_arm_data = 0.01;
    bool error_outside_threshold_to_deactivate_i_block_z_axis_right_arm_event = false;
    double lin_pos_error_y_axis_left_arm_data = 0.0;
    double lin_pos_error_x_axis_left_arm_data = 0.0;

    // logging
    std::string log_file_pid_investigation = "log_files/log_file_pid_investigation.csv";
    std::ofstream file_pid_investigation(log_file_pid_investigation);

    if (!file_pid_investigation.is_open())
    {
        std::cerr << "Failed to open file: " << log_file_pid_investigation << std::endl;
        return 0;
    }
    // adding header
    file_pid_investigation << "time_period_of_complete_controller_cycle_data,measured_lin_pos_z_axis_left_arm_data,measured_lin_vel_z_axis_left_arm_data,lin_vel_error_pid_z_axis_left_arm_data,p_term_lin_vel_z_axis_left_arm_data,d_term_lin_vel_error_z_axis_left_arm_data,d_term_lin_vel_z_axis_left_arm_data,integral_of_vel_term_error_z_axis_left_arm_data,i_term_lin_vel_z_axis_left_arm_data,apply_ee_force_z_axis_left_arm_data\n"; // header

    std::vector<double> dataArray; // Vector to store data temporarily
    // initialise multi-dimensional array to store data
    std::vector<std::array<double, 10>> data_pid_investigation;
    int iterationCount = 0;

    auto start_time_of_task = std::chrono::high_resolution_clock::now(); // start_time_of_task.count() gives time in seconds
    const auto task_time_out = std::chrono::duration<double>(TIMEOUT_DURATION_TASK);
    auto previous_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed = std::chrono::duration<double>(previous_time - start_time_of_task);

    double measured_quat_GF_left_arm[4];
    double measured_quat_GF_right_arm[4];
    double desired_quat_GF_left_arm[4] = {0.7071, 0.0, 0.0, 0.7071};
    double desired_quat_GF_right_arm[4] = {0.0, 0.7071, 0.7071, 0.0};

    KDL::Vector angle_axis_diff_GF_right_arm;
    KDL::Vector angle_axis_diff_GF_left_arm;

    double stiffness_angular_left_arm[3] = {-15.0, -15., -15.};
    double stiffness_angular_right_arm[3] = {-15.0, -15., -15.};

    KDL::Frame desired_endEffPose_GF_left_arm;
    desired_endEffPose_GF_left_arm.M = KDL::Rotation::Quaternion(desired_quat_GF_left_arm[0], desired_quat_GF_left_arm[1], desired_quat_GF_left_arm[2], desired_quat_GF_left_arm[3]);
    KDL::Frame desired_endEffPose_GF_right_arm;
    desired_endEffPose_GF_right_arm.M = KDL::Rotation::Quaternion(desired_quat_GF_right_arm[0], desired_quat_GF_right_arm[1], desired_quat_GF_right_arm[2], desired_quat_GF_right_arm[3]);

    while (time_elapsed < task_time_out)
    {

        // if any interruption (Ctrl+C or window resizing) is detected
        if (flag == 1)
        {
            // logging
            // Write remaining data to file
            if (!dataArray.empty())
            {
                // appendDataToFile(file, dataArray);
                appendDataToFile_dynamic_size(file_pid_investigation, data_pid_investigation);
                data_pid_investigation.clear();
            }

            // file.close();
            // file_ee_pose.close();
            file_pid_investigation.close();
            std::cout << "Data collection completed.\n";
            break;
        }

        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_1_LEFT)
        {
            kinova_feedback(kinova_left, jnt_positions_left, jnt_velocities_left,
                            jnt_torques_left_read);
            get_end_effector_pose_and_twist(
                jnt_velocity_left, jnt_positions_left, jnt_velocities_left,
                measured_endEffPose_BL_left_arm, measured_endEffTwist_BL_left_arm,
                measured_endEffPose_GF_left_arm, measured_endEffTwist_GF_left_arm,
                fkSolverPos, fkSolverVel, BL_wrt_GF_frame_left);
        }

        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_2_RIGHT)
        {
            kinova_feedback(kinova_right, jnt_positions_right, jnt_velocities_right,
                            jnt_torques_right_read);
            get_end_effector_pose_and_twist(
                jnt_velocity_right, jnt_positions_right, jnt_velocities_right,
                measured_endEffPose_BL_right_arm, measured_endEffTwist_BL_right_arm,
                measured_endEffPose_GF_right_arm, measured_endEffTwist_GF_right_arm,
                fkSolverPos, fkSolverVel, BL_wrt_GF_frame_right);
        }

        // update the time variables
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_period = std::chrono::duration<double>(current_time - previous_time);

        while (time_period.count() < DESIRED_TIME_STEP)
        {
            current_time = std::chrono::high_resolution_clock::now();
            time_period = std::chrono::duration<double>(current_time - previous_time);
        }

        time_elapsed = std::chrono::duration<double>(current_time - start_time_of_task);
        previous_time = current_time;
        time_period_of_complete_controller_cycle_data = time_period.count();
        std::cout << "time_period: " << time_period_of_complete_controller_cycle_data << std::endl;

        measured_lin_pos_x_axis_left_arm_data = measured_endEffPose_GF_left_arm.p.x();
        measured_lin_vel_x_axis_left_arm_data = measured_endEffTwist_GF_left_arm.GetTwist().vel.x();
        measured_lin_pos_y_axis_left_arm_data = measured_endEffPose_GF_left_arm.p.y();
        measured_lin_vel_y_axis_left_arm_data = measured_endEffTwist_GF_left_arm.GetTwist().vel.y();
        measured_lin_pos_z_axis_left_arm_data = measured_endEffPose_GF_left_arm.p.z();
        measured_lin_vel_z_axis_left_arm_data = measured_endEffTwist_GF_left_arm.GetTwist().vel.z();

        measured_lin_pos_x_axis_right_arm_data = measured_endEffPose_GF_right_arm.p.x();
        measured_lin_vel_x_axis_right_arm_data = measured_endEffTwist_GF_right_arm.GetTwist().vel.x();
        measured_lin_pos_y_axis_right_arm_data = measured_endEffPose_GF_right_arm.p.y();
        measured_lin_vel_y_axis_right_arm_data = measured_endEffTwist_GF_right_arm.GetTwist().vel.y();
        measured_lin_pos_z_axis_right_arm_data = measured_endEffPose_GF_right_arm.p.z();
        measured_lin_vel_z_axis_right_arm_data = measured_endEffTwist_GF_right_arm.GetTwist().vel.z();

        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_1_LEFT)
        {
            measured_endEffPose_GF_left_arm.M.GetQuaternion(measured_quat_GF_left_arm[0], measured_quat_GF_left_arm[1], measured_quat_GF_left_arm[2], measured_quat_GF_left_arm[3]);
            angle_axis_diff_GF_left_arm = KDL::diff(measured_endEffPose_GF_left_arm.M, desired_endEffPose_GF_left_arm.M);
        }

        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_2_RIGHT)
        {
            measured_endEffPose_GF_right_arm.M.GetQuaternion(measured_quat_GF_right_arm[0], measured_quat_GF_right_arm[1], measured_quat_GF_right_arm[2], measured_quat_GF_right_arm[3]);
            angle_axis_diff_GF_right_arm = KDL::diff(measured_endEffPose_GF_right_arm.M, desired_endEffPose_GF_right_arm.M);
        }

        // for every new dimension in the robot
        // check if any motion specification satisfies pre condition
        if (!states_z_axis_right_arm_is_active)
        {
            // executing all functions of the pre-condition and checking if any set of flags are true
            greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_flag);

            if (pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_flag)
            {
                states_z_axis_right_arm_is_active = true;
            }
            // executing all functions of the pre-condition and checking if any set of flags are true
            in_interval_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag);

            if (pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag)
            {
                states_z_axis_right_arm_is_active = true;
            }
        }
        // if a pre-condition is satisfied, then execute the motion specification
        if (states_z_axis_right_arm_is_active)
        {
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_flag && states_z_axis_right_arm_is_active)
            {
                // execute all functions in post condition
                in_interval_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag);

                // check if post condition flags are true
                if (pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag)
                {
                    states_z_axis_right_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event);

                    out_of_interval_monitor(&measured_lin_vel_z_axis_right_arm_data, &lin_vel_sp_z_axis_right_arm_data, &lin_vel_sp_threshold_to_activate_i_z_axis_right_arm_data, &error_outside_threshold_to_deactivate_i_block_z_axis_right_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event && error_outside_threshold_to_deactivate_i_block_z_axis_right_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_vel_sp_z_axis_right_arm_data, &measured_lin_vel_z_axis_right_arm_data, &lin_vel_error_pid_z_axis_right_arm_data);

                        multiply2(&p_gain_z_axis_right_arm_data, &lin_vel_error_pid_z_axis_right_arm_data, &p_term_lin_vel_z_axis_right_arm_data);

                        differentiator(&lin_vel_error_pid_z_axis_right_arm_data, &previous_lin_vel_error_pid_z_axis_right_arm_data, &time_period_of_complete_controller_cycle_data, &d_term_lin_vel_error_z_axis_right_arm_data);

                        set_value_of_first_to_second_variable(&lin_vel_error_pid_z_axis_right_arm_data, &previous_lin_vel_error_pid_z_axis_right_arm_data);

                        multiply2(&d_gain_lin_vel_z_axis_right_arm_data, &d_term_lin_vel_error_z_axis_right_arm_data, &d_term_lin_vel_z_axis_right_arm_data);

                        summation2(&p_term_lin_vel_z_axis_right_arm_data, &d_term_lin_vel_z_axis_right_arm_data, &apply_ee_force_z_axis_right_arm_data);

                        // set all flags in the schedule to false
                        pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event = false;
                        error_outside_threshold_to_deactivate_i_block_z_axis_right_arm_event = false;
                    }
                    in_interval_monitor(&measured_lin_vel_z_axis_right_arm_data, &lin_vel_sp_z_axis_right_arm_data, &lin_vel_sp_threshold_to_activate_i_z_axis_right_arm_data, &error_within_threshold_to_activate_i_block_z_axis_right_arm_event);

                    greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event);

                    // check if all flags in the schedule are true

                    if (error_within_threshold_to_activate_i_block_z_axis_right_arm_event && pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_vel_sp_z_axis_right_arm_data, &measured_lin_vel_z_axis_right_arm_data, &lin_vel_error_pid_z_axis_right_arm_data);

                        multiply2(&p_gain_z_axis_right_arm_data, &lin_vel_error_pid_z_axis_right_arm_data, &p_term_lin_vel_z_axis_right_arm_data);

                        differentiator(&lin_vel_error_pid_z_axis_right_arm_data, &previous_lin_vel_error_pid_z_axis_right_arm_data, &time_period_of_complete_controller_cycle_data, &d_term_lin_vel_error_z_axis_right_arm_data);

                        set_value_of_first_to_second_variable(&lin_vel_error_pid_z_axis_right_arm_data, &previous_lin_vel_error_pid_z_axis_right_arm_data);

                        multiply2(&d_gain_lin_vel_z_axis_right_arm_data, &d_term_lin_vel_error_z_axis_right_arm_data, &d_term_lin_vel_z_axis_right_arm_data);

                        integrator(&time_period_of_complete_controller_cycle_data, &lin_vel_error_pid_z_axis_right_arm_data, &integral_of_vel_term_error_z_axis_right_arm_data);

                        multiply2(&i_gain_lin_vel_z_axis_right_arm_data, &integral_of_vel_term_error_z_axis_right_arm_data, &i_term_lin_vel_z_axis_right_arm_data);

                        saturation(&i_term_lin_vel_z_axis_right_arm_data, &i_term_saturation_lower_limit_z_axis_right_arm_data, &i_term_saturation_upper_limit_z_axis_right_arm_data);

                        summation3(&p_term_lin_vel_z_axis_right_arm_data, &i_term_lin_vel_z_axis_right_arm_data, &d_term_lin_vel_z_axis_right_arm_data, &apply_ee_force_z_axis_right_arm_data);

                        // set all flags in the schedule to false
                        error_within_threshold_to_activate_i_block_z_axis_right_arm_event = false;
                        pos_constraint_arm_above_uncertain_contact_region_z_axis_right_arm_event = false;
                    }
                }
            }
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_flag && states_z_axis_right_arm_is_active)
            {
                // execute all functions in post condition
                less_than_equal_to_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &pos_constraint_arm_below_setpoint_contact_region_z_axis_right_arm_flag);

                lower_than_upper_limit_monitor(&measured_lin_vel_z_axis_right_arm_data, &lin_zero_vel_z_axis_right_arm_data, &lin_vel_sp_equality_tolerance_z_axis_right_arm_data, &vel_constraint_zero_vel_z_axis_right_arm_flag);

                greater_than_monitor(&apply_ee_force_z_axis_right_arm_data, &applied_force_threshold_apply_ee_force_z_axis_right_arm_data, &net_applied_force_greater_than_threshold_z_axis_right_arm_flag);

                // check if post condition flags are true
                if (pos_constraint_arm_below_setpoint_contact_region_z_axis_right_arm_flag && vel_constraint_zero_vel_z_axis_right_arm_flag && net_applied_force_greater_than_threshold_z_axis_right_arm_flag)
                {
                    states_z_axis_right_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    in_interval_monitor(&measured_lin_pos_z_axis_right_arm_data, &lin_pos_sp_z_axis_right_arm_data, &lin_pos_sp_uncertainty_range_z_axis_right_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_pos_sp_z_axis_right_arm_data, &measured_lin_pos_z_axis_right_arm_data, &lin_pos_error_stiffness_z_axis_right_arm_data);

                        multiply2(&stiffness_lin_z_axis_right_arm_data, &lin_pos_error_stiffness_z_axis_right_arm_data, &stiffness_term_lin_z_axis_right_arm_data);

                        subtraction(&lin_vel_sp_z_axis_right_arm_data, &measured_lin_vel_z_axis_right_arm_data, &lin_vel_error_damping_z_axis_right_arm_data);

                        multiply2(&damping_lin_z_axis_right_arm_data, &lin_vel_error_damping_z_axis_right_arm_data, &damping_term_z_axis_right_arm_data);

                        summation2(&stiffness_term_lin_z_axis_right_arm_data, &damping_term_z_axis_right_arm_data, &apply_ee_force_z_axis_right_arm_data);

                        // set all flags in the schedule to false
                        pos_constraint_arm_in_uncertain_contact_region_z_axis_right_arm_event = false;
                    }
                }
            }
        }
        // for every new dimension in the robot
        // check if any motion specification satisfies pre condition
        if (!states_z_axis_left_arm_is_active)
        {
            // executing all functions of the pre-condition and checking if any set of flags are true
            greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_flag);

            if (pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_flag)
            {
                states_z_axis_left_arm_is_active = true;
            }
            // executing all functions of the pre-condition and checking if any set of flags are true
            in_interval_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag);

            greater_than_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &pos_constraint_arm_above_pos_sp_z_axis_left_arm_flag);

            if (pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag && pos_constraint_arm_above_pos_sp_z_axis_left_arm_flag)
            {
                states_z_axis_left_arm_is_active = true;
            }
            // executing all functions of the pre-condition and checking if any set of flags are true
            less_than_equal_to_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag);

            if (pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag)
            {
                states_z_axis_left_arm_is_active = true;
            }
        }
        // if a pre-condition is satisfied, then execute the motion specification
        if (states_z_axis_left_arm_is_active)
        {
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_flag && states_z_axis_left_arm_is_active)
            {
                // execute all functions in post condition
                in_interval_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag);

                // check if post condition flags are true
                if (pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag)
                {
                    states_z_axis_left_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event);

                    out_of_interval_monitor(&measured_lin_vel_z_axis_left_arm_data, &lin_vel_sp_z_axis_left_arm_data, &lin_vel_sp_threshold_to_activate_i_z_axis_left_arm_data, &error_outside_threshold_to_deactivate_i_block_z_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event && error_outside_threshold_to_deactivate_i_block_z_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_vel_sp_z_axis_left_arm_data, &measured_lin_vel_z_axis_left_arm_data, &lin_vel_error_pid_z_axis_left_arm_data);

                        multiply2(&p_gain_z_axis_left_arm_data, &lin_vel_error_pid_z_axis_left_arm_data, &p_term_lin_vel_z_axis_left_arm_data);

                        differentiator(&lin_vel_error_pid_z_axis_left_arm_data, &previous_lin_vel_error_z_axis_left_arm_data, &time_period_of_complete_controller_cycle_data, &d_term_lin_vel_error_z_axis_left_arm_data);

                        set_value_of_first_to_second_variable(&lin_vel_error_pid_z_axis_left_arm_data, &previous_lin_vel_error_z_axis_left_arm_data);

                        multiply2(&d_gain_lin_vel_z_axis_left_arm_data, &d_term_lin_vel_error_z_axis_left_arm_data, &d_term_lin_vel_z_axis_left_arm_data);

                        summation2(&p_term_lin_vel_z_axis_left_arm_data, &d_term_lin_vel_z_axis_left_arm_data, &apply_ee_force_z_axis_left_arm_data);

                        // set all flags in the schedule to false
                        pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event = false;
                        error_outside_threshold_to_deactivate_i_block_z_axis_left_arm_event = false;
                    }
                    in_interval_monitor(&measured_lin_vel_z_axis_left_arm_data, &lin_vel_sp_z_axis_left_arm_data, &lin_vel_sp_threshold_to_activate_i_z_axis_left_arm_data, &error_within_threshold_to_activate_i_block_z_axis_left_arm_event);

                    greater_than_upper_limit_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (error_within_threshold_to_activate_i_block_z_axis_left_arm_event && pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_vel_sp_z_axis_left_arm_data, &measured_lin_vel_z_axis_left_arm_data, &lin_vel_error_pid_z_axis_left_arm_data);

                        multiply2(&p_gain_z_axis_left_arm_data, &lin_vel_error_pid_z_axis_left_arm_data, &p_term_lin_vel_z_axis_left_arm_data);

                        differentiator(&lin_vel_error_pid_z_axis_left_arm_data, &previous_lin_vel_error_z_axis_left_arm_data, &time_period_of_complete_controller_cycle_data, &d_term_lin_vel_error_z_axis_left_arm_data);

                        set_value_of_first_to_second_variable(&lin_vel_error_pid_z_axis_left_arm_data, &previous_lin_vel_error_z_axis_left_arm_data);

                        multiply2(&d_gain_lin_vel_z_axis_left_arm_data, &d_term_lin_vel_error_z_axis_left_arm_data, &d_term_lin_vel_z_axis_left_arm_data);

                        integrator(&time_period_of_complete_controller_cycle_data, &lin_vel_error_pid_z_axis_left_arm_data, &integral_of_vel_term_error_z_axis_left_arm_data);

                        multiply2(&i_gain_lin_vel_z_axis_left_arm_data, &integral_of_vel_term_error_z_axis_left_arm_data, &i_term_lin_vel_z_axis_left_arm_data);

                        saturation(&i_term_lin_vel_z_axis_left_arm_data, &i_term_saturation_lower_limit_z_axis_left_arm_data, &i_term_saturation_upper_limit_z_axis_left_arm_data);

                        summation3(&p_term_lin_vel_z_axis_left_arm_data, &i_term_lin_vel_z_axis_left_arm_data, &d_term_lin_vel_z_axis_left_arm_data, &apply_ee_force_z_axis_left_arm_data);

                        // set all flags in the schedule to false
                        error_within_threshold_to_activate_i_block_z_axis_left_arm_event = false;
                        pos_constraint_arm_above_uncertain_contact_region_z_axis_left_arm_event = false;
                    }
                }
            }
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_flag && pos_constraint_arm_above_pos_sp_z_axis_left_arm_flag && states_z_axis_left_arm_is_active)
            {
                // execute all functions in post condition
                less_than_equal_to_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag);

                // check if post condition flags are true
                if (pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag)
                {
                    states_z_axis_left_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    in_interval_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &lin_pos_sp_uncertainty_range_z_axis_left_arm_data, &pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_event);

                    greater_than_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &pos_constraint_arm_above_pos_sp_z_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_event && pos_constraint_arm_above_pos_sp_z_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_pos_sp_z_axis_left_arm_data, &measured_lin_pos_z_axis_left_arm_data, &lin_pos_error_stiffness_z_axis_left_arm_data);

                        multiply2(&stiffness_lin_z_axis_left_arm_data, &lin_pos_error_stiffness_z_axis_left_arm_data, &stiffness_term_lin_z_axis_left_arm_data);

                        subtraction(&lin_vel_sp_z_axis_left_arm_data, &measured_lin_vel_z_axis_left_arm_data, &lin_vel_error_damping_z_axis_left_arm_data);

                        multiply2(&damping_lin_z_axis_left_arm_data, &lin_vel_error_damping_z_axis_left_arm_data, &damping_term_z_axis_left_arm_data);

                        summation2(&stiffness_term_lin_z_axis_left_arm_data, &damping_term_z_axis_left_arm_data, &apply_ee_force_z_axis_left_arm_data);

                        // set all flags in the schedule to false
                        pos_constraint_arm_in_uncertain_contact_region_z_axis_left_arm_event = false;
                        pos_constraint_arm_above_pos_sp_z_axis_left_arm_event = false;
                    }
                }
            }
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_flag && states_z_axis_left_arm_is_active)
            {
                // execute all functions in post condition
                // check if post condition flags are true
                if (false)
                {
                    states_z_axis_left_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    less_than_equal_to_monitor(&measured_lin_pos_z_axis_left_arm_data, &lin_pos_sp_z_axis_left_arm_data, &pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        set_value_of_first_to_second_variable(&force_to_apply_to_table_z_axis_left_arm_data, &apply_ee_force_z_axis_left_arm_data);

                        // set all flags in the schedule to false
                        pos_constraint_arm_below_setpoint_contact_region_z_axis_left_arm_event = false;
                    }
                }
            }
        }
        // for every new dimension in the robot
        // check if any motion specification satisfies pre condition
        if (!states_x_axis_left_arm_is_active)
        {
            // executing all functions of the pre-condition and checking if any set of flags are true

            if (true)
            {
                states_x_axis_left_arm_is_active = true;
            }
        }
        // if a pre-condition is satisfied, then execute the motion specification
        if (states_x_axis_left_arm_is_active)
        {
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (true && states_x_axis_left_arm_is_active)
            {
                // execute all functions in post condition
                // check if post condition flags are true
                if (false)
                {
                    states_x_axis_left_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    out_of_interval_monitor(&measured_lin_pos_x_axis_left_arm_data, &lin_pos_sp_x_axis_left_arm_data, &lin_pos_sp_equality_tolerance_x_axis_left_arm_data, &pos_out_of_tolerance_about_setpoint_x_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_out_of_tolerance_about_setpoint_x_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_pos_sp_x_axis_left_arm_data, &measured_lin_pos_x_axis_left_arm_data, &lin_pos_error_x_axis_left_arm_data);

                        multiply2(&p_gain_pos_x_axis_left_arm_data, &lin_pos_error_x_axis_left_arm_data, &apply_ee_force_x_axis_left_arm_data);

                        // set all flags in the schedule to false
                        pos_out_of_tolerance_about_setpoint_x_axis_left_arm_event = false;
                    }
                }
            }
        }
        // for every new dimension in the robot
        // check if any motion specification satisfies pre condition
        if (!states_y_axis_left_arm_is_active)
        {
            // executing all functions of the pre-condition and checking if any set of flags are true

            if (true)
            {
                states_y_axis_left_arm_is_active = true;
            }
        }
        // if a pre-condition is satisfied, then execute the motion specification
        if (states_y_axis_left_arm_is_active)
        {
            // for each motion spec node ...
            // if combination of pre-cond flags are true
            if (true && states_y_axis_left_arm_is_active)
            {
                // execute all functions in post condition
                // check if post condition flags are true
                if (false)
                {
                    states_y_axis_left_arm_is_active = false;
                }
                else
                {
                    // loop through each schedule and execute corresponding function names in monitors key
                    out_of_interval_monitor(&measured_lin_pos_y_axis_left_arm_data, &lin_pos_sp_y_axis_left_arm_data, &lin_pos_sp_equality_tolerance_y_axis_left_arm_data, &pos_out_of_tolerance_about_setpoint_y_axis_left_arm_event);

                    // check if all flags in the schedule are true

                    if (pos_out_of_tolerance_about_setpoint_y_axis_left_arm_event)
                    {
                        // execute all functions in the trigger chain
                        subtraction(&lin_pos_sp_y_axis_left_arm_data, &measured_lin_pos_y_axis_left_arm_data, &lin_pos_error_y_axis_left_arm_data);

                        multiply2(&p_gain_pos_y_axis_left_arm_data, &lin_pos_error_y_axis_left_arm_data, &apply_ee_force_y_axis_left_arm_data);

                        // set all flags in the schedule to false
                        pos_out_of_tolerance_about_setpoint_y_axis_left_arm_event = false;
                    }
                }
            }
        }

        // send the torques to the robot
        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_1_LEFT)
        {
            // write the ee torques to to linkWrenches_GF_left
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(0) = apply_ee_force_x_axis_left_arm_data;
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(1) = apply_ee_force_y_axis_left_arm_data;
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(2) = apply_ee_force_z_axis_left_arm_data;
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(0) = stiffness_angular_left_arm[0] * angle_axis_diff_GF_left_arm(0);
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(1) = stiffness_angular_left_arm[1] * angle_axis_diff_GF_left_arm(1);
            linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(2) = stiffness_angular_left_arm[2] * angle_axis_diff_GF_left_arm(2);

            // thresholding in cartesian space of the end effector
            for (int i = 0; i < 3; i++)
            {
                if (linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(i) > 0.0)
                {
                    linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(i) = std::min(WRENCH_THRESHOLD_LINEAR, linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(i));
                }
                else
                {
                    linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(i) = std::max(-WRENCH_THRESHOLD_LINEAR, linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].force(i));
                }

                if (linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(i) > 0.0)
                {
                    linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(i) = std::min(WRENCH_THRESHOLD_ROTATIONAL, linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(i));
                }
                else
                {
                    linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(i) = std::max(-WRENCH_THRESHOLD_ROTATIONAL, linkWrenches_GF_left[kinova_constants::NUMBER_OF_JOINTS].torque(i));
                }
            }

            // LinkWrenches are calculated in BL frame. As RNE solver requires them in EE frame, the wrenches are transformed from BL to EE frame
            linkWrenches_EE_left[NUM_LINKS - 1].force = measured_endEffPose_GF_left_arm.M.Inverse() * linkWrenches_GF_left[NUM_LINKS - 1].force;
            linkWrenches_EE_left[NUM_LINKS - 1].torque = measured_endEffPose_GF_left_arm.M.Inverse() * linkWrenches_GF_left[NUM_LINKS - 1].torque;

            calculate_joint_torques_RNEA(jacobDotSolver, ikSolverAcc, idSolver_left,
                                         jnt_velocity_left, jd_qd_left, xdd_left,
                                         xdd_minus_jd_qd_left, jnt_accelerations_left,
                                         jnt_positions_left, jnt_velocities_left,
                                         linkWrenches_EE_left, jnt_torques_left_cmd);

            // thresholding the jnt_torques_left_cmd before sending to the robot
            for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            {
                if (jnt_torques_left_cmd(i) > 0.0)
                {
                    jnt_torques_left_cmd(i) = std::min(JOINT_TORQUE_THRESHOLD, jnt_torques_left_cmd(i));
                }
                else
                {
                    jnt_torques_left_cmd(i) = std::max(-JOINT_TORQUE_THRESHOLD, jnt_torques_left_cmd(i));
                }
            }
            kinova_left.set_joint_torques(jnt_torques_left_cmd);
        }

        if (robots_to_control == robot_controlled::DUAL_KINOVA_GEN3_ARMS || robots_to_control == robot_controlled::KINOVA_GEN3_2_RIGHT)
        {
            // write the ee torques to to linkWrenches_GF_right
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(0) = apply_ee_force_x_axis_right_arm_data;
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(1) = apply_ee_force_y_axis_right_arm_data;
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(2) = apply_ee_force_z_axis_right_arm_data;
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(0) = stiffness_angular_right_arm[0] * angle_axis_diff_GF_right_arm(0);
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(1) = stiffness_angular_right_arm[1] * angle_axis_diff_GF_right_arm(1);
            linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(2) = stiffness_angular_right_arm[2] * angle_axis_diff_GF_right_arm(2);

            // thresholding in cartesian space of the end effector
            for (int i = 0; i < 3; i++)
            {
                if (linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(i) > 0.0)
                {
                    linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(i) = std::min(WRENCH_THRESHOLD_LINEAR, linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(i));
                }
                else
                {
                    linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(i) = std::max(-WRENCH_THRESHOLD_LINEAR, linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].force(i));
                }

                if (linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(i) > 0.0)
                {
                    linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(i) = std::min(WRENCH_THRESHOLD_ROTATIONAL, linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(i));
                }
                else
                {
                    linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(i) = std::max(-WRENCH_THRESHOLD_ROTATIONAL, linkWrenches_GF_right[kinova_constants::NUMBER_OF_JOINTS].torque(i));
                }
            }

            // LinkWrenches are calculated in BL frame. As RNE solver requires them in EE frame, the wrenches are transformed from BL to EE frame
            linkWrenches_EE_right[NUM_LINKS - 1].force = measured_endEffPose_GF_right_arm.M.Inverse() * linkWrenches_GF_right[NUM_LINKS - 1].force;
            linkWrenches_EE_right[NUM_LINKS - 1].torque = measured_endEffPose_GF_right_arm.M.Inverse() * linkWrenches_GF_right[NUM_LINKS - 1].torque;

            calculate_joint_torques_RNEA(jacobDotSolver, ikSolverAcc, idSolver_right,
                                         jnt_velocity_right, jd_qd_right, xdd_right,
                                         xdd_minus_jd_qd_right, jnt_accelerations_right,
                                         jnt_positions_right, jnt_velocities_right,
                                         linkWrenches_EE_right, jnt_torques_right_cmd);

            // thresholding the jnt_torques_right_cmd before sending to the robot
            for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            {
                if (jnt_torques_right_cmd(i) > 0.0)
                {
                    jnt_torques_right_cmd(i) = std::min(JOINT_TORQUE_THRESHOLD, jnt_torques_right_cmd(i));
                }
                else
                {
                    jnt_torques_right_cmd(i) = std::max(-JOINT_TORQUE_THRESHOLD, jnt_torques_right_cmd(i));
                }
            }
            kinova_right.set_joint_torques(jnt_torques_right_cmd);
        }

        data_pid_investigation.push_back({time_period_of_complete_controller_cycle_data, measured_lin_pos_z_axis_left_arm_data, measured_lin_vel_z_axis_left_arm_data, lin_vel_error_pid_z_axis_left_arm_data, p_term_lin_vel_z_axis_left_arm_data, d_term_lin_vel_error_z_axis_left_arm_data, d_term_lin_vel_z_axis_left_arm_data, integral_of_vel_term_error_z_axis_left_arm_data, i_term_lin_vel_z_axis_left_arm_data, apply_ee_force_z_axis_left_arm_data});
        iterationCount++;
        // Check if we should write to file (every 100 iterations)
        if (iterationCount % SAVE_LOG_EVERY_NTH_STEP == 0)
        {
            appendDataToFile_dynamic_size(file_pid_investigation, data_pid_investigation);
            data_pid_investigation.clear();
        }
    }

    if (!dataArray.empty())
    {
        appendDataToFile_dynamic_size(file_pid_investigation, data_pid_investigation);
        data_pid_investigation.clear();
    }

    file_pid_investigation.close();
    std::cout << "Data collection completed.\n";
    return 0;
}
