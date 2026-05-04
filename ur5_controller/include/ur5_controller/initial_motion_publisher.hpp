#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// Forward declarations
namespace ur5_controller {
    struct RobotState;
    struct CartesianState;
    struct NodeConfig;
    class JointStateMapper;
}

namespace ur5_controller {

/**
 * @class InitialMotionPublisher
 * @brief Manages the initial motion phase of the UR5 controller.
 * 
 * Handles stepwise publication of initial joint positions, detects when the robot
 * starts moving and when it reaches the target configuration, and manages transition
 * to automatic trajectory mode.
 */
class InitialMotionPublisher
{
public:
    /**
     * @brief Constructor
     * @param logger ROS logger for diagnostics
     */
    explicit InitialMotionPublisher(const rclcpp::Logger& logger);

    /**
     * @brief Configure the motion publisher with node dependencies
     * @param robot_state Reference to robot state struct
     * @param cartesian_state Reference to cartesian state struct
     * @param config Reference to node configuration
     * @param joint_position_pub Publisher for joint position commands
     * @param joint_state_mapper Reference to joint state mapper
     * @param pinocchio_model Pointer to pinocchio model
     * @param pinocchio_data Pointer to pinocchio data
     * @param tool_frame_id Frame ID of the end effector
     */
    void configure(
        RobotState& robot_state,
        CartesianState& cartesian_state,
        NodeConfig& config,
        const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& joint_position_pub,
        const JointStateMapper& joint_state_mapper,
        const pinocchio::Model* pinocchio_model,
        pinocchio::Data* pinocchio_data,
        int tool_frame_id);

    /**
     * @brief Initialize and activate the initial motion phase and create internal timer
     * @param node Pointer to the node used to create the timer
     * @param ctrl_hz Control loop frequency used to set timer period
     * @return Pointer to the created timer
     */
    rclcpp::TimerBase::SharedPtr initialize(rclcpp::Node* node, double ctrl_hz);

    /**
     * @brief Periodic tick callback for the initial motion timer
     */
    void tick();

    /**
     * @brief Check if initial motion is currently active
     */
    bool isActive() const { return init_move_active_; }

    /**
     * @brief Get reference to the timer for external control (e.g., cancellation)
     */
    rclcpp::TimerBase::SharedPtr getTimer() { return init_move_timer_; }

    /**
     * @brief Reset the motion publisher state (for testing or reinitialization)
     */
    void reset();

    /**
     * @brief Set the last received joint state (used for baseline capture)
     */
    void setLastJointState(const sensor_msgs::msg::JointState::SharedPtr& msg) {
        last_joint_state_ = msg;
    }

private:
    rclcpp::Logger logger_;

    // Pointers to external state (not owned)
    RobotState* robot_state_ = nullptr;
    CartesianState* cartesian_state_ = nullptr;
    NodeConfig* config_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
    const JointStateMapper* joint_state_mapper_ = nullptr;
    const pinocchio::Model* pinocchio_model_ = nullptr;
    pinocchio::Data* pinocchio_data_ = nullptr;
    int tool_frame_id_ = -1;

    // Last received joint state message (for baseline capture verification)
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    // Timer for periodic execution
    rclcpp::TimerBase::SharedPtr init_move_timer_;
    // Node pointer for creating transient timers (not owned)
    rclcpp::Node* node_ = nullptr;
    // One-shot timer to delay trajectory activation after reaching target
    rclcpp::TimerBase::SharedPtr activation_timer_;

    // State tracking
    bool init_move_active_ = false;              // Publishing until movement detected
    bool init_baseline_set_ = false;             // Baseline position captured
    bool init_movement_started_ = false;         // Robot has started moving
    bool step_publishing_initialized_ = false;   // Step sequence initialized

    // Baseline joint positions
    std::vector<double> q_baseline_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Step publication state
    std::vector<double> step_error_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // Current error per joint
    std::vector<double> step_q_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};       // Current position being published
    int step_count_ = 0;                         // Number of steps already published
    int step_total_ = 0;                         // Total steps required

    // Thresholds and counters
    double move_detect_threshold_ = 0.010;       // rad, ~0.57 deg
    double reach_threshold_rad_ = 0.020;         // rad, closeness to q_target
    int reach_count_ = 0;                        // Consecutive verifications within threshold
    int reach_count_required_ = 5;               // Required consecutive verifications to confirm arrival

    // Private methods
    /**
     * @brief Publish a single step of the initial motion
     */
    void publish_initial_joint_position();

    /**
     * @brief Capture baseline joint positions for movement detection
     */
    void capture_baseline();

    /**
     * @brief Detect if the robot has started moving from baseline
     */
    void detect_movement();

    /**
     * @brief Check if the robot has reached the target configuration
     */
    bool check_target_reached();

    /**
     * @brief Handle transition when target is reached
     */
    void on_target_reached();
};

} // namespace ur5_controller
