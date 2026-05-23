#include "ur5_controller/initial_motion_publisher.hpp"
#include "ur5_controller/structs.hpp"
#include "ur5_controller/joint_state_mapper.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <array>
#include <utility>

namespace ur5_controller {

InitialMotionPublisher::InitialMotionPublisher(const rclcpp::Logger& logger)
    : logger_(logger)
{
}

void InitialMotionPublisher::configure(
    RobotState& robot_state,
    CartesianState& cartesian_state,
    NodeConfig& config,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& joint_trajectory_pub,
    const JointStateMapper& joint_state_mapper,
    const pinocchio::Model* pinocchio_model,
    pinocchio::Data* pinocchio_data,
    int tool_frame_id)
{
    robot_state_ = &robot_state;
    cartesian_state_ = &cartesian_state;
    config_ = &config;
    joint_trajectory_pub_ = joint_trajectory_pub;
    joint_state_mapper_ = &joint_state_mapper;
    pinocchio_model_ = pinocchio_model;
    pinocchio_data_ = pinocchio_data;
    tool_frame_id_ = tool_frame_id;
}

rclcpp::TimerBase::SharedPtr InitialMotionPublisher::initialize(rclcpp::Node* node, double ctrl_hz)
{
    if (!robot_state_ || !config_) {
        RCLCPP_ERROR(logger_, "InitialMotionPublisher not properly configured before initialize()");
        return nullptr;
    }

    if (!config_->use_ur5_pos_init) {
        RCLCPP_DEBUG(logger_, "Initial motion is disabled (use_ur5_pos_init=false)");
        return nullptr;
    }

    init_move_active_ = true;
    init_baseline_set_ = false;
    step_publishing_initialized_ = false;
    trajectory_sent_ = false;
    step_count_ = 0;
    step_total_ = 0;
    reach_count_ = 0;
    init_movement_started_ = false;

    RCLCPP_INFO(logger_, "InitialMotionPublisher initialized. Waiting for baseline capture...");

    if (node) {
        // Create and store the timer internally so it remains alive
        node_ = node;
        init_move_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, ctrl_hz))),
            std::bind(&InitialMotionPublisher::tick, this)
        );
    }

    return init_move_timer_;
}

void InitialMotionPublisher::tick()
{
    if (!init_move_active_) {
        return;
    }

    // Ensure baseline is captured on first joint state
    if (!init_baseline_set_) {
        capture_baseline();
        if (!init_baseline_set_) {
            return;  // Not ready yet
        }
    }

    // Detect if movement has started
    detect_movement();

    // Check if target is reached with hysteresis
    if (check_target_reached()) {
        on_target_reached();
        return;
    }

    // Publish the initial trajectory once while the robot hasn't reached the target
    publish_initial_trajectory();
}

void InitialMotionPublisher::reset()
{
    init_move_active_ = false;
    init_baseline_set_ = false;
    init_movement_started_ = false;
    step_publishing_initialized_ = false;
    trajectory_sent_ = false;
    step_count_ = 0;
    step_total_ = 0;
    reach_count_ = 0;

    std::fill(q_baseline_.begin(), q_baseline_.end(), 0.0);
    std::fill(step_error_.begin(), step_error_.end(), 0.0);
    std::fill(step_q_.begin(), step_q_.end(), 0.0);
}

void InitialMotionPublisher::capture_baseline()
{
    if (!robot_state_ || !joint_state_mapper_) {
        return;
    }

    // We need to have received joint_states and have valid mapping
    if (last_joint_state_ && joint_state_mapper_->isReady()) {
        for (int i = 0; i < 6; ++i) {
            q_baseline_[i] = robot_state_->q[i];
        }
        init_baseline_set_ = true;

        RCLCPP_INFO(logger_,
            "Baseline joint position captured for movement detection: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            q_baseline_[0], q_baseline_[1], q_baseline_[2],
            q_baseline_[3], q_baseline_[4], q_baseline_[5]);
    }
}

void InitialMotionPublisher::detect_movement()
{
    if (!robot_state_ || init_movement_started_) {
        return;
    }

    if (!init_baseline_set_) {
        return;
    }

    // Check if any joint has moved beyond threshold from baseline
    for (int i = 0; i < 6; ++i) {
        if (std::abs(robot_state_->q[i] - q_baseline_[i]) > move_detect_threshold_) {
            init_movement_started_ = true;
            RCLCPP_INFO(logger_,
                "Initial movement detected (> %.3f rad). Continuing until target is reached.",
                move_detect_threshold_);
            break;
        }
    }
}

bool InitialMotionPublisher::check_target_reached()
{
    if (!robot_state_ || !config_) {
        return false;
    }

    // Calculate maximum error across all joints
    double max_err = 0.0;
    for (int i = 0; i < 6; ++i) {
        max_err = std::max(max_err, std::abs(robot_state_->q[i] - config_->q_target[i]));
    }

    // Hysteresis: count consecutive verifications within threshold
    if (max_err < reach_threshold_rad_) {
        reach_count_++;
    } else {
        reach_count_ = 0;
    }

    return (reach_count_ >= reach_count_required_);
}

void InitialMotionPublisher::on_target_reached()
{
    if (!robot_state_ || !cartesian_state_ || !config_) {
        return;
    }

    init_move_active_ = false;
    step_publishing_initialized_ = false;

    if (init_move_timer_) {
        init_move_timer_->cancel();
    }

    RCLCPP_INFO(logger_,
        "Target joint configuration reached (|e|_max < %.3f rad for %d cycles).",
        reach_threshold_rad_, reach_count_required_);

    // Reconfigure initial pose based on current position before starting trajectory
    if (!config_->use_geomagic && pinocchio_model_ && pinocchio_data_) {
        pinocchio::forwardKinematics(*pinocchio_model_, *pinocchio_data_, robot_state_->q);
        pinocchio::updateGlobalPlacements(*pinocchio_model_, *pinocchio_data_);
        const auto& frame_now = pinocchio_data_->oMf[tool_frame_id_];

        cartesian_state_->position_initial = frame_now.translation();
        cartesian_state_->rotation_matrix_initial = frame_now.rotation();
        cartesian_state_->orientation_initial = Eigen::Quaterniond(frame_now.rotation());

        RCLCPP_INFO(logger_,
            "Initial pose (x_init, R_init) reconfigured to current pose before trajectory activation.");
        RCLCPP_INFO(logger_,
            "Waiting 1 second before activating automatic trajectory...");
        // Create a one-shot timer to signal the node to start the automatic trajectory
        if (node_) {
            // Cancel any existing activation timer
            if (activation_timer_) activation_timer_->cancel();
            activation_timer_ = node_->create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    if (config_) {
                        config_->initial_motion_done = true;
                        RCLCPP_INFO(logger_, "Initial motion done: signaling node to activate trajectory.");
                    }
                    if (activation_timer_) activation_timer_->cancel();
                }
            );
        } else {
            // If no node pointer available, set flag immediately
            config_->initial_motion_done = true;
            RCLCPP_INFO(logger_, "Initial motion done (no node): signaling node to activate trajectory immediately.");
        }
    }
}

void InitialMotionPublisher::publish_initial_trajectory()
{
    if (!robot_state_ || !config_ || !joint_trajectory_pub_) {
        return;
    }

    if (config_->q_target.size() != 6) {
        RCLCPP_ERROR(logger_, "q_target must have 6 elements, got %zu", config_->q_target.size());
        return;
    }

    if (trajectory_sent_) {
        return;
    }

    std::vector<double> robot_q(robot_state_->q.data(), robot_state_->q.data() + 6);
    std::array<std::string, 6> joint_names = {
        config_->nmspace.empty() ? std::string("shoulder_pan_joint") : config_->nmspace + std::string("_shoulder_pan_joint"),
        config_->nmspace.empty() ? std::string("shoulder_lift_joint") : config_->nmspace + std::string("_shoulder_lift_joint"),
        config_->nmspace.empty() ? std::string("elbow_joint") : config_->nmspace + std::string("_elbow_joint"),
        config_->nmspace.empty() ? std::string("wrist_1_joint") : config_->nmspace + std::string("_wrist_1_joint"),
        config_->nmspace.empty() ? std::string("wrist_2_joint") : config_->nmspace + std::string("_wrist_2_joint"),
        config_->nmspace.empty() ? std::string("wrist_3_joint") : config_->nmspace + std::string("_wrist_3_joint")
    };

    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
    trajectory_msg.joint_names.assign(joint_names.begin(), joint_names.end());

    const double duration = std::max(0.5, config_->q_target_time);
    const int num_points = std::max(2, static_cast<int>(std::ceil(duration * 20.0)));

    for (int i = 1; i <= num_points; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(num_points);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(6);
        for (size_t j = 0; j < 6; ++j) {
            point.positions[j] = robot_q[j] + ratio * (config_->q_target[j] - robot_q[j]);
        }
        point.time_from_start = rclcpp::Duration::from_seconds(duration * ratio);
        trajectory_msg.points.push_back(point);
    }

    RCLCPP_INFO(logger_,
        "Publishing initial JointTrajectory with %d points toward q_target (duration: %.2f s)",
        num_points, duration);

    joint_trajectory_pub_->publish(trajectory_msg);
    trajectory_sent_ = true;
}

} // namespace ur5_controller
