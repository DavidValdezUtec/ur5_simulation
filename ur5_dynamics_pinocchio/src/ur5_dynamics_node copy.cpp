#include <rclcpp/rclcpp.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp> // For M, C, G terms
#include <pinocchio/math/rpy.hpp> // For rotation matrices if needed for orientation control

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // Include JointState message type

#include "ur5_dynamics_pinocchio/yaml_loader.hpp"
#include <chrono>
#include <thread>


// For Eigen matrix operations like inverse
#include <Eigen/Dense>

class UR5DynamicsNode : public rclcpp::Node
{
public:
    UR5DynamicsNode() : Node("ur5_dynamics_node")
    {
        // Load URDF model
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("ur5_dynamics_pinocchio");
        std::string urdf_path = pkg_share_dir + "/urdf/ur5.urdf";
        std::string yaml_path = pkg_share_dir + "/configs/joint_config.yaml"; // Initial desired position

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        RCLCPP_INFO(this->get_logger(), "Modelo cargado con %d grados de libertad", model_.nq);

        nq_ = model_.nq; // Number of generalized coordinates
        nv_ = model_.nv; // Number of generalized velocities

        // Initialize desired and current states
        q_desired_ = load_joint_positions(yaml_path, nq_); // Initial desired position from YAML
        q_current_ = Eigen::VectorXd::Zero(nq_); // Current position, updated by subscriber
        v_current_ = Eigen::VectorXd::Zero(nv_); // Current velocity, updated by subscriber

        // For SMC, we need to track our own integrated state, not just rely on Gazebo's current state
        q_integrated_ = q_desired_; // Start integrated position at desired
        v_integrated_ = Eigen::VectorXd::Zero(nv_); // Start integrated velocity at zero

        // Initialize control parameters for Sliding Mode Control
        lambda_ = Eigen::VectorXd::Constant(nv_, 5.0); // Convergence rate to sliding surface
        k_ = Eigen::VectorXd::Constant(nv_, 1.0);     // Gain for reaching phase (SMC robustness)
                                                      // You might need to tune these values!
        dt_ = 0.001; // Integration time step for SMC loop (e.g., 1ms for finer control)
                     // This dt is *different* from the publishing frequency.
                     // The SMC loop will run much faster than the publishing to Gazebo.

        // Create publisher for desired joint trajectory
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        // Create subscriber for actual joint states from Gazebo
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR5DynamicsNode::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriber to /joint_states created.");

        // Set up a timer for the main control loop (SMC calculation and integration)
        // This timer runs at a high frequency (e.g., 1000 Hz or 1ms)
        control_timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(dt_ * 1000000)), // Convert dt_ (seconds) to microseconds
            std::bind(&UR5DynamicsNode::controlLoop, this));

        // Set up a separate timer for publishing the computed trajectory points to Gazebo
        // This should be at a lower frequency than the control loop (e.g., 100 Hz or 10ms)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // Publish every 20ms = 0.02 seconds
            std::bind(&UR5DynamicsNode::publishTrajectoryPoint, this));

        RCLCPP_INFO(this->get_logger(), "UR5 Dynamics Node initialized with SMC control.");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current joint positions and velocities from Gazebo
        // This is crucial for feedback in your control loop
        for (int i = 0; i < int(msg->name.size()); ++i) {
            // Find the index of the joint in Pinocchio's model
            // For robust mapping, you'd use model_.getJointId(msg->name[i])
            // For simplicity, assuming the order matches and only taking the first nq_ joints
            if (i < nq_) {
                q_current_[i] = msg->position[i];
                if (i < int(msg->velocity.size())) {
                    v_current_[i] = msg->velocity[i];
                }
            }
        }
        // RCLCPP_DEBUG(this->get_logger(), "Received JointState. q_current_[0]: %f", q_current_[0]);
    }

     void controlLoop()
    {
        try {
            // 1. Calculate errors and sliding surface 's'
            Eigen::VectorXd e_q = q_integrated_ - q_desired_;
            Eigen::VectorXd e_v = v_integrated_ - Eigen::VectorXd::Zero(nv_);
            Eigen::VectorXd s = e_v + lambda_.cwiseProduct(e_q);

            // 2. Compute dynamic terms using Pinocchio
            pinocchio::computeAllTerms(model_, data_, q_integrated_, v_integrated_);

            // DEBUG PRINTS (keep them for now, remove once stable)
            RCLCPP_INFO(this->get_logger(), "DEBUG: M dims: %ld x %ld", data_.M.rows(), data_.M.cols());
            RCLCPP_INFO(this->get_logger(), "DEBUG: C dims: %ld x %ld", data_.C.rows(), data_.C.cols()); // This is the Coriolis MATRIX (6x6)
            RCLCPP_INFO(this->get_logger(), "DEBUG: g dims: %ld x %ld", data_.g.rows(), data_.g.cols());
            RCLCPP_INFO(this->get_logger(), "DEBUG: e_q dims: %ld x %ld", e_q.rows(), e_q.cols());
            RCLCPP_INFO(this->get_logger(), "DEBUG: e_v dims: %ld x %ld", e_v.rows(), e_v.cols());
            RCLCPP_INFO(this->get_logger(), "DEBUG: s dims: %ld x %ld", s.rows(), s.cols());
            RCLCPP_INFO(this->get_logger(), "DEBUG: k_ dims: %ld x %ld", k_.rows(), k_.cols());
            // END DEBUG PRINTS

            Eigen::MatrixXd M = data_.M;
            RCLCPP_INFO(this->get_logger(), "DEBUG: se asigno M");

            // FIX: Calculate Coriolis vector by multiplying Coriolis matrix (data_.C) by velocity (v_integrated_)
            Eigen::VectorXd C_times_V = data_.C * v_integrated_; // <--- THIS IS THE CHANGE
            RCLCPP_INFO(this->get_logger(), "DEBUG: se asigno C_times_V"); // This line will now be reached

            Eigen::VectorXd G = data_.g;
            RCLCPP_INFO(this->get_logger(), "DEBUG: se asigno G");

            
            /*
            reumen de smc en un ur uu
            tau = M*ddq + C*q + G
            tau = M*(inv(Ja)*(ddx - Ja_dot*q_dot)) + C*q + G
            ddx = Ja*inv(M)*(tau - C*q - G ) + Ja_dot*q_dot (----1-----)
            S = dx - dxdes + lambda*(x - xdes)
            S_dot = ddx -ddxdes + lambda*(dx - dxdes)
            V = 0.5*S^T*S
            V_dot = S^T*(S_dot) -> S_dot = -k*s -k2*tanh(10*s) -> vdot siempre nega
            -k*s -k2*tanh(10*s) = ddx -ddxdes + lambda*(dx - dxdes)
            reemplazando (----1-----)
            -k*s -k2*tanh(10*s) = ddx = Ja*inv(M)*(tau - C*q - G) + Ja_dot*q_dot -ddxdes + lambda*(dx - dxdes)
            tau = M*inv(Ja)*[-k*s -k2*tanh(10*s) - (Ja_dot*q_dot -ddxdes + lambda*(dx - dxdes))] + C*q + G
            done, pero el ur solo recibe pos ._. t.t
            so:
            M*ddq + C*q + G = tau
            M*ddq + C*q + G =  M*inv(Ja)*[-k*s -k2*tanh(10*s) - (Ja_dot*q_dot -ddxdes + lambda*(dx - dxdes))] + C*q + G
            M*ddq = M*inv(Ja)*[-k*s -k2*tanh(10*s) - (Ja_dot*q_dot -ddxdes + lambda*(dx - dxdes))]
            ddq = inv(Ja)*[-ks-k2*tanh(10*s) - (Ja_dot*q_dot -ddxdes + lambda*(dx - dxdes))]
            */
            Eigen::VectorXd tau_smc = M * (-lambda_.cwiseProduct(e_v)) + C_times_V + G - k_.cwiseProduct(s.cwiseSign());

            // 4. Calculate joint accelerations (forward dynamics)
            Eigen::VectorXd q_ddot_calculated = M.llt().solve(tau_smc - C_times_V - G);

            // 5. Numerical Integration
            v_integrated_ = v_integrated_ + q_ddot_calculated * dt_;
            q_integrated_ = q_integrated_ + v_integrated_ * dt_;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Caught exception in controlLoop: %s", e.what());
            rclcpp::shutdown();
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Caught unknown exception in controlLoop.");
            rclcpp::shutdown();
        }
    }

    void publishTrajectoryPoint()
    {
        // Publish the current integrated desired position
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        // Use the integrated position as the target for the controller
        point.positions = std::vector<double>(q_integrated_.data(), q_integrated_.data() + q_integrated_.size());
        
        // This time_from_start tells the controller how quickly to reach this point.
        // Since we are continuously sending new points, a small duration is appropriate.
        // The controller will interpolate to reach this point from its current state.
        point.time_from_start = rclcpp::Duration::from_seconds(0.02); // Publish every 20ms, so 20ms target

        traj_msg.points.push_back(point);

        // Publish the message
        publisher_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Published trajectory point to controller. Target q[0]: %f", q_integrated_[0]);
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    int nq_; // Number of generalized coordinates
    int nv_; // Number of generalized velocities

    // Desired state (target from YAML or trajectory)
    Eigen::VectorXd q_desired_;

    // Current state from Gazebo (feedback)
    Eigen::VectorXd q_current_;
    Eigen::VectorXd v_current_;

    // Integrated state (what our SMC computes as the desired state)
    Eigen::VectorXd q_integrated_;
    Eigen::VectorXd v_integrated_;

    // Control gains
    Eigen::VectorXd lambda_; // For sliding surface
    Eigen::VectorXd k_;      // For reaching law

    double dt_; // Integration time step

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_; // Timer for SMC loop (high frequency)
    rclcpp::TimerBase::SharedPtr publish_timer_; // Timer for publishing to Gazebo (lower frequency)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5DynamicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}