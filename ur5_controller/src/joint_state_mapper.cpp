#include <ur5_controller/joint_state_mapper.hpp>

#include <algorithm>
#include <sstream>
#include <unordered_map>

namespace ur5_controller
{

JointStateMapper::JointStateMapper() = default;

void JointStateMapper::configure(const std::string& namespace_name, rclcpp::Logger logger)
{
  namespace_name_ = namespace_name;
  logger_ = std::move(logger);
}

bool JointStateMapper::updateOrderedState(const sensor_msgs::msg::JointState::SharedPtr& msg,
                                           Eigen::VectorXd& q,
                                           Eigen::VectorXd& qd)
{
  if (!msg) {
    RCLCPP_WARN(logger_, "Mensaje JointState nulo recibido");
    return false;
  }

  if (q.size() != 6) {
    q = Eigen::VectorXd::Zero(6);
  }
  if (qd.size() != 6) {
    qd = Eigen::VectorXd::Zero(6);
  }

  if (!joint_map_initialized_ || !sameNameList(msg->name, last_js_names_)) {
    rebuildJointIndexMap(msg);
  }

  if (!joint_map_initialized_) {
    RCLCPP_WARN(logger_, "Esperando mapeo válido de joints para reordenar JointState");
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    const int joint_index = joint_index_map_[static_cast<size_t>(i)];
    if (joint_index < 0 || static_cast<size_t>(joint_index) >= msg->position.size()) {
      RCLCPP_WARN(logger_, "Índice de joint fuera de rango para posiciones: %d", joint_index);
      return false;
    }

    q[i] = msg->position[static_cast<size_t>(joint_index)];
    if (static_cast<size_t>(joint_index) < msg->velocity.size()) {
      qd[i] = msg->velocity[static_cast<size_t>(joint_index)];
    } else {
      qd[i] = 0.0;
    }
  }

  return true;
}

bool JointStateMapper::isReady() const
{
  return joint_map_initialized_;
}

std::vector<std::string> JointStateMapper::getExpectedJointNames() const
{
  const std::string prefix = namespace_name_.empty() ? std::string("") : (namespace_name_ + std::string("_"));
  return {
    prefix + "shoulder_pan_joint",
    prefix + "shoulder_lift_joint",
    prefix + "elbow_joint",
    prefix + "wrist_1_joint",
    prefix + "wrist_2_joint",
    prefix + "wrist_3_joint"
  };
}

std::vector<std::string> JointStateMapper::getExpectedBaseJointNames() const
{
  return {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
}

bool JointStateMapper::endsWith(const std::string& str, const std::string& suffix)
{
  if (suffix.size() > str.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

bool JointStateMapper::sameNameList(const std::vector<std::string>& a,
                                    const std::vector<std::string>& b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

void JointStateMapper::rebuildJointIndexMap(const sensor_msgs::msg::JointState::SharedPtr& msg)
{
  joint_map_initialized_ = false;
  joint_index_map_ = {{-1, -1, -1, -1, -1, -1}};
  last_js_names_ = msg->name;

  const auto expected = getExpectedJointNames();
  const auto expected_base = getExpectedBaseJointNames();

  std::unordered_map<std::string, int> name_to_idx;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    name_to_idx[msg->name[i]] = static_cast<int>(i);
  }

  int found_exact = 0;
  for (int i = 0; i < 6; ++i) {
    const auto it = name_to_idx.find(expected[static_cast<size_t>(i)]);
    if (it != name_to_idx.end()) {
      joint_index_map_[static_cast<size_t>(i)] = it->second;
      ++found_exact;
    }
  }

  if (found_exact < 6) {
    for (int i = 0; i < 6; ++i) {
      if (joint_index_map_[static_cast<size_t>(i)] != -1) {
        continue;
      }
      const auto itb = name_to_idx.find(expected_base[static_cast<size_t>(i)]);
      if (itb != name_to_idx.end()) {
        joint_index_map_[static_cast<size_t>(i)] = itb->second;
        ++found_exact;
      }
    }
  }

  if (found_exact < 6) {
    for (int i = 0; i < 6; ++i) {
      if (joint_index_map_[static_cast<size_t>(i)] != -1) {
        continue;
      }
      for (size_t j = 0; j < msg->name.size(); ++j) {
        if (endsWith(msg->name[j], expected_base[static_cast<size_t>(i)])) {
          joint_index_map_[static_cast<size_t>(i)] = static_cast<int>(j);
          ++found_exact;
          break;
        }
      }
    }
  }

  if (found_exact == 6) {
    joint_map_initialized_ = true;
    std::ostringstream map_info;
    map_info << "Mapa de joints establecido (idx en /joint_states): ";
    for (int i = 0; i < 6; ++i) {
      map_info << joint_index_map_[static_cast<size_t>(i)] << (i < 5 ? "," : "");
    }
    RCLCPP_INFO(logger_, "%s", map_info.str().c_str());
  } else {
    RCLCPP_WARN(logger_, "No se pudo establecer el mapeo de joints por nombre. Recibidos %zu nombres.", msg->name.size());
  }
}

}  // namespace ur5_controller