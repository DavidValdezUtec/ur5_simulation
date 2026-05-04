#include "ur5_controller/parameter_manager.hpp"
#include "ur5_controller/structs.hpp"
#include <cmath>

namespace ur5_controller {

ParameterManager::ParameterManager(NodeConfig* config,
                                   Eigen::Vector3i* map_pos,
                                   Eigen::Vector3d* sign_pos,
                                   Eigen::Vector3i* map_rot,
                                   Eigen::Vector3d* sign_rot,
                                   std::mutex* node_mutex,
                                   const rclcpp::Logger& logger)
    : config_(config), map_pos_(map_pos), sign_pos_(sign_pos), map_rot_(map_rot), sign_rot_(sign_rot), node_mutex_(node_mutex), logger_(logger)
{
}

bool ParameterManager::validate_map_idx(double v, const std::string& name, rcl_interfaces::msg::SetParametersResult& result) const {
    // Accept doubles but ensure they are integer-valued and in [0,2]
    if (std::isnan(v) || std::isinf(v)) {
        result.successful = false;
        result.reason = name + " debe ser un número finito";
        return false;
    }
    double iv = std::floor(v);
    if (std::abs(v - iv) > 1e-9) {
        result.successful = false;
        result.reason = name + " debe ser un entero (0,1,2)";
        return false;
    }
    int vi = static_cast<int>(iv);
    if (vi < 0 || vi > 2) {
        result.successful = false;
        result.reason = name + " debe estar en [0,2]";
        return false;
    }
    return true;
}

bool ParameterManager::validate_sign(double v, const std::string& name, rcl_interfaces::msg::SetParametersResult& result) const {
    if (!(v == 1.0 || v == -1.0)) {
        result.successful = false;
        result.reason = name + " debe ser 1.0 o -1.0";
        return false;
    }
    return true;
}

rcl_interfaces::msg::SetParametersResult ParameterManager::on_parameters_set(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    // Local copies
    Eigen::Vector3i new_map_pos = *map_pos_;
    Eigen::Vector3d new_sign_pos = *sign_pos_;
    Eigen::Vector3i new_map_rot = *map_rot_;
    Eigen::Vector3d new_sign_rot = *sign_rot_;

    Eigen::Vector3d new_traj_A = config_->traj_A;
    double new_traj_wn = config_->traj_wn;
    double new_traj_c0 = config_->traj_c0;
    int new_traj_mode = config_->traj_mode;
    std::string new_controller = config_->controller;

    auto reject = [&](const std::string& why) {
        result.successful = false;
        result.reason = why;
    };

    for (const auto& p : params) {
        const auto& name = p.get_name();

        if (name == "map_x") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_x debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_pos.x() = static_cast<int>(std::lround(v));
        } else if (name == "map_y") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_y debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_pos.y() = static_cast<int>(std::lround(v));
        } else if (name == "map_z") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_z debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_pos.z() = static_cast<int>(std::lround(v));
        } else if (name == "sign_x") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_x debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_pos.x() = v;
        } else if (name == "sign_y") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_y debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_pos.y() = v;
        } else if (name == "sign_z") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_z debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_pos.z() = v;

        } else if (name == "map_roll") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_roll debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_rot.x() = static_cast<int>(std::lround(v));
        } else if (name == "map_pitch") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_pitch debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_rot.y() = static_cast<int>(std::lround(v));
        } else if (name == "map_yaw") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("map_yaw debe ser double"); break; }
            double v = p.as_double();
            if (!validate_map_idx(v, name, result)) break;
            new_map_rot.z() = static_cast<int>(std::lround(v));
        } else if (name == "sign_roll") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_roll debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_rot.x() = v;
        } else if (name == "sign_pitch") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_pitch debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_rot.y() = v;
        } else if (name == "sign_yaw") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_yaw debe ser double"); break; }
            double v = p.as_double();
            if (!validate_sign(v, name, result)) break;
            new_sign_rot.z() = v;

        } else if (name == "traj_A") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) { reject("traj_A debe ser double[]"); break; }
            auto v = p.as_double_array();
            if (v.size() != 3) { reject("traj_A debe tener tamaño 3"); break; }
            new_traj_A = Eigen::Vector3d(v[0], v[1], v[2]);
        } else if (name == "traj_wn") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("traj_wn debe ser double"); break; }
            new_traj_wn = p.as_double();
        } else if (name == "traj_c0") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("traj_c0 debe ser double"); break; }
            new_traj_c0 = p.as_double();
        } else if (name == "traj_mode") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("traj_mode debe ser int"); break; }
            int v = static_cast<int>(p.as_int());
            if (v != 1 && v != 2) { reject("traj_mode debe ser 1 o 2"); break; }
            new_traj_mode = v;

        } else if (name == "controller_type") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) { reject("controller_type debe ser string"); break; }
            auto v = p.as_string();
            if (!(v == "QP" || v == "IMP" || v == "SLD")) { reject("controller_type debe ser 'QP', 'IMP' o 'SLD'"); break; }
            new_controller = v;
        }
    }

    if (!result.successful) {
        return result;
    }

    {
        if (node_mutex_) {
            std::scoped_lock<std::mutex> lock(*node_mutex_);
            *map_pos_ = new_map_pos;
            *sign_pos_ = new_sign_pos;
            *map_rot_ = new_map_rot;
            *sign_rot_ = new_sign_rot;
            config_->traj_A = new_traj_A;
            config_->traj_wn = new_traj_wn;
            config_->traj_c0 = new_traj_c0;
            config_->traj_mode = new_traj_mode;
            config_->controller = new_controller;
        } else {
            // Fallback: no mutex provided
            *map_pos_ = new_map_pos;
            *sign_pos_ = new_sign_pos;
            *map_rot_ = new_map_rot;
            *sign_rot_ = new_sign_rot;
            config_->traj_A = new_traj_A;
            config_->traj_wn = new_traj_wn;
            config_->traj_c0 = new_traj_c0;
            config_->traj_mode = new_traj_mode;
            config_->controller = new_controller;
        }
    }

    RCLCPP_INFO(logger_,
                "Parametros actualizados: map_pos=[%d,%d,%d] sign_pos=[%.0f,%.0f,%.0f] map_rot=[%d,%d,%d] sign_rot=[%.0f,%.0f,%.0f] traj_mode=%d controller=%s",
                map_pos_->x(), map_pos_->y(), map_pos_->z(),
                sign_pos_->x(), sign_pos_->y(), sign_pos_->z(),
                map_rot_->x(), map_rot_->y(), map_rot_->z(),
                sign_rot_->x(), sign_rot_->y(), sign_rot_->z(),
                config_->traj_mode,
                config_->controller.c_str());

    return result;
}

} // namespace ur5_controller
