#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>

inline Eigen::VectorXd load_joint_positions(const std::string &yaml_path, std::size_t dof)
{
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config["joint_positions"]) {
        throw std::runtime_error("No 'joint_positions' key found in YAML file.");
    }

    std::vector<double> positions = config["joint_positions"].as<std::vector<double>>();

    if (positions.size() != dof) {
        throw std::runtime_error("Size mismatch between model DOF and joint_positions vector.");
    }

    Eigen::VectorXd q(dof);
    for (std::size_t i = 0; i < dof; ++i) {
        q[i] = positions[i];
    }

    return q;
}
