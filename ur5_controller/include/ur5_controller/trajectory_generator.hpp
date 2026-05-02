#ifndef UR5_CONTROLLER__TRAJECTORY_GENERATOR_HPP_
#define UR5_CONTROLLER__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>

namespace ur5_controller
{

class TrajectoryGenerator
{
public:
  struct State
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
  };

  static State calculate(
    const Eigen::Vector3d& x_init,
    const Eigen::Vector3d& A,
    double wn,
    double c0,
    double time_elapsed,
    int grafica);
};

}  // namespace ur5_controller

#endif  // UR5_CONTROLLER__TRAJECTORY_GENERATOR_HPP_
