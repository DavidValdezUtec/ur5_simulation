#include <ur5_controller/trajectory_generator.hpp>

#include <cmath>

namespace ur5_controller
{

TrajectoryGenerator::State TrajectoryGenerator::calculate(
  const Eigen::Vector3d& x_init,
  const Eigen::Vector3d& A,
  double wn,
  double c0,
  double time_elapsed,
  int grafica)
{
  State state;
  const double t = time_elapsed;

  const double exp_neg_c0_t = std::exp(-c0 * t);
  const double sin_wn_t = std::sin(wn * t);
  const double cos_wn_t = std::cos(wn * t);

  const double amp_factor = 1.0 - exp_neg_c0_t;
  const double d_amp_factor_dt = c0 * exp_neg_c0_t;
  const double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t;

  if (grafica == 1) {
    state.position.x() = x_init.x() + A.x() * amp_factor * sin_wn_t;
    state.position.y() = x_init.y() + A.y() * amp_factor * cos_wn_t;
    state.position.z() = x_init.z() + A.z() * amp_factor * sin_wn_t;

    state.velocity.x() = A.x() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
    state.velocity.y() = A.y() * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
    state.velocity.z() = A.z() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

    const double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
    const double term2_cos = 2.0 * d_amp_factor_dt * wn;
    state.acceleration.x() = A.x() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
    state.acceleration.y() = A.y() * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
    state.acceleration.z() = A.z() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
  } else if (grafica == 2) {
    state.position.x() = x_init.x() + A.x() - A.x() * exp_neg_c0_t;
    state.position.y() = x_init.y() + A.y() - A.y() * exp_neg_c0_t;
    state.position.z() = x_init.z() + A.z() - A.z() * exp_neg_c0_t;

    state.velocity.x() = A.x() * (-c0) * exp_neg_c0_t;
    state.velocity.y() = A.y() * (-c0) * exp_neg_c0_t;
    state.velocity.z() = A.z() * (-c0) * exp_neg_c0_t;

    state.acceleration.x() = A.x() * (c0 * c0) * exp_neg_c0_t;
    state.acceleration.y() = A.y() * (c0 * c0) * exp_neg_c0_t;
    state.acceleration.z() = A.z() * (c0 * c0) * exp_neg_c0_t;
  } else if (grafica == 3) {
    const double r_x = A.x();
    const double r_y = A.y();

    state.position.x() = x_init.x() + r_x * cos_wn_t;
    state.position.y() = x_init.y() + r_y * sin_wn_t;
    state.position.z() = x_init.z();

    state.velocity.x() = -r_x * wn * sin_wn_t;
    state.velocity.y() = r_y * wn * cos_wn_t;
    state.velocity.z() = 0.0;

    state.acceleration.x() = -r_x * wn * wn * cos_wn_t;
    state.acceleration.y() = -r_y * wn * wn * sin_wn_t;
    state.acceleration.z() = 0.0;
  }

  return state;
}

}  // namespace ur5_controller
