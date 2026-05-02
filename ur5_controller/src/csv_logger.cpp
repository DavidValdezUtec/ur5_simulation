#include <ur5_controller/csv_logger.hpp>

#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <utility>

namespace ur5_controller
{

CsvLogger::CsvLogger(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

void CsvLogger::configure(bool enabled,
                          const std::string& csv_dir,
                          const std::string& csv_prefix,
                          const std::string& namespace_name)
{
  enabled_ = enabled;
  if (!enabled_) {
    return;
  }

  std::string output_dir = csv_dir;
  if (output_dir.empty()) {
    output_dir = getHomeDir() + std::string("/.ros/ur5_logs");
  }

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    RCLCPP_WARN(logger_, "No se pudo crear el directorio de CSV '%s': %s", output_dir.c_str(), ec.message().c_str());
  }

  const std::string ns_part = namespace_name.empty() ? std::string("nonamespace") : namespace_name;
  const std::string file_name = csv_prefix + std::string("_") + ns_part + std::string("_") + timestampString() + std::string(".csv");
  file_path_ = (std::filesystem::path(output_dir) / file_name).string();

  file_.open(file_path_, std::ios::out);
  if (!file_.is_open()) {
    RCLCPP_ERROR(logger_, "No se pudo abrir archivo CSV: %s", file_path_.c_str());
    enabled_ = false;
    return;
  }

  file_ << "t,dt"
        << ",q_meas_0,q_meas_1,q_meas_2,q_meas_3,q_meas_4,q_meas_5"
        << ",q_cmd_0,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5"
        << ",e_q_0,e_q_1,e_q_2,e_q_3,e_q_4,e_q_5"
        << ",x_des_x,x_des_y,x_des_z"
        << ",x_meas_x,x_meas_y,x_meas_z"
        << ",q_des_w,q_des_x,q_des_y,q_des_z"
        << ",q_meas_w,q_meas_x,q_meas_y,q_meas_z"
        << ",euler_des_r,euler_des_p,euler_des_y"
        << ",euler_meas_r,euler_meas_p,euler_meas_y"
        << ",e_R_angle"
        << ",pos_err_x,pos_err_y,pos_err_z"
        << ",ori_err_axis_x,ori_err_axis_y,ori_err_axis_z,ori_err_angle"
        << ",u_control_0,u_control_1,u_control_2,u_control_3,u_control_4,u_control_5"
        << ",ik_ms,loop_ms"
        << std::endl;

  RCLCPP_INFO(logger_, "CSV logging habilitado: %s", file_path_.c_str());
}

void CsvLogger::writeRow(const CsvRow& row)
{
  if (!enabled_ || !file_.is_open()) {
    return;
  }

  file_ << std::fixed << std::setprecision(6)
        << row.t << "," << row.dt;

  for (int i = 0; i < 6; ++i) file_ << "," << row.q_meas_joints[i];
  for (int i = 0; i < 6; ++i) file_ << "," << row.q_cmd[i];
  for (int i = 0; i < 6; ++i) file_ << "," << (row.q_cmd[i] - row.q_meas_joints[i]);

  file_ << "," << row.x_des.x() << "," << row.x_des.y() << "," << row.x_des.z();
  file_ << "," << row.x_meas.x() << "," << row.x_meas.y() << "," << row.x_meas.z();
  file_ << "," << row.q_des.w() << "," << row.q_des.x() << "," << row.q_des.y() << "," << row.q_des.z();
  file_ << "," << row.q_meas_pose.w() << "," << row.q_meas_pose.x() << "," << row.q_meas_pose.y() << "," << row.q_meas_pose.z();
  file_ << "," << row.euler_des.x() << "," << row.euler_des.y() << "," << row.euler_des.z();
  file_ << "," << row.euler_meas.x() << "," << row.euler_meas.y() << "," << row.euler_meas.z();
  file_ << "," << row.e_R_angle;
  file_ << "," << row.pos_err.x() << "," << row.pos_err.y() << "," << row.pos_err.z();
  file_ << "," << row.ori_err_axis.x() << "," << row.ori_err_axis.y() << "," << row.ori_err_axis.z() << "," << row.ori_err_angle;
  for (int i = 0; i < 6; ++i) file_ << "," << row.u_control[i];
  file_ << "," << row.ik_ms << "," << row.loop_ms;
  file_ << std::endl;
}

bool CsvLogger::isEnabled() const
{
  return enabled_;
}

const std::string& CsvLogger::filePath() const
{
  return file_path_;
}

std::string CsvLogger::getHomeDir()
{
  const char* home = std::getenv("HOME");
  return home ? std::string(home) : std::string(".");
}

std::string CsvLogger::timestampString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&tt, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

}  // namespace ur5_controller