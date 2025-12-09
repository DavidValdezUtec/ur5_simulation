#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

class GeomagicLoggerNode : public rclcpp::Node {
public:
	GeomagicLoggerNode()
	: rclcpp::Node("geomagic_logger") {
		// Parámetros opcionales
		topic_ = this->declare_parameter<std::string>("topic", "/phantom2/pose");
		out_dir_ = this->declare_parameter<std::string>("out_dir", get_default_dir());
		file_prefix_ = this->declare_parameter<std::string>("file_prefix", "geomagic_log");

		// Crear directorio
		std::error_code ec;
		std::filesystem::create_directories(out_dir_, ec);
		if (ec) {
			RCLCPP_WARN(this->get_logger(), "No se pudo crear el directorio '%s': %s",
									out_dir_.c_str(), ec.message().c_str());
		}

		// Abrir archivo con timestamp
		const std::string filepath = out_dir_ + "/" + file_prefix_ + "_" + now_string() + ".csv";
		csv_.open(filepath, std::ios::out);
		if (!csv_.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo CSV: %s", filepath.c_str());
		} else {
			RCLCPP_INFO(this->get_logger(), "Registrando datos en: %s", filepath.c_str());
			write_header();
		}

		// Suscripción
		sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				topic_, rclcpp::SensorDataQoS(),
				std::bind(&GeomagicLoggerNode::pose_callback, this, std::placeholders::_1));
	}

	~GeomagicLoggerNode() override {
		if (csv_.is_open()) {
			csv_.flush();
			csv_.close();
		}
	}

private:
	std::string topic_;
	std::string out_dir_;
	std::string file_prefix_;
	std::ofstream csv_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

	static std::string get_default_dir() {
		const char* home = std::getenv("HOME");
		std::string base = home ? std::string(home) : std::string(".");
		return base + std::string("/.ros/geomagic_sim");
	}

	static std::string now_string() {
		auto now = std::chrono::system_clock::now();
		std::time_t tt = std::chrono::system_clock::to_time_t(now);
		std::tm tm{};
		localtime_r(&tt, &tm);
		char buf[32];
		std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
		return std::string(buf);
	}

	void write_header() {
		csv_ << "stamp_sec,stamp_nanosec"
				 << ",pos_x,pos_y,pos_z"
				 << ",quat_w,quat_x,quat_y,quat_z"
				 << std::endl;
	}

	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
		if (!msg || !csv_.is_open()) return;
		// Tiempo del mensaje
		const auto& st = msg->header.stamp;
		// Posición
		const auto& p = msg->pose.position;
		// Cuaternión
		const auto& q = msg->pose.orientation;
        std::cout<<"Logging Pose: "
                 <<"Position("<<p.x<<", "<<p.y<<", "<<p.z<<"), "
                 <<"Orientation("<<q.w<<", "<<q.x<<", "<<q.y<<", "<<q.z<<")"
                 <<std::endl;

		csv_ << std::fixed << std::setprecision(9)
				 << st.sec << "," << st.nanosec
				 << "," << p.x << "," << p.y << "," << p.z
				 << "," << q.w << "," << q.x << "," << q.y << "," << q.z
				 << std::endl;
	}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GeomagicLoggerNode>());
	rclcpp::shutdown();
	return 0;
}

