// publicador que reproduce datos guardados por guardar_geo.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <omni_msgs/msg/omni_button_event.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

#include <termios.h>
#include <unistd.h>

struct GeoSample {
	double t;
	double px, py, pz;
	double qw, qx, qy, qz;
};

class GeoSimulationNode : public rclcpp::Node {
public:
	GeoSimulationNode() : Node("geo_simulation"), started_(false), idx_(0) {
		// Parámetro del archivo
		this->declare_parameter<std::string>("file_path", get_default_path());
		this->get_parameter("file_path", file_path_);

		pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/phantom/pose", 10);
		button_pub_ = this->create_publisher<omni_msgs::msg::OmniButtonEvent>("/phantom/button", 10);

		if (!load_file(file_path_)) {
			RCLCPP_ERROR(this->get_logger(), "No se pudieron cargar datos: %s", file_path_.c_str());
		} else {
			RCLCPP_INFO(this->get_logger(), "Cargadas %zu muestras desde %s", samples_.size(), file_path_.c_str());
		}

		// Hilo para lectura de teclado (z/x)
		key_thread_ = std::thread([this]() { keyboard_loop(); });

		// Timer a 1000 Hz
		timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { tick(); });
	}

	~GeoSimulationNode() override {
		stop_keys_ = true;
		if (key_thread_.joinable()) key_thread_.join();
	}

private:
	static std::string get_default_path() {
		const char* home = std::getenv("HOME");
		std::string base = home ? std::string(home) : std::string(".");
		// Por defecto, un CSV en ~/.ros/geomagic_data.csv
		return base + "/.ros/geomagic_data.csv";
	}

	bool load_file(const std::string& path) {
		std::ifstream in(path);
		if (!in.is_open()) return false;
		samples_.clear();
		std::string line;
		// Se espera un CSV con cabecera opcional: t,px,py,pz,qw,qx,qy,qz
		// Ignoramos la primera línea si contiene letras
		bool first = true;
		while (std::getline(in, line)) {
			if (line.empty()) continue;
			if (first) {
				first = false;
				bool has_alpha = false;
				for (char c : line) { if (std::isalpha(static_cast<unsigned char>(c))) { has_alpha = true; break; } }
				if (has_alpha) continue; // saltar cabecera
			}
			std::stringstream ss(line);
			std::string field;
			std::vector<std::string> fields;
			while (std::getline(ss, field, ',')) fields.push_back(field);
			if (fields.size() < 8) continue;
			GeoSample s{};
			try {
				s.t = std::stod(fields[0]);
				s.px = std::stod(fields[2]);
				s.py = std::stod(fields[3]);
				s.pz = std::stod(fields[4]);
				s.qw = std::stod(fields[5]);
				s.qx = std::stod(fields[6]);
				s.qy = std::stod(fields[7]);
				s.qz = std::stod(fields[8]);
				samples_.push_back(s);
			} catch (...) {
				continue;
			}
		}
		return !samples_.empty();
	}

	void publish_pose(const GeoSample& s) {
		geometry_msgs::msg::PoseStamped msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = "phantom_frame";
		msg.pose.position.x = s.px;
		msg.pose.position.y = s.py;
		msg.pose.position.z = s.pz;
		msg.pose.orientation.w = s.qw;
		msg.pose.orientation.x = s.qx;
		msg.pose.orientation.y = s.qy;
		msg.pose.orientation.z = s.qz;
		pose_pub_->publish(msg);
	}

	void publish_button(bool grey, bool white) {
		omni_msgs::msg::OmniButtonEvent ev;
		ev.grey_button = grey ? 1 : 0;
		ev.white_button = white ? 1 : 0;
		button_pub_->publish(ev);
	}

	void tick() {
		if (!started_) return;
		if (samples_.empty()) return;
		const auto& s = samples_[idx_];
		publish_pose(s);
		idx_ = (idx_ + 1) % samples_.size();
	}

	void keyboard_loop() {
		// Configurar terminal en modo no canónico sin eco
		termios oldt{};
		tcgetattr(STDIN_FILENO, &oldt);
		termios newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		newt.c_cc[VMIN] = 0;
		newt.c_cc[VTIME] = 1; // 100 ms
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);

		while (!stop_keys_) {
			char c = 0;
			ssize_t n = read(STDIN_FILENO, &c, 1);
			if (n == 1) {
				if (c == 'z' || c == 'Z') {
					started_ = true;
					publish_button(true, false); // gris
					RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Botón gris (z) detectado: inicio de publicación");
				} else if (c == 'x' || c == 'X') {
					publish_button(false, true); // blanco
					RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Botón blanco (x) detectado");
				}
			}
		}

		// Restaurar modo terminal
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	}

	// Estado
	std::string file_path_;
	std::vector<GeoSample> samples_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::thread key_thread_;
	std::atomic<bool> started_;
	std::atomic<bool> stop_keys_{false};
	size_t idx_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GeoSimulationNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}