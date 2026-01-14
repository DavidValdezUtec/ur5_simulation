#include <QApplication>
#include <QTimer>
#include "ros2_qt_rviz/simple_viz.hpp"

int main(int argc, char **argv)
{
    // 1. Init Qt y ROS
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("qt_rviz_node");

    // 2. Crear nuestra ventana
    SimpleViz window(node);
    window.resize(800, 600);
    window.show();

    // 3. Loop de integración: ROS dentro de Qt
    // Usamos un QTimer para llamar a spin_some() periódicamente
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&node]() {
        rclcpp::spin_some(node);
    });
    timer.start(10); // Ejecutar cada 10ms (100Hz aprox)

    // 4. Ejecutar la app
    int result = app.exec();

    rclcpp::shutdown();
    return result;
}