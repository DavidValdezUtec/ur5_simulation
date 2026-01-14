#ifndef SIMPLE_VIZ_HPP
#define SIMPLE_VIZ_HPP

#include <QWidget>
#include <rclcpp/rclcpp.hpp>

namespace rviz_common {
    class RenderPanel;
    class VisualizationManager;
    namespace ros_integration {
        class RosNodeAbstraction;
    }
}

class SimpleViz : public QWidget
{
    Q_OBJECT

public:
    SimpleViz(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SimpleViz() override;

protected:
    // Sobrescribimos este evento para iniciar RViz solo cuando sea seguro
    void showEvent(QShowEvent* event) override;

private:
    bool initialized_ = false; // Bandera de control
    rclcpp::Node::SharedPtr node_;

    rviz_common::RenderPanel* render_panel_;
    rviz_common::VisualizationManager* manager_;
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_abstraction_;
};

#endif // SIMPLE_VIZ_HPP